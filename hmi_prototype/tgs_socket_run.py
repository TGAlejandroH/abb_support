"""Socket-start run of a REAL .tgs program under Production Manager - VC in the loop.

Design: TGuideWeldingHMI/docs/socket_start_trigger_hmi_plan_v1.md, Stage R
residuals. The earlier stand-in (socket_start_stub.py) proved arming,
dispatch, part id, return-to-PM and wire-loss recovery with NO program run.
This one runs the program the HMI would run, so it also proves:

  * id-10 file transfer + Load \\Dynamic + late-bound call INSIDE a
    PM-dispatched part (every production cycle takes this path);
  * the station guard: PM's dispatched station, carried in the START line,
    checked against the .tgs work_zone stamp before anything runs (V-17);
  * with --abort-at-capture, the R-5 residual: the HMI dies while the robot
    is AWAY from home (at a capture pose), not parked at its safe pose.

What it reads from the .tgs is exactly what the real HMI reads: the program
text embedded under ROBOT_PROGRAM/<name>.mod (transferred verbatim - the HMI
does not render RAPID; curobo's AbbTranslator does, at planning time) and the
project_meta work_zone stamp.

Answers are the no-localization answers, so the robot runs the PLANNED path:
  * weld frame  = the program's own nominal wobj oframe (TG_ReqWeldFrame
    writes WObj.oframe := reply ABSOLUTE; identity would put the part at the
    world origin and the program's own 100 mm guard would abort);
  * cam frame   = identity (no motion uses wobjTG_Cam);
  * touch-ups   = 0, 0, 0;
  * dry run     = 1 - NOTE this is a NO-OP on ABB today: TG_DryRunOn in
    TG_Cell.sys is an intentionally empty placeholder, so it does not inhibit
    the arc. Harmless on the VC's simulated welder; see finding V-18.

Usage:
    python tgs_socket_run.py [--cycles N] [--abort-at-capture] [--tgs PATH] [--mod PATH]

--mod replaces the .tgs's embedded program with a hand-edited module (same program name), for
RAPID-only VC experiments that come before an exporter change. project_meta still comes from --tgs.
"""

import json
import os
import pathlib
import re
import socket
import sqlite3
import struct
import sys
import time

from abb_server import ACK, AbbTgsHmi, fmt_real
from rws_session import RwsSession

HERE = os.path.dirname(os.path.abspath(__file__))
TGS_PATH = r"C:\Users\TG_Laptop08\Downloads\abb_coordinated_v5_regenerated - Copy.tgs"
RUN_DIR = os.path.join(HERE, "tgs_run")
STATE_FILE = os.path.join(HERE, ".socket_start_stub_state.json")   # shared with the stub
URL = "http://127.0.0.1:80"

# Stand-in for the work-selection map (S11 / D47). Both ids resolve to the SAME
# station-1 project on purpose: dispatching 9022 for station 2 must be REFUSED
# by the station guard, not run.
PART_MAP = {"9011": "TfCItn7EEr", "9022": "TfCItn7EEr"}


class HmiDied(Exception):
    """Raised after the deliberate RST in --abort-at-capture mode."""


def ts():
    return time.strftime("%H:%M:%S") + ".%03d" % int((time.time() % 1) * 1000)


def log(msg):
    print("[RUN %s] %s" % (ts(), msg), flush=True)


def load_tgs(path, mod_override=None):
    """-> (program name, module bytes, work_zone, [(wobj, nominal oframe literal), ...], agnostic)"""
    uri = pathlib.Path(path).as_uri() + "?mode=ro"
    con = sqlite3.connect(uri, uri=True)
    try:
        meta = dict(con.execute("select key, value from project_meta"))
        rows = con.execute(
            "select path, content_blob from robot_bundle_files "
            "where path like 'ROBOT_PROGRAM/%.mod' and path not like 'ROBOT_PROGRAM/baseline/%'"
        ).fetchall()
    finally:
        con.close()
    if len(rows) != 1:
        raise RuntimeError("expected exactly one current ROBOT_PROGRAM module, found %r"
                           % [r[0] for r in rows])
    bundle_path, blob = rows[0]
    if mod_override:
        with open(mod_override, "rb") as fh:
            blob = fh.read()
        bundle_path = mod_override
    name = os.path.splitext(os.path.basename(bundle_path))[0]
    text = blob.decode("utf-8", "replace")
    oframes = dict(re.findall(r"(wobj\w+)\.oframe:=(\[\[[^\]]+\],\[[^\]]+\]\])", text))
    order = re.findall(r"TG_ReqWeldFrame[^;]*\\WObj:=(wobj\w+)", text)
    frames = [(w, oframes[w]) for w in order]
    # The stamp decides, never the RAPID text (owner, 2026-09-23; curobo stamp plan D12):
    # work_zone 0 = any station -> no station guard. Same reading rule as the stamp contract:
    # absent, malformed or out of range reads as zone 1.
    try:
        zone = int(round(float(str(meta.get("work_zone", "1")).strip())))
    except ValueError:
        zone = 1
    if zone < 0 or zone > 10:
        zone = 1
    agnostic = zone == 0
    return name, blob, zone, frames, agnostic


def parse_start(line):
    toks = line.strip().split()
    if not toks or toks[0] != "START":
        return None
    try:
        seq = int(toks[1])
    except (IndexError, ValueError):
        log("MALFORMED START %r - ignoring" % line.strip())
        return None
    part = toks[2] if len(toks) > 2 else "0"
    stn = int(toks[3]) if len(toks) > 3 and toks[3].lstrip("-").isdigit() else 0
    return seq, part, stn


def load_last_seq():
    try:
        with open(STATE_FILE) as fh:
            return int(json.load(fh)["last_seq"])
    except Exception:
        return 0


def save_last_seq(seq):
    with open(STATE_FILE, "w") as fh:
        json.dump({"last_seq": seq}, fh)


class SocketStartRun(AbbTgsHmi):
    def __init__(self, tgs, abort_at_capture=False):
        super().__init__("127.0.0.1", 2000, program_selection=1, verbose=True,
                         rws=RwsSession(URL))
        self.prog_name, blob, self.project_zone, self.weld_frames, self.station_agnostic = tgs
        os.makedirs(RUN_DIR, exist_ok=True)
        with open(os.path.join(RUN_DIR, self.prog_name + ".mod"), "wb") as fh:
            fh.write(blob)
        self.tgs_source_dir = RUN_DIR          # what _transfer_tgs_module sends
        self.cam_frame_xyzwpr = [0.0] * 6
        self.touchup_offsets_in = [0.0, 0.0, 0.0]
        self.dry_run = 1                       # no-op on ABB today - V-18
        # Miller EIP rejects schedule 0 on every arc segment (80001 "Requested program number is
        # less than 1", defaults to program 1) - seen 2026-09-23. abb_server's bare default is 0;
        # MONARC uses schedules 1, 2 and 4.
        self.weld_sched = 1
        self.abort_at_capture = abort_at_capture
        self.last_seq = load_last_seq()
        self._frame_idx = 0
        self.decision = None

    def _log(self, msg):
        print("[RUN %s] %s" % (ts(), msg), flush=True)

    # -- connect loop replaces the green button (S1) ------------------------
    def connect(self, retry_seconds=None):
        refused = 0
        while True:
            try:
                s = socket.create_connection((self.host, self.port), timeout=2.0)
                s.settimeout(120.0)            # longest silent stretch is the weld
                self.sock = s
                log("CONNECTED - listener is up, PM dispatched a TG part (after %d refusals)" % refused)
                return
            except OSError:
                refused += 1
                if refused % 20 == 1:
                    log("  (robot not listening - %d refusals)" % refused)
                time.sleep(0.5)

    # -- S12 lookahead + S9 + station guard -----------------------------------
    def serve_program_selection(self):
        self._frame_idx = 0
        first = self._recv()
        log("  robot -> %r" % first)
        start = parse_start(first)
        if start is None:
            log("  no START line -> legacy robot; this IS the program-id prompt")
            self.sock.sendall(str(self.program_selection).encode())
            return
        self.sock.sendall(ACK)
        seq, part, stn = start

        sel, why = 1, "run"
        if seq == self.last_seq:
            sel, why = 2, "REPLAY of cycle %d - not re-running it" % seq
        else:
            if seq < self.last_seq:
                log("COUNTER RESET: robot %d < last %d - adopting and continuing" % (seq, self.last_seq))
            project = PART_MAP.get(part)
            if project is None:
                sel, why = 2, "unknown part id %s (S5: refuse, no latch)" % part
            elif project != self.prog_name:
                sel, why = 2, "part %s maps to %s, not the loaded %s" % (part, project, self.prog_name)
            elif stn != self.project_zone and not self.station_agnostic:
                sel, why = 2, ("STATION GUARD: PM dispatched part %s for STATION %d, but %s is "
                               "stamped work_zone %d - running it would drive the station on the "
                               "operator's side" % (part, stn, self.prog_name, self.project_zone))
            save_last_seq(seq)
            self.last_seq = seq
        if sel == 1 and self.station_agnostic:
            why = "run (work_zone 0 = any station; station %d, TG_ActMechUnit picks the unit)" % stn
        self.decision = (seq, part, stn, sel, why)
        log(">>> START seq=%d part=%s station=%d -> %s" % (seq, part, stn,
            ("RUN %s" % self.prog_name) if sel == 1 else ("REFUSED: " + why)))
        self.do_send(str(sel))

    # -- no-localization answers --------------------------------------------
    def handle_weld_frame_req(self):
        self._recv_pose_and_sub()
        wobj, literal = self.weld_frames[min(self._frame_idx, len(self.weld_frames) - 1)]
        self._frame_idx += 1
        log("  weld frame -> nominal %s.oframe %s" % (wobj, literal))
        self.do_send(literal)
        self.do_send(str(self.weld_status))
        for value in self.touchup_offsets_in:
            self.do_send(fmt_real(value))

    def handle_cam_frame_req(self):
        if self.abort_at_capture:
            self._recv_pose_and_sub()
            log("  !!! robot is AT a capture pose, away from home - simulating an HMI crash (TCP RST)")
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
            self.sock.close()
            self.sock = None
            raise HmiDied()
        super().handle_cam_frame_req()


def main(argv):
    cycles = int(argv[argv.index("--cycles") + 1]) if "--cycles" in argv else 999
    abort = "--abort-at-capture" in argv
    tgs_path = argv[argv.index("--tgs") + 1] if "--tgs" in argv else TGS_PATH
    mod = argv[argv.index("--mod") + 1] if "--mod" in argv else None
    tgs = load_tgs(tgs_path, mod)
    name, blob, zone, frames, agnostic = tgs
    log("loaded %s: program %s (%d bytes%s), work_zone %d, weld frames %s, station-agnostic %s"
        % (os.path.basename(tgs_path), name, len(blob), (", HAND-EDITED module " + mod) if mod else "",
           zone, [w for w, _ in frames], agnostic))
    run = SocketStartRun(tgs, abort_at_capture=abort)
    log("persisted last_seq = %d; abort-at-capture = %s" % (run.last_seq, abort))
    for i in range(cycles):
        try:
            run.serve_cycle()
        except HmiDied:
            log("HMI 'died' as intended. Stopping; inspect the controller now.")
            return 0
        except Exception as exc:
            log("cycle ended with %r" % (exc,))
        log("CYCLE DONE: decision=%s requests=%s" % (run.decision, run.request_log))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv))
    except KeyboardInterrupt:
        print()
