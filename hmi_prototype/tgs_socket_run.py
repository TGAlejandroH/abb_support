"""Socket-start run of a REAL .tgs program under Production Manager - VC in the loop.

Design: TGuideWeldingHMI/docs/socket_start_trigger_hmi_plan_v1.md, Stage R and Stage R2.
The earlier stand-in (socket_start_stub.py) proved arming, dispatch, part id, return-to-PM and
wire-loss recovery with NO program run. This one runs the program the HMI would run, so it
also proves:

  * the two-port handshake (S14-S17, 2026-09-24): the connect loop targets the HANDSHAKE port;
    the robot sends START, STNFRAME + pose, STNAXES and prompts for a verdict; both sides close;
    only then does the robot listen on the protocol port and the run proceeds exactly as before
    the handshake existed (abb_server.AbbTgsHmi.handshake / serve_cycle);
  * the verdict (S15): part id -> project through the map, the S9 sequence rule, the station
    guard against the .tgs work_zone stamp (0 = any station, Q-27) - all decided BEFORE the
    verdict is answered, so a refusal ends the part gracefully on the robot side (S17);
  * id-10 file transfer + Load \\Dynamic + late-bound call INSIDE a PM-dispatched part;
  * with --abort-at-capture, the R-5 residual: the HMI dies while the robot is AWAY from home.

What it reads from the .tgs is exactly what the real HMI reads: the program text embedded under
ROBOT_PROGRAM/<name>.mod (transferred verbatim) and the project_meta work_zone stamp.

Answers are the no-localization answers, so the robot runs the PLANNED path (weld frame = the
program's own nominal wobj oframe; cam frame = identity; touch-ups = 0; dry run = 1, which is a
NO-OP on ABB today - finding V-18).

Every handshake's STNFRAME / STNAXES is written to tgs_run/stnframe_seq<N>_stn<S>.json for the
V-45 comparison against vc_probes/stn<S>_T2.json (probe_math.py).

Usage:
    python tgs_socket_run.py [--cycles N] [--abort-at-capture] [--tgs PATH] [--mod PATH]
                             [--refuse-part ID] [--last-seq N]

--mod replaces the .tgs's embedded program with a hand-edited module (same program name).
--refuse-part removes that part id from the map, so its START is refused (V-41).
--last-seq overrides the persisted last_seq before the first cycle (V-43: set it to the next
  START's number to see a "replay" refused, or far above it to see a "counter reset" adopted).
--probe-run-port: during every handshake, before the verdict, try to connect to the RUN port
  (2000). S14's promise is that it is refused: 2000 is not listening until the handshake is
  done (V-44).
--drop-in-handshake: close the handshake connection with a TCP RST before answering the
  verdict, to record what the controller does with a client that vanishes mid-handshake (V-47).
--park-and-release: answer the verdict 1, then do NOT serve the run - the robot parks at
  TG_SocketCom's SocketAccept on the run port - and after a few seconds end that cycle the way
  the HMI's ReleaseParkedRun does: connect on 2000, take "Give me the program ID", answer 0 (an
  id TG_ReqProgSel does not know -> "unknown program ID - ending cycle"), read to end-of-file.
  Proves the RAPID side of the release (socket plan V-58); the HMI's client side is the
  handshake harness `parked` scenario.
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

from abb_server import ACK, AbbTgsHmi, ConnectionClosedError, fmt_real
from rws_session import RwsSession

HERE = os.path.dirname(os.path.abspath(__file__))
TGS_PATH = r"C:\Users\TG_Laptop08\Downloads\abb_coordinated_v5_regenerated - Copy.tgs"
RUN_DIR = os.path.join(HERE, "tgs_run")
STATE_FILE = os.path.join(HERE, ".socket_start_stub_state.json")   # shared with the stub
URL = os.environ.get("TG_VC_RWS_URL", "http://127.0.0.1:80")   # the VC is not always on 80

# Stand-in for the work-selection map (S11 / S18): part id -> project (program name). The real
# HMI loads the mapped project; this runner has ONE project loaded, so both VC test parts map
# to it and --refuse-part takes one away to exercise the unknown-id refusal.
VC_PART_IDS = ("9011", "9022")


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
    def __init__(self, tgs, abort_at_capture=False, refuse_part=None,
                 probe_run_port=False, drop_in_handshake=False, park_and_release=False):
        super().__init__("127.0.0.1", 2000, program_selection=1, verbose=True,
                         rws=RwsSession(URL))
        self.probe_run_port = probe_run_port
        self.drop_in_handshake = drop_in_handshake
        self.park_and_release = park_and_release
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
        refused = set(str(refuse_part).split(",")) if refuse_part else set()   # "9022" or "9011,9022"
        self.part_map = {pid: self.prog_name for pid in VC_PART_IDS if pid not in refused}
        self.last_seq = load_last_seq()
        self._frame_idx = 0
        self.decision = None

    def _log(self, msg):
        print("[RUN %s] %s" % (ts(), msg), flush=True)

    # -- V-58: the release of a parked run (the HMI's ReleaseParkedRun, byte for byte) ----

    PARK_SECONDS = 4.0

    def serve_cycle(self):
        if not self.park_and_release:
            return super().serve_cycle()
        self.request_log = []
        self.last_start = self.handshake()
        if self.last_start["verdict"] != 1:
            self._log("handshake: refused - the robot ends the part, no run this cycle")
            return
        self._log("V-58: verdict 1 sent and NOT serving the run - the robot now parks at "
                  "TG_SocketCom's SocketAccept on port %d; waiting %.0f s" % (self.port, self.PARK_SECONDS))
        time.sleep(self.PARK_SECONDS)
        t0 = time.monotonic()
        self.connect(retry_seconds=10.0)
        prompt = self._recv()
        self._log("  robot prompts %r" % prompt)
        if "program ID" not in prompt:
            self._log("  !!! V-58 FAIL: expected the program-id prompt")
        self.sock.sendall(b"0")
        self._log("  hmi   -> '0' (an id TG_ReqProgSel does not know)")
        try:
            trailing = self._recv()
            self._log("  !!! unexpected data after the reply: %r" % trailing)
        except ConnectionClosedError:
            self._log("  robot closed the connection %.2f s after the connect - the cycle ended "
                      "gracefully (TPWrite 'TG: unknown program ID - ending cycle', TG_SocketDisc)"
                      % (time.monotonic() - t0))
        finally:
            self.close()
        self.request_log.append("release")

    # -- the connect loop IS the arming (S1), now on the handshake port (S14) ----
    def handshake_connect(self, retry_seconds=None):
        refused = 0
        while True:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                s.settimeout(self.HANDSHAKE_TIMEOUT)
                s.connect((self.host, self.handshake_port))
                self.sock = s
                log("HANDSHAKE CONNECTED on port %d - PM dispatched a TG part (after %d refusals)"
                    % (self.handshake_port, refused))
                return
            except OSError:
                s.close()
                self.sock = None
                refused += 1
                if refused % 20 == 1:
                    log("  (robot not listening on %d - %d refusals)" % (self.handshake_port, refused))
                time.sleep(0.5)

    # -- the verdict (S15): S9 sequence rule + map + station guard, all before answering ----
    def decide_start(self, event):
        self._frame_idx = 0
        seq, part, stn = event["seq"], event["part"], event["station"]
        log("  START seq=%d part=%s station=%d | STNFRAME %s | STNAXES tilt=%s chuck=%s"
            % (seq, part, stn, event["station_frame"], event["tilt_deg"], event["chuck_deg"]))
        with open(os.path.join(RUN_DIR, "stnframe_seq%d_stn%d.json" % (seq, stn)), "w") as fh:
            json.dump({"seq": seq, "part": part, "station": stn,
                       "station_frame": event["station_frame"],
                       "tilt_deg": event["tilt_deg"], "chuck_deg": event["chuck_deg"],
                       "time": time.strftime("%Y-%m-%d %H:%M:%S")}, fh, indent=1)

        if self.probe_run_port:
            # V-44: the run port must be closed while the handshake is open.
            probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            probe.settimeout(0.5)
            try:
                probe.connect((self.host, self.port))
                log("  !!! V-44 FAIL: port %d ACCEPTED a connection during the handshake" % self.port)
            except OSError as exc:
                log("  V-44: connect to port %d during the handshake -> refused (%s)" % (self.port, exc.__class__.__name__))
            finally:
                probe.close()
        if self.drop_in_handshake:
            log("  !!! V-47: dropping the handshake connection (TCP RST) before the verdict")
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
            self.sock.close()
            self.sock = None
            raise HmiDied()

        sel, why = 1, "run"
        if seq == self.last_seq:
            sel, why = 0, "REPLAY of cycle %d - not re-running it (S9 ==)" % seq
        else:
            if seq < self.last_seq:
                log("COUNTER RESET: robot %d < last %d - adopting and continuing (S9 <)" % (seq, self.last_seq))
            project = self.part_map.get(part)
            if project is None:
                sel, why = 0, "unknown part id %s (S5: refuse, no latch)" % part
            elif project != self.prog_name:
                sel, why = 0, "part %s maps to %s, not the loaded %s" % (part, project, self.prog_name)
            elif stn != self.project_zone and not self.station_agnostic:
                sel, why = 0, ("STATION GUARD: PM dispatched part %s for STATION %d, but %s is "
                               "stamped work_zone %d - running it would drive the station on the "
                               "operator's side" % (part, stn, self.prog_name, self.project_zone))
            save_last_seq(seq)
            self.last_seq = seq
        if sel == 1 and self.station_agnostic:
            why = "run (work_zone 0 = any station; station %d, TG_ActMechUnit picked the unit)" % stn
        self.decision = (seq, part, stn, sel, why)
        log(">>> VERDICT for seq=%d part=%s station=%d -> %s" % (seq, part, stn,
            ("RUN %s" % self.prog_name) if sel == 1 else ("REFUSE: " + why)))
        return sel

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
    refuse_part = argv[argv.index("--refuse-part") + 1] if "--refuse-part" in argv else None
    probe_run_port = "--probe-run-port" in argv
    drop_in_handshake = "--drop-in-handshake" in argv
    park_and_release = "--park-and-release" in argv
    if "--last-seq" in argv:
        save_last_seq(int(argv[argv.index("--last-seq") + 1]))
    tgs = load_tgs(tgs_path, mod)
    if "--zone" in argv:
        # Force the stamp the station guard reads (V-42 without a second project file).
        forced = int(argv[argv.index("--zone") + 1])
        tgs = (tgs[0], tgs[1], forced, tgs[3], forced == 0)
    name, blob, zone, frames, agnostic = tgs
    log("loaded %s: program %s (%d bytes%s), work_zone %d, weld frames %s, station-agnostic %s"
        % (os.path.basename(tgs_path), name, len(blob), (", HAND-EDITED module " + mod) if mod else "",
           zone, [w for w, _ in frames], agnostic))
    run = SocketStartRun(tgs, abort_at_capture=abort, refuse_part=refuse_part,
                         probe_run_port=probe_run_port, drop_in_handshake=drop_in_handshake,
                         park_and_release=park_and_release)
    log("part map %s; persisted last_seq = %d; abort-at-capture = %s"
        % (run.part_map, run.last_seq, abort))
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
