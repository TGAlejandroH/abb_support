"""Bench driver: run ONE TG part on the VC with a stand-in HMI, so the recorder has a run to watch.

    python bench_cycle.py --mod T9382dffx1.mod [--params params.json] [--part 9022] [--sent sent.json]

This is test equipment, NOT the recorder: it writes to the controller (module upload, program
pointer, start), so it is for a virtual controller only. The recorder itself is read-only.

The stand-in HMI is hmi_prototype/abb_server.AbbTgsHmi (the protocol as the C++ HMI speaks it)
with no-localization answers: camera frame identity, each weld frame = the module's own nominal
``wobj....oframe`` in TG_ReqWeldFrame order, touch-ups 0, dry run 1. The weld parameters come
from ``--params``: a JSON list, entry N served to the Nth weld (abb_server's
weld_param_sequence), so one run can give each weld different values - or zeros.

The part is started over RWS the way the earlier VC cases were (no Production Manager, no I/O
simulator toggle): PP to TG_Parts/TG_Part<part>, start once, wait for stop, PP back to main.

``--sent`` writes what was served to each weld, which is the recorder's expected-values input.
"""

import argparse
import json
import os
import re
import sys
import threading
import time
import urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "hmi_prototype"))
from abb_server import DEFAULT_SEAM_PHASES, AbbTgsHmi, fmt_real  # noqa: E402
from rws_session import RwsSession  # noqa: E402

URL = os.environ.get("TG_VC_RWS_URL", "http://127.0.0.1:80")
FORM = {"Content-Type": "application/x-www-form-urlencoded"}

DEFAULT_PARAMS = [{"udwp_flag": 1, "welder_type": 2, "weld_proc": 1, "travel_speed": 10.0,
                   "wire_feed_speed": 520.0, "arc_length": 0.0, "arc_control": 0.0, "weld_sched": 1,
                   "seam_phases": list(DEFAULT_SEAM_PHASES)}]


class BenchHmi(AbbTgsHmi):
    def __init__(self, mod_path, params, rws):
        super().__init__("127.0.0.1", 2000, program_selection=1, verbose=True, rws=rws)
        self.tgs_source_dir = os.path.dirname(os.path.abspath(mod_path))
        self.prog_name = os.path.splitext(os.path.basename(mod_path))[0]
        text = open(mod_path, encoding="utf-8", errors="replace").read()
        oframes = dict(re.findall(r"(wobj\w+)\.oframe:=(\[\[[^\]]+\],\[[^\]]+\]\])", text))
        self.weld_frames = [(w, oframes.get(w)) for w in re.findall(r"TG_ReqWeldFrame[^;]*\\WObj:=(wobj\w+)", text)]
        self.cam_frame_xyzwpr = [0.0] * 6
        self.touchup_offsets_in = [0.0, 0.0, 0.0]
        self.dry_run = 1
        self.weld_param_sequence = params
        self._frame_idx = 0
        self.sent = []
        self.error = None

    def handle_weld_frame_req(self):
        self._recv_pose_and_sub()
        wobj, literal = self.weld_frames[min(self._frame_idx, len(self.weld_frames) - 1)]
        self._frame_idx += 1
        self.do_send(literal)
        self.do_send(str(self.weld_status))
        for value in self.touchup_offsets_in:
            self.do_send(fmt_real(value))

    def handle_weld_params_req(self):
        super().handle_weld_params_req()
        self.sent.append({"t": time.time(), "udwp_flag": self.udwp_flag, "welder_type": self.welder_type,
                          "weld_proc": self.weld_proc, "travel_speed": self.travel_speed,
                          "wire_feed_speed": self.wire_feed_speed, "arc_length": self.arc_length,
                          "arc_control": self.arc_control, "weld_sched": self.weld_sched,
                          "seam_phases": list(self.seam_phases)})


def rws_post(c, path, query, fields):
    return c._request("POST", path, data=urllib.parse.urlencode(fields).encode(), query=query, headers=FORM)


def exec_state(c):
    t = c._request("GET", "/rw/rapid/execution", query={"json": "1"}).decode("utf-8", "replace")
    return re.search(r'"ctrlexecstate":\s*"([^"]+)"', t).group(1)


def run_part(part="9022", timeout=600.0, label="bench"):
    """Run TG_Part<part> once over RWS (PP to the routine, start, wait for stop, PP back to main)
    and return the task state at the end: "stopped", or the state it was in when the timeout
    stopped it. Whoever serves the HMI side (the stand-in below, or the real HMI armed in
    socket-start mode) must be listening before this is called."""
    c = RwsSession(URL)
    if exec_state(c) != "stopped":
        raise RuntimeError("controller is running - refusing to touch it")
    c.request_mastership()
    try:
        rws_post(c, "/rw/rapid/tasks/T_ROB1/pcp", {"action": "set-pp-routine"},
                 {"module": "TG_Parts", "routine": "TG_Part" + part, "userlevel": "FALSE"})
    finally:
        c.release_mastership()
    time.sleep(0.5)
    t0 = time.time()
    rws_post(c, "/rw/rapid/execution", {"action": "start"},
             {"regain": "continue", "execmode": "continue", "cycle": "once", "condition": "none",
              "stopatbp": "disabled", "alltaskbytsp": "false"})
    print("%s: started TG_Part%s at %s" % (label, part, time.strftime("%H:%M:%S")), flush=True)
    time.sleep(1.0)
    while exec_state(c) != "stopped" and time.time() - t0 < timeout:
        time.sleep(0.5)
    state = exec_state(c)
    if state != "stopped":
        rws_post(c, "/rw/rapid/execution", {"action": "stop"}, {"stopmode": "stop", "usetsp": "normal"})
        print("%s: TIMEOUT - stopped the task" % label)
    print("%s: task stopped after %.1f s" % (label, time.time() - t0), flush=True)
    # Give the recorder time to see the task stopped before the program pointer moves.
    time.sleep(1.5)
    c.request_mastership()
    try:
        rws_post(c, "/rw/rapid/execution", {"action": "resetpp"}, {})
    finally:
        c.release_mastership()
    return state


def run_cycle(mod_path, params=None, part="9022", timeout=600.0):
    """Serve one cycle of ``mod_path`` and run TG_Part<part>; returns the "sent" record
    (program, requests served, the weld parameters served to each weld, error, task state)."""
    params = params or DEFAULT_PARAMS
    probe = RwsSession(URL)
    state = exec_state(probe)
    if state != "stopped":
        raise RuntimeError("controller is running - refusing to touch it")
    hmi = BenchHmi(mod_path, params, RwsSession(URL))
    print("bench: program %s, weld frames %s, %d parameter set(s)" % (hmi.prog_name, [w for w, _ in hmi.weld_frames],
                                                                       len(params)), flush=True)

    def serve():
        try:
            hmi.serve_cycle()
        except Exception as exc:
            hmi.error = repr(exc)

    server = threading.Thread(target=serve, daemon=True)
    server.start()
    state = run_part(part, timeout)
    print("bench: requests served %s; error %s" % (hmi.request_log, hmi.error), flush=True)
    server.join(5.0)
    return {"program": hmi.prog_name, "part": part, "requests": hmi.request_log,
            "weld_params": hmi.sent, "error": hmi.error, "task_state": state}


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--mod", required=True)
    ap.add_argument("--params")
    ap.add_argument("--part", default="9022")
    ap.add_argument("--sent")
    ap.add_argument("--timeout", type=float, default=600.0)
    args = ap.parse_args(argv)
    params = None
    if args.params:
        with open(args.params) as fh:
            params = json.load(fh)
    sent = run_cycle(args.mod, params, args.part, args.timeout)
    if args.sent:
        with open(args.sent, "w") as fh:
            json.dump(sent, fh, indent=1)
    return 0 if sent["error"] is None and sent["task_state"] == "stopped" else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
