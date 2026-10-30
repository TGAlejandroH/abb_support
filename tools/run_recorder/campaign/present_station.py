"""Present a station on the VC without a TG part (VC only): load a stand-alone copy of
vc_modules/TGIndexStn<n>.mod over RWS, run its routine, unload it.

    python present_station.py 2

TG_Main refuses a part while no station reads in position (80001), and the TGIndexStn helpers
run as a TG part, so once no station is presented they cannot run. This runs the same index
calls (Production Manager's EvIndexToStn<n>, then ActStn<n>) directly, without the HMI.
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import bench_cycle  # noqa: E402  (puts hmi_prototype on the path)
from rws_session import RwsSession  # noqa: E402


def main(station):
    name = "TGPresentStn%s" % station
    with open(os.path.join(HERE, "vc_modules", "TGIndexStn%s.mod" % station), encoding="utf-8") as fh:
        text = fh.read()
    text = text.replace("TGIndexStn%s_Mod" % station, name + "_Mod").replace("TGIndexStn%s()" % station, name + "()")
    text = text.replace("        TG_ReqEnd \\Tool:=tool0 \\WObj:=wobj0;\n", "")
    c = RwsSession(bench_cycle.URL)
    c.put_file("$home/TGS/%s.mod" % name, text.replace("\n", "\r\n").encode("utf-8"))
    c.request_mastership()
    try:
        bench_cycle.rws_post(c, "/rw/rapid/tasks/T_ROB1", {"action": "loadmod"},
                             {"modulepath": "$home/TGS/%s.mod" % name, "replace": "true"})
        bench_cycle.rws_post(c, "/rw/rapid/tasks/T_ROB1/pcp", {"action": "set-pp-routine"},
                             {"module": name + "_Mod", "routine": name, "userlevel": "FALSE"})
    finally:
        c.release_mastership()
    bench_cycle.rws_post(c, "/rw/rapid/execution", {"action": "start"},
                         {"regain": "continue", "execmode": "continue", "cycle": "once", "condition": "none",
                          "stopatbp": "disabled", "alltaskbytsp": "false"})
    t0 = time.time()
    time.sleep(1.0)
    while bench_cycle.exec_state(c) != "stopped" and time.time() - t0 < 120:
        time.sleep(0.5)
    print("station %s presented, task %s after %.1f s" % (station, bench_cycle.exec_state(c), time.time() - t0))
    c.request_mastership()
    try:
        bench_cycle.rws_post(c, "/rw/rapid/execution", {"action": "resetpp"}, {})
        bench_cycle.rws_post(c, "/rw/rapid/tasks/T_ROB1", {"action": "unloadmod"}, {"module": name + "_Mod"})
    finally:
        c.release_mastership()


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "2")
