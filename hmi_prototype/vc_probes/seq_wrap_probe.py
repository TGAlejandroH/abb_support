"""Prove the START counter wrap on the VC without a Production Manager dispatch (2026-09-25).

    python seq_wrap_probe.py          (from hmi_prototype/vc_probes)

Loads TG_SeqWrapProbe.mod, which presets nTG_CycleSeq to 7999999 and runs the deployed handshake
open / START / close twice on port 2002. This script is the client: it acks each START with "0",
exactly as the HMI does, and reads to end-of-file. PASS = the wire carried "START 8000000 ..." then
"START 1 ...". Whatever happens, nTG_HandshakePort is put back to 2001 and nTG_CycleSeq to 0, the
probe is unloaded and Production Manager restarted. Reuses listen_probe.py's RWS plumbing.
"""
import os
import sys
import threading
import time

import listen_probe as lp

lp.MODULE = "TG_SeqWrapProbe"
lp.MOD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "TG_SeqWrapProbe.mod")
lp.RESULTS = ("stSwStep", "nSwFirst", "nSwSecond")


class StartClient(threading.Thread):
    """Two handshake connections: read the START line, ack it, read to end-of-file."""

    def __init__(self):
        super().__init__(daemon=True)
        self.lines = []

    def run(self):
        end = time.perf_counter() + 45.0
        while len(self.lines) < 2 and time.perf_counter() < end:
            s = lp.fast_socket()
            s.settimeout(2.0)
            try:
                s.connect((lp.HOST, lp.PORT))
            except OSError:
                s.close()
                time.sleep(0.02)
                continue
            try:
                s.settimeout(10.0)
                self.lines.append(s.recv(256).decode("ascii", "replace").strip())
                s.sendall(b"0")
                while s.recv(64):
                    pass
            except OSError as exc:
                self.lines.append("error: %s" % exc)
            finally:
                s.close()


def main():
    if not lp.load():
        lp.log("LOAD FAILED - see the event log above")
        lp.back_to_pm()
        return 1
    ok = False
    try:
        cl = StartClient()
        cl.start()
        el, res, ev = lp.run_routine("TG_SwWrap")
        cl.join(5)
        lp.show("wrap", el, res, ev)
        lp.log("wire: %s" % cl.lines)
        ok = (len(cl.lines) == 2 and cl.lines[0].startswith("START 8000000 ")
              and cl.lines[1].startswith("START 1 ") and res["stSwStep"] == "done"
              and float(res["nSwFirst"]) == 8000000 and float(res["nSwSecond"]) == 1)
    finally:
        if lp.exec_state() != "stopped":
            lp.stop()
        for sym, want in (("nTG_HandshakePort", "2001"), ("nTG_CycleSeq", "0")):
            if lp.c.get_symbol(sym) != want:
                lp.c.set_symbol(sym, want)
                lp.log("restored %s -> %s" % (sym, want))
            lp.log("%s = %s" % (sym, lp.c.get_symbol(sym)))
        lp.unload()
        lp.back_to_pm()
    lp.log("%s  START counter wraps 8000000 -> 1 on the wire" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
