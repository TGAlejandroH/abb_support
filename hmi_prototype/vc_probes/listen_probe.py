"""Prove the listener helper on the VC before it goes into TG_Comms.sys (socket start latency item 1).

    python listen_probe.py            (from hmi_prototype/vc_probes; run make_listen_probe.py first)

The helper - open a listener at once, retry only a port that is not free yet, bounded, then let
the socket error through - replaces the blind 2 s wait at the start of every part. It runs here
inside TG_ListenProbe.mod, a generated verbatim copy (make_listen_probe.py), on port 2002, so a
mistake cannot take down TG_Comms.sys (a failed SysMod load puts the task in system failure) and
the HMI's connect loop on 2001 never sees the probe. This script plays the other side:

  A  free port       one open, timed; expect 0 retries and a few ms
  B  busy -> free    this script holds 2002 (SO_EXCLUSIVEADDRUSE, bound, not listening) until 1 s
                     after the routine starts; expect a few retries, the info event, a normal accept
  C  rebind loop     20 open / accept / close rounds, no pause, the robot closing actively each
                     time - tighter than any Production Manager part sequence; expect 20 rounds
  D  exhausted       2002 held for 6 s; expect the warning, then the last attempt's socket error
                     reaching the caller's handler after ~3 s (nLpErrno) - NOT a stopped task (40228)

Each routine runs alone (PP to routine, cycle once); results are the probe's PERS values and the
new event-log entries. At the end the probe is unloaded and Production Manager restarted from main
(TG_RoboCal L8/L9; socket plan V-2/V-25/V-27). No motion anywhere.
"""
import ctypes
import ctypes.wintypes as wt
import os
import re
import socket
import sys
import threading
import time
import urllib.parse

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from rws_session import RwsSession  # noqa: E402

URL = os.environ.get("TG_VC_RWS_URL", "http://127.0.0.1:80")
HOST, PORT = "127.0.0.1", 2002
MODULE = "TG_ListenProbe"
MOD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), MODULE + ".mod")
H = {"Content-Type": "application/x-www-form-urlencoded"}
RESULTS = ("stLpStep", "nLpRetries", "nLpOpenMs", "nLpErrno", "nLpLoops", "nLpMaxRetries", "nLpTotalRetries")

c = RwsSession(URL)
get = lambda p: c._request("GET", p, query={"json": "1", "lang": "en"}).decode("utf-8", "replace")
post = lambda p, q, f: c._request("POST", p, data=urllib.parse.urlencode(f).encode(), query=q, headers=H)
T0 = time.perf_counter()


def log(msg):
    print("[%7.2f] %s" % (time.perf_counter() - T0, msg), flush=True)


# ---------------------------------------------------------------- RWS plumbing (as _run_probe.py)
def elog_mark():
    return max(int(x) for x in re.findall(r"/rw/elog/0/(\d+)", get("/rw/elog/0")))


def elog_after(mark):
    b = get("/rw/elog/0")
    out = []
    for m in re.finditer(r'"_title":"/rw/elog/0/(\d+)"(.*?)(?="_title":"/rw/elog/0/|\Z)', b, re.S):
        if int(m.group(1)) <= mark:
            continue
        seg = m.group(2)
        g = lambda p: (re.search(p, seg, re.S).group(1) if re.search(p, seg, re.S) else "")
        desc = re.sub(r"\s+", " ", g(r'"desc":"([^"]*)"')).strip()
        out.append((int(m.group(1)), g(r'"code":"(\d+)"'), re.sub(r"\s+", " ", g(r'"title":"([^"]*)"')).strip(), desc[:200]))
    return sorted(out)


def exec_state():
    return re.search(r'"ctrlexecstate":\s*"([^"]+)"', get("/rw/rapid/execution")).group(1)


def stop():
    post("/rw/rapid/execution", {"action": "stop"}, {"stopmode": "stop", "usetsp": "normal"})
    time.sleep(1.5)


def start(cycle):
    post("/rw/rapid/execution", {"action": "start"},
         {"regain": "continue", "execmode": "continue", "cycle": cycle,
          "condition": "none", "stopatbp": "disabled", "alltaskbytsp": "false"})


def sym(name):
    m = re.search(r'"value":\s*"((?:\\.|[^"\\])*)"', get("/rw/rapid/symbol/data/RAPID/T_ROB1/%s/%s" % (MODULE, name)))
    return m.group(1).replace('\\"', '"').strip('"') if m else "?"


def load():
    mark = elog_mark()
    stop()
    with open(MOD_PATH, "rb") as fh:
        c.put_file("$home/TGS/" + MODULE + ".mod", fh.read())
    c.request_mastership()
    try:
        try:
            post("/rw/rapid/tasks/T_ROB1", {"action": "unloadmod"}, {"module": MODULE})
        except Exception:
            pass
        time.sleep(1.0)
        post("/rw/rapid/tasks/T_ROB1", {"action": "loadmod"}, {"modulepath": "$home/TGS/" + MODULE + ".mod"})
    finally:
        c.release_mastership()
    time.sleep(2.0)
    ev = elog_after(mark)
    for e in ev:
        log("  elog [%s] %s | %s" % e[1:])
    return not any(e[1] in ("40322", "40160") for e in ev)


def unload():
    if exec_state() != "stopped":
        stop()
    c.request_mastership()
    try:
        post("/rw/rapid/tasks/T_ROB1", {"action": "unloadmod"}, {"module": MODULE})
    finally:
        c.release_mastership()
    log("probe module unloaded")


def back_to_pm():
    c.request_mastership()
    try:
        post("/rw/rapid/execution", {"action": "resetpp"}, {})
    finally:
        c.release_mastership()
    time.sleep(1.0)
    start("forever")
    time.sleep(4.0)
    log("back to Production Manager: execution %s" % exec_state())


def run_routine(routine, timeout=60.0, on_started=None):
    mark = elog_mark()
    if exec_state() != "stopped":
        stop()
    c.request_mastership()
    try:
        post("/rw/rapid/tasks/T_ROB1/pcp", {"action": "set-pp-routine"},
             {"module": MODULE, "routine": routine, "userlevel": "FALSE"})
    finally:
        c.release_mastership()
    time.sleep(0.5)
    t = time.perf_counter()
    start("once")
    if on_started is not None:
        on_started()
    time.sleep(0.3)
    while exec_state() != "stopped" and time.perf_counter() - t < timeout:
        time.sleep(0.2)
    res = {r: sym(r) for r in RESULTS}
    return time.perf_counter() - t, res, elog_after(mark)


# ---------------------------------------------------------------- the other side of the wire
SIO_TCP_INITIAL_RTO = 0x98000011          # fail a refused connect in ~10 ms instead of 2 s


class _Rto(ctypes.Structure):
    _fields_ = [("Rtt", ctypes.c_ushort), ("MaxSynRetransmissions", ctypes.c_ubyte)]


def fast_socket():
    s = socket.socket()
    p, n = _Rto(0xFFFF, 0xFE), wt.DWORD(0)
    ctypes.WinDLL("ws2_32").WSAIoctl(ctypes.c_size_t(s.fileno()), wt.DWORD(SIO_TCP_INITIAL_RTO), ctypes.byref(p),
                                     ctypes.sizeof(p), None, 0, ctypes.byref(n), None, None)
    return s


class Client(threading.Thread):
    """Connect as soon as the probe listens, read to end-of-file (the robot closes), repeat."""

    def __init__(self, rounds, deadline_s=45.0):
        super().__init__(daemon=True)
        self.rounds, self.deadline = rounds, deadline_s
        self.done, self.eofs, self.error = 0, 0, ""

    def run(self):
        end = time.perf_counter() + self.deadline
        while self.done < self.rounds and time.perf_counter() < end:
            s = fast_socket()
            s.settimeout(2.0)
            try:
                s.connect((HOST, PORT))
            except OSError:
                s.close()
                time.sleep(0.02)
                continue
            try:
                s.settimeout(10.0)
                while s.recv(64):
                    pass
                self.eofs += 1
            except OSError as exc:
                self.error = str(exc)
            finally:
                s.close()
            self.done += 1


class Holder(threading.Thread):
    """Occupy the probe port (bound, not listening, exclusive) from now until hold_s after start()
    - start() is called right after the routine's start POST, so the hold is measured from when
    RAPID begins, not from when this script began setting the routine up."""

    def __init__(self, hold_s):
        super().__init__(daemon=True)
        self.hold_s = hold_s
        self.s = socket.socket()
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        self.s.bind((HOST, PORT))
        self.freed_at = None

    def run(self):
        time.sleep(self.hold_s)
        self.s.close()
        self.freed_at = time.perf_counter()
        log("  holder: port %d freed after %.1f s" % (PORT, self.hold_s))


# ---------------------------------------------------------------- the tests
def show(name, elapsed, res, ev):
    log("%s: %.1f s  %s" % (name, elapsed, "  ".join("%s=%s" % kv for kv in res.items())))
    for e in ev:
        log("  elog [%s] %s | %s" % e[1:])


def main():
    verdicts = []
    if not load():
        log("LOAD FAILED - see the event log above; nothing run")
        back_to_pm()
        return 1
    log("probe module loaded clean")
    try:
        # A - free port
        cl = Client(1)
        cl.start()
        el, res, ev = run_routine("TG_LpOpenOnce")
        cl.join(5)
        show("A free port", el, res, ev)
        ok = res["stLpStep"] == "once: done" and float(res["nLpRetries"]) == 0 and cl.eofs == 1
        verdicts.append(("A free port: opened first time, client served", ok))

        # B - busy for 1 s, then free
        hd = Holder(1.0)
        cl = Client(1)
        cl.start()
        el, res, ev = run_routine("TG_LpOpenOnce", on_started=hd.start)
        cl.join(5)
        hd.join(5)
        show("B busy->free", el, res, ev)
        n = float(res["nLpRetries"])
        info = any(e[1] == "80003" and "busy" in e[2].lower() for e in ev)
        ok = res["stLpStep"] == "once: done" and 1 <= n < 12 and cl.eofs == 1 and info
        verdicts.append(("B busy->free: retried %d x, info event %s, client served" % (n, "yes" if info else "NO"), ok))

        # C - 20 rounds, immediate rebind after an active close
        cl = Client(20)
        cl.start()
        el, res, ev = run_routine("TG_LpLoop", timeout=90.0)
        cl.join(10)
        show("C rebind loop", el, res, ev)
        ok = res["stLpStep"] == "loop: done" and float(res["nLpLoops"]) == 20 and cl.eofs == 20
        verdicts.append(("C rebind loop: %s/20 rounds, max retries %s, total %s"
                         % (res["nLpLoops"], res["nLpMaxRetries"], res["nLpTotalRetries"]), ok))

        # D - busy past the budget
        hd = Holder(6.0)
        el, res, ev = run_routine("TG_LpExhaust", on_started=hd.start)
        hd.join(10)
        show("D exhausted", el, res, ev)
        warn = any(e[1] == "80002" and "busy" in e[2].lower() for e in ev)
        stopped = any(e[1] == "40228" for e in ev)
        entries = sum(1 for e in ev if e[1] == "41571")
        ms = float(res["nLpOpenMs"])
        ok = (res["stLpStep"] == "exhaust: error reached the caller" and float(res["nLpErrno"]) != 0
              and float(res["nLpRetries"]) == 12 and 2500 <= ms <= 5000 and warn and not stopped)
        verdicts.append(("D exhausted: error %s reached the caller after %.0f ms, warning %s, task stopped %s, "
                         "41571 entries %d" % (res["nLpErrno"], ms, "yes" if warn else "NO",
                                                "YES" if stopped else "no", entries), ok))
    finally:
        unload()
        back_to_pm()
    log("")
    for text, ok in verdicts:
        log("%s  %s" % ("PASS" if ok else "FAIL", text))
    return 0 if verdicts and all(ok for _, ok in verdicts) else 1


if __name__ == "__main__":
    sys.exit(main())
