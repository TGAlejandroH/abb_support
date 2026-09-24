#!/usr/bin/env python3
"""comms_probe.py - PC <-> IRC5 communication check for a REAL controller.

One file, standard library only, independent of the rest of the repo
(tools/rapid_check.py is used for the offline RAPID check when it is present
one folder up, and skipped with a warning when it is not).
Runbook, pendant steps and pass criteria: README.md in this folder.

Every run writes a timestamped log to ./logs/ and ends with a PASS/FAIL summary.
Exit code 0 = no failing step.

COMMANDS (all need --ip; credentials default to RobotWare's factory account)

  ping      ICMP reachability of the controller.
  rws       Robot Web Services checklist: identity + RobotWare version, option
            list (is 616-1 PC Interface there?), opmode / controller state /
            RAPID state / tasks, loaded modules, HOME: listing, and a
            fileservice write round trip (PUT, GET, DELETE) in HOME:/TGS.
  upload    rapid_check + PUT TG_SocketProbe.mod to HOME:/TGS/ so the pendant
            can load it from there.
  socket    Connect to the TG_SocketProbe echo server, send TG_PING, expect
            TG_ECHO, then send QUIT so the routine ends by itself.
  all       ping -> rws -> upload -> socket. The socket step is skipped when the
            option list says 616-1 is absent (--force-socket overrides).

  Bench helpers - RWS actions that need RAPID mastership. In MANUAL the
  FlexPendant holds it, so these are for the virtual controller or a cell
  in AUTO; on the real cell in MANUAL do the same steps on the pendant.
  stop      Stop RAPID execution.
  load      loadmod HOME:/TGS/TG_SocketProbe.mod into the task; verdict from
            the module list and the event log.
  setip     Write stTG_ProbeIP on the controller (no file edit, no reload).
  run       PP to routine TG_SocketProbe and start, cycle once.
  unload    unloadmod TG_SocketProbe_Mod.
  restore   resetpp (PP to main) and start, cycle forever.
"""

import argparse
import datetime as _dt
import http.cookiejar
import importlib.util
import json
import os
import platform
import re
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
MODULE_FILE = "TG_SocketProbe.mod"
MODULE_NAME = "TG_SocketProbe_Mod"
ROUTINE_NAME = "TG_SocketProbe"
IP_SYMBOL = "stTG_ProbeIP"
CTRL_DIR = "$home/TGS"
CTRL_MODULE_PATH = CTRL_DIR + "/" + MODULE_FILE
CTRL_PROBE_FILE = CTRL_DIR + "/tg_comms_check.txt"
DEFAULT_USER = "Default User"
DEFAULT_PASSWORD = "robotics"
ACCEPT_TIMEOUT_S = 90          # mirrors ACCEPT_TIMEOUT in TG_SocketProbe.mod

EPILOG = """examples (PowerShell; quote the user name because of the space):
  python comms_probe.py --ip 192.168.125.1 all
  python comms_probe.py --ip 10.20.30.40 --user "Default User" --password robotics rws
  python comms_probe.py --ip 10.20.30.40 socket --wait 300
  python comms_probe.py --ip 127.0.0.1 stop
  python comms_probe.py --ip 127.0.0.1 load
  python comms_probe.py --ip 127.0.0.1 setip 127.0.0.1
  python comms_probe.py --ip 127.0.0.1 run
"""


# ============================================================ logging / verdicts

class Log:
    """Timestamped console + file log, and the list of step verdicts."""

    def __init__(self, log_dir, verbose):
        os.makedirs(log_dir, exist_ok=True)
        now = _dt.datetime.now()
        # Millisecond suffix: two commands chained within a second must not share a log.
        stamp = now.strftime("%Y%m%d_%H%M%S") + "_%03d" % (now.microsecond // 1000)
        self.path = os.path.join(log_dir, "comms_check_%s.log" % stamp)
        self._fh = open(self.path, "w", encoding="utf-8")
        self.verbose = verbose
        self.results = []          # (status, step, message)

    @staticmethod
    def _now():
        return _dt.datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def _emit(self, line, console=True):
        text = "[%s] %s" % (self._now(), line)
        self._fh.write(text + "\n")
        self._fh.flush()
        if console:
            print(text, flush=True)

    def say(self, msg):
        self._emit(msg)

    def detail(self, msg):
        """Raw payloads: always in the log file, on the console only with -v."""
        for ln in (str(msg).splitlines() or [""]):
            self._emit("    | " + ln, console=self.verbose)

    def _result(self, status, step, msg):
        self.results.append((status, step, msg))
        self._emit("[%s] %-12s %s" % (status, step, msg))

    def passed(self, step, msg):
        self._result("PASS", step, msg)

    def failed(self, step, msg):
        self._result("FAIL", step, msg)

    def skipped(self, step, msg):
        self._result("SKIP", step, msg)

    def warn(self, step, msg):
        self._result("WARN", step, msg)

    def info(self, step, msg):
        self._result("INFO", step, msg)

    def summary(self):
        self._emit("")
        self._emit("================ SUMMARY ================")
        for status, step, msg in self.results:
            if status != "INFO":
                self._emit("  %-4s  %-12s %s" % (status, step, msg))
        fails = sum(1 for s, _, _ in self.results if s == "FAIL")
        if fails:
            overall = "FAIL (%d failing step%s)" % (fails, "" if fails == 1 else "s")
        else:
            overall = "PASS"
        self._emit("OVERALL: " + overall)
        self._emit("log file: " + self.path)
        self._fh.close()
        return 1 if fails else 0


# ============================================================ RWS client (stdlib)

class RwsError(Exception):
    """An RWS request failed. kind: NETWORK | AUTH | GRANT | NOTFOUND | HTTP."""

    def __init__(self, kind, message, code=None, body=b""):
        super().__init__(message)
        self.kind = kind
        self.code = code
        self.body = body

    def explain(self):
        hints = {
            "NETWORK": "no HTTP answer: wrong IP/port, laptop not on the robot's subnet, "
                       "or a firewall (RWS is base RobotWare on TCP 80)",
            "AUTH": "HTTP 401: user/password rejected by UAS - get the account from MONARC "
                    "or create the dedicated TG user",
            "GRANT": "HTTP 403: the account exists but lacks the grant for this action",
            "NOTFOUND": "HTTP 404: the resource does not exist on this controller",
        }
        text = str(self)
        if self.kind in hints:
            text += " -> " + hints[self.kind]
        said = rws_status_message(self.body)
        if said:
            text += " [controller says: %s]" % said
        return text


def rws_status_message(body):
    """Pull the 'msg' (or numeric 'code') out of an RWS error payload."""
    try:
        text = body.decode("utf-8", "replace")
    except Exception:
        return ""
    m = re.search(r'"msg"\s*:\s*"([^"]{0,160})', text)
    if m:
        return m.group(1)
    m = re.search(r'"code"\s*:\s*(-?\d+)', text)
    return ("status code " + m.group(1)) if m else ""


def state(payload):
    """The RWS 1.0 JSON body: payload._embedded._state (a list)."""
    if isinstance(payload, dict):
        return payload.get("_embedded", {}).get("_state", []) or []
    return []


class Rws:
    """One controller: base URL + digest credentials + ONE session (cookie jar).

    The cookie jar matters: without it every request opens a new RWS session,
    which on RW6 produced 503 then a 401 lockout (abb_support finding V-3).
    """

    def __init__(self, ip, port, user, password, timeout):
        self.base = "http://%s:%d" % (ip, port)
        self.timeout = timeout
        pm = urllib.request.HTTPPasswordMgrWithDefaultRealm()
        pm.add_password(None, self.base, user, password)
        self.cookies = http.cookiejar.CookieJar()
        self._opener = urllib.request.build_opener(
            urllib.request.HTTPCookieProcessor(self.cookies),
            urllib.request.HTTPDigestAuthHandler(pm))

    def request(self, method, path, data=None, headers=None, query=None):
        url = self.base + urllib.parse.quote(path, safe="/$")
        if query:
            url += "?" + urllib.parse.urlencode(query)
        req = urllib.request.Request(url, data=data, method=method, headers=headers or {})
        try:
            with self._opener.open(req, timeout=self.timeout) as resp:
                return resp.status, resp.read()
        except urllib.error.HTTPError as exc:
            try:
                body = exc.read()
            except Exception:
                body = b""
            kind = {401: "AUTH", 403: "GRANT", 404: "NOTFOUND"}.get(exc.code, "HTTP")
            raise RwsError(kind, "%s %s: HTTP %d %s" % (method, path, exc.code, exc.reason),
                           exc.code, body) from exc
        except urllib.error.URLError as exc:
            raise RwsError("NETWORK", "%s %s: %s" % (method, path, exc.reason)) from exc
        except (socket.timeout, TimeoutError) as exc:
            raise RwsError("NETWORK", "%s %s: timed out after %.0f s"
                           % (method, path, self.timeout)) from exc
        except ValueError as exc:          # e.g. an auth scheme urllib cannot do
            raise RwsError("HTTP", "%s %s: %s" % (method, path, exc)) from exc

    def get_json(self, path, query=None):
        q = dict(query or {})
        q["json"] = "1"
        _, body = self.request("GET", path, query=q)
        text = body.decode("utf-8", "replace")
        try:
            return json.loads(text)
        except json.JSONDecodeError as exc:
            raise RwsError("HTTP", "GET %s: body is not JSON (%s)" % (path, exc), body=body)

    def post_form(self, path, fields, query=None):
        body = urllib.parse.urlencode(fields).encode("ascii")
        return self.request("POST", path, data=body, query=query,
                            headers={"Content-Type": "application/x-www-form-urlencoded"})

    # -- fileservice ---------------------------------------------------------
    def get_file(self, path):
        return self.request("GET", "/fileservice/" + path.lstrip("/"))[1]

    def put_file(self, path, data):
        self.request("PUT", "/fileservice/" + path.lstrip("/"), data=data,
                     headers={"Content-Type": "application/octet-stream"})

    def delete_file(self, path):
        self.request("DELETE", "/fileservice/" + path.lstrip("/"))

    def list_dir(self, path):
        return state(self.get_json("/fileservice/" + path.strip("/") + "/"))

    def create_dir(self, parent, name):
        self.post_form("/fileservice/" + parent.strip("/") + "/",
                       {"fs-newname": name, "fs-action": "create"})

    # -- mastership ----------------------------------------------------------
    def request_mastership(self):
        self.post_form("/rw/mastership", {}, {"action": "request"})

    def release_mastership(self):
        self.post_form("/rw/mastership", {}, {"action": "release"})


def with_mastership(rws, fn):
    rws.request_mastership()
    try:
        return fn()
    finally:
        try:
            rws.release_mastership()
        except RwsError:
            pass


def mastership_explain(exc):
    text = exc.explain()
    low = text.lower()
    if "c004841a" in low or "held" in low or "mastership" in low:
        text += (" -> RAPID mastership is held by the FlexPendant in MANUAL; the bench "
                 "helpers need AUTO, or do this step on the pendant")
    return text


# ============================================================ step: ping

def step_ping(args, log):
    is_win = platform.system().lower().startswith("win")
    if is_win:
        cmd = ["ping", "-n", "4", "-w", "1000", args.ip]
    else:
        cmd = ["ping", "-c", "4", "-W", "1", args.ip]
    log.say("ping: " + " ".join(cmd))
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=40)
    except FileNotFoundError:
        log.failed("ping", "ping command not found on this PC")
        return False
    except subprocess.TimeoutExpired:
        log.failed("ping", "ping did not finish within 40 s")
        return False
    out = (proc.stdout or "") + (proc.stderr or "")
    log.detail(out.strip())
    replies = len(re.findall(r"\bttl[=<]", out, re.I))   # 'TTL=' on Windows, 'ttl=' on Linux
    if replies >= 4:
        log.passed("ping", "%d/4 replies from %s" % (replies, args.ip))
        return True
    if replies >= 1:
        log.warn("ping", "%d/4 replies from %s - lossy link, continuing" % (replies, args.ip))
        return True
    log.failed("ping", "no ICMP reply from %s - wrong IP, laptop not on the robot's subnet, "
                       "cable/port, or controller off" % args.ip)
    return False


# ============================================================ steps: RWS

def step_rws_identity(rws, log):
    try:
        payload = rws.get_json("/rw/system")
    except RwsError as exc:
        log.failed("rws-identity", exc.explain())
        return None
    entry = next((s for s in state(payload) if s.get("_type") == "sys-system-li"), {})
    log.detail(json.dumps(entry, indent=1)[:1500])
    log.passed("rws-identity", "system '%s', RobotWare %s, controller up since %s"
               % (entry.get("name", "?"),
                  entry.get("rwversionname") or entry.get("rwversion", "?"),
                  entry.get("starttm", "?")))
    return payload


def option_list(rws, sys_payload):
    try:
        opts = [s.get("option", "") for s in state(rws.get_json("/rw/system/options"))]
        if opts:
            return opts
    except RwsError:
        pass
    for s in state(sys_payload or {}):            # older shape: inline in /rw/system
        if s.get("_type") == "sys-options-li":
            return [o.get("option", "") for o in s.get("options", [])]
    return []


def step_rws_options(rws, log, sys_payload):
    opts = option_list(rws, sys_payload)
    if not opts:
        log.warn("rws-options", "option list unreadable - check 616-1 on the pendant "
                                "(ABB menu -> System Info -> System Properties -> Options)")
        return None
    log.detail("\n".join(opts))

    def has(code):
        return any(o.strip().startswith(code) for o in opts)

    if has("616-1"):
        log.passed("rws-options", "616-1 PC Interface PRESENT (%d options listed)" % len(opts))
    else:
        log.failed("rws-options", "616-1 PC Interface ABSENT - TG_SocketProbe.mod cannot load and "
                                  "no socket test is possible until ABB installs the option. "
                                  "RWS itself is unaffected")
    for code, name in (("633-4", "Arc"), ("812-1", "Production Manager"),
                       ("623-1", "Multitasking"), ("614-1", "FTP/SFTP client")):
        log.info("rws-options", "%s %s: %s" % (code, name, "present" if has(code) else "absent"))
    return has("616-1")


def step_rws_state(rws, log, task):
    try:
        opmode = state(rws.get_json("/rw/panel/opmode"))[0].get("opmode", "?")
        ctrl = state(rws.get_json("/rw/panel/ctrlstate"))[0].get("ctrlstate", "?")
        ex = state(rws.get_json("/rw/rapid/execution"))[0]
        tasks = state(rws.get_json("/rw/rapid/tasks"))
    except RwsError as exc:
        log.failed("rws-state", exc.explain())
        return None
    except (IndexError, AttributeError) as exc:
        log.failed("rws-state", "unexpected payload shape: %r" % (exc,))
        return None
    names = ", ".join("%s%s" % (t.get("name"), "*" if t.get("motiontask") == "TRUE" else "")
                      for t in tasks)
    log.passed("rws-state", "opmode %s, controller %s, RAPID %s (cycle %s); tasks: %s (*=motion)"
               % (opmode, ctrl, ex.get("ctrlexecstate", "?"), ex.get("cycle", "?"), names))
    if not any(t.get("name") == task for t in tasks):
        log.warn("rws-state", "task '%s' not found - pass --task <motion task name>" % task)
    return {"opmode": opmode, "ctrlstate": ctrl, "execstate": ex.get("ctrlexecstate")}


def module_names(rws, task):
    return [m.get("name", "") for m in state(rws.get_json("/rw/rapid/modules", {"task": task}))]


def step_rws_modules(rws, log, task):
    try:
        names = module_names(rws, task)
    except RwsError as exc:
        log.failed("rws-modules", exc.explain())
        return None
    log.detail(", ".join(names))
    tg = [n for n in ("TG_Comms", "TG_Main", "TG_Parts", MODULE_NAME) if n in names]
    log.passed("rws-modules", "%d modules loaded in %s; %s"
               % (len(names), task, ("TG modules: " + ", ".join(tg)) if tg else "no TG module loaded"))
    return names


def step_rws_home(rws, log):
    try:
        entries = rws.list_dir("$home")
    except RwsError as exc:
        log.failed("rws-home", exc.explain())
        return None
    dirs = [e.get("_title", "") for e in entries if e.get("_type") == "fs-dir"]
    files = [e.get("_title", "") for e in entries if e.get("_type") == "fs-file"]
    log.detail("dirs: %s\nfiles: %s" % (", ".join(dirs), ", ".join(files)))
    log.passed("rws-home", "HOME: listed - %d dirs, %d files; TGS folder %s"
               % (len(dirs), len(files),
                  "present" if "TGS" in dirs else "absent (the write test creates it)"))
    return dirs


def ensure_ctrl_dir(rws, log, step):
    try:
        rws.list_dir(CTRL_DIR)
        return
    except RwsError as exc:
        if exc.kind != "NOTFOUND":
            raise
    rws.create_dir("$home", "TGS")
    log.info(step, "created HOME:/TGS on the controller")


def step_rws_write(rws, log):
    stamp = _dt.datetime.now().isoformat(timespec="seconds")
    content = ("TG comms check %s from %s\r\n" % (stamp, socket.gethostname())).encode("ascii", "replace")
    try:
        ensure_ctrl_dir(rws, log, "rws-write")
        rws.put_file(CTRL_PROBE_FILE, content)
        back = rws.get_file(CTRL_PROBE_FILE)
        if back != content:
            log.failed("rws-write", "PUT/GET mismatch: sent %d bytes, read back %d"
                       % (len(content), len(back)))
            return False
        rws.delete_file(CTRL_PROBE_FILE)
        try:
            rws.get_file(CTRL_PROBE_FILE)
            log.failed("rws-write", "file still readable after DELETE")
            return False
        except RwsError as exc:
            if exc.kind != "NOTFOUND":
                raise
    except RwsError as exc:
        hint = ""
        if exc.kind == "GRANT":
            hint = " - this account cannot write files; use/create a UAS user with file write grants"
        log.failed("rws-write", exc.explain() + hint)
        return False
    log.passed("rws-write", "PUT / GET / DELETE round trip on HOME:/TGS OK (%d bytes)" % len(content))
    return True


def run_rws_suite(args, log, write=True):
    rws = Rws(args.ip, args.rws_port, args.user, args.password, args.timeout)
    sys_payload = step_rws_identity(rws, log)
    if sys_payload is None:
        return rws, {"ok": False, "has_616": None}
    has_616 = step_rws_options(rws, log, sys_payload)
    step_rws_state(rws, log, args.task)
    step_rws_modules(rws, log, args.task)
    step_rws_home(rws, log)
    if write:
        step_rws_write(rws, log)
    else:
        log.skipped("rws-write", "--no-write")
    return rws, {"ok": True, "has_616": has_616}


# ============================================================ step: upload

def rapid_check_findings(path, log):
    checker = os.path.normpath(os.path.join(HERE, "..", "rapid_check.py"))
    if not os.path.isfile(checker):
        log.warn("upload", "rapid_check.py not found at %s - offline RAPID check skipped" % checker)
        return []
    spec = importlib.util.spec_from_file_location("rapid_check", checker)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.check_file(path)


def step_upload(rws, log):
    local = os.path.join(HERE, MODULE_FILE)
    if not os.path.isfile(local):
        log.failed("upload", "%s is missing beside comms_probe.py" % MODULE_FILE)
        return False
    findings = rapid_check_findings(local, log)
    if findings:
        log.detail("\n".join(findings))
        log.failed("upload", "rapid_check found %d problem(s) in %s - fix them before uploading"
                   % (len(findings), MODULE_FILE))
        return False
    with open(local, "rb") as fh:
        data = fh.read()
    try:
        ensure_ctrl_dir(rws, log, "upload")
        rws.put_file(CTRL_MODULE_PATH, data)
        back = rws.get_file(CTRL_MODULE_PATH)
    except RwsError as exc:
        log.failed("upload", exc.explain())
        return False
    if back != data:
        log.failed("upload", "read-back differs from the local file (%d vs %d bytes)"
                   % (len(back), len(data)))
        return False
    log.passed("upload", "%s (%d bytes) is on the controller at HOME:/TGS/%s"
               % (MODULE_FILE, len(data), MODULE_FILE))
    return True


# ============================================================ step: socket

def _connect(ip, port, deadline, per_try=3.0):
    attempts = 0
    last = None
    while True:
        attempts += 1
        try:
            return socket.create_connection((ip, port), timeout=per_try), attempts, None
        except OSError as exc:
            last = exc
        if time.time() >= deadline:
            return None, attempts, last
        time.sleep(1.0)


def _exchange(sock, text, timeout=10.0):
    sock.settimeout(timeout)
    t0 = time.perf_counter()
    sock.sendall(text.encode("ascii"))
    reply = sock.recv(1024)
    return reply.decode("ascii", "replace"), (time.perf_counter() - t0) * 1000.0


def step_socket(args, log, wait, quit_after=True):
    log.say("socket: connecting to %s:%d, retrying for up to %d s. If TG_SocketProbe is not "
            "running yet, start it on the pendant now." % (args.ip, args.socket_port, wait))
    deadline = time.time() + wait
    attempts = 0
    stale = 0
    while True:
        sock, tries, last = _connect(args.ip, args.socket_port, deadline)
        attempts += tries
        if sock is None:
            extra = ""
            if stale:
                extra = (" %d connection(s) were accepted but never answered: a listener left "
                         "open by a STOPPED routine (guard stop / enabling device released). "
                         "Restart TG_SocketProbe and keep the enabling device pressed." % stale)
            log.failed("socket", "no working listener on %s:%d after %d s / %d attempt%s (last: %s). "
                                 "TG_SocketProbe not running, %s is not the controller's own IP, "
                                 "or 616-1 is missing.%s"
                       % (args.ip, args.socket_port, wait, attempts, "" if attempts == 1 else "s",
                          last, IP_SYMBOL, extra))
            return False
        msg = "TG_PING " + _dt.datetime.now().strftime("%H:%M:%S")
        try:
            with sock:
                reply, rtt = _exchange(sock, msg)
            break
        except (OSError, socket.timeout) as exc:
            # Real-cell finding 2026-09-24: a routine stopped inside SocketAccept
            # leaves its listener open, so the connect succeeds and nothing answers.
            stale += 1
            log.warn("socket", "connected, but no answer within 10 s (%s) - stale listener from a "
                               "stopped routine? retrying until the routine is really running" % exc)
            if time.time() >= deadline:
                log.failed("socket", "gave up after %d s: %d accepted connection(s) never answered. "
                                     "Restart TG_SocketProbe on the pendant and keep the enabling "
                                     "device pressed" % (wait, stale))
                return False
            time.sleep(2.0)
    log.detail("sent %r, got %r" % (msg, reply))
    if reply != "TG_ECHO " + msg:
        log.failed("socket", "unexpected reply %r (expected %r)" % (reply, "TG_ECHO " + msg))
        return False
    log.passed("socket", "echo OK from %s:%d, round trip %.1f ms (connected on attempt %d)"
               % (args.ip, args.socket_port, rtt, attempts))
    if quit_after:
        sock2, _, last = _connect(args.ip, args.socket_port, time.time() + 15)
        if sock2 is None:
            log.warn("socket-quit", "could not reconnect to send QUIT (%s) - the routine ends "
                                    "by itself after %d s without a client" % (last, ACCEPT_TIMEOUT_S))
            return True
        try:
            with sock2:
                reply, _ = _exchange(sock2, "QUIT")
            log.info("socket-quit", "QUIT sent, robot answered %r - TG_SocketProbe has finished" % reply)
        except (OSError, socket.timeout) as exc:
            log.warn("socket-quit", "QUIT exchange failed: %s" % exc)
    return True


# ============================================================ bench helpers (RWS + mastership)

def exec_state(rws):
    return state(rws.get_json("/rw/rapid/execution"))[0].get("ctrlexecstate", "?")


def wait_exec(rws, want, seconds=8.0):
    end = time.time() + seconds
    while time.time() < end:
        st = exec_state(rws)
        if st == want:
            return st
        time.sleep(0.5)
    return exec_state(rws)


def require_stopped(rws, log, step):
    st = exec_state(rws)
    if st != "stopped":
        log.failed(step, "RAPID is '%s' - stop it first ('stop' here on a bench, or Stop on the pendant)" % st)
        return False
    return True


def elog_text(rws):
    _, body = rws.request("GET", "/rw/elog/0", query={"json": "1", "lang": "en"})
    return body.decode("utf-8", "replace")


def elog_mark(rws):
    ids = [int(x) for x in re.findall(r'"/rw/elog/0/(\d+)"', elog_text(rws))]
    return max(ids) if ids else 0


def elog_after(rws, mark):
    out = []
    for m in re.finditer(r'"_title":"/rw/elog/0/(\d+)"(.*?)(?="_title":"/rw/elog/0/|\Z)',
                         elog_text(rws), re.S):
        seq = int(m.group(1))
        if seq <= mark:
            continue
        seg = m.group(2)

        def g(pattern):
            mm = re.search(pattern, seg)
            return mm.group(1) if mm else ""

        out.append((seq, g(r'"code":"(\d+)"'), g(r'"title":"([^"]*)"'), g(r'"desc":"([^"]*)"')[:200]))
    return sorted(out)


def pcp(rws, task):
    for s in state(rws.get_json("/rw/rapid/tasks/%s/pcp" % task)):
        if s.get("_title") == "progpointer":
            return s.get("modulemame") or s.get("modulename", "?"), s.get("routinename", "?")
    return "?", "?"


START_FIELDS = {"regain": "continue", "execmode": "continue", "condition": "none",
                "stopatbp": "disabled", "alltaskbytsp": "false"}


def cmd_stop(rws, log):
    try:
        rws.post_form("/rw/rapid/execution", {"stopmode": "stop", "usetsp": "normal"}, {"action": "stop"})
        st = wait_exec(rws, "stopped")
    except RwsError as exc:
        log.failed("stop", exc.explain())
        return False
    (log.passed if st == "stopped" else log.failed)("stop", "RAPID execution is %s" % st)
    return st == "stopped"


def cmd_load(rws, log, task):
    try:
        if not require_stopped(rws, log, "load"):
            return False
        mark = elog_mark(rws)
        with_mastership(rws, lambda: rws.post_form("/rw/rapid/tasks/" + task,
                                                   {"modulepath": CTRL_MODULE_PATH},
                                                   {"action": "loadmod"}))
        time.sleep(2.0)
        names = module_names(rws, task)
        events = elog_after(rws, mark)
    except RwsError as exc:
        log.failed("load", mastership_explain(exc))
        return False
    for seq, code, title, desc in events:
        log.detail("elog %d [%s] %s | %s" % (seq, code, title, desc))
    fatal = [e for e in events if e[1] in ("40322", "40223")]
    if MODULE_NAME in names and not fatal:
        log.passed("load", "%s loaded into %s (%d new event-log entr%s, none fatal)"
                   % (MODULE_NAME, task, len(events), "y" if len(events) == 1 else "ies"))
        return True
    what = "; ".join("[%s] %s" % (e[1], e[2]) for e in fatal) or "module missing from the task's module list"
    log.failed("load", "%s - if the event log names SocketCreate/SocketBind as unknown, 616-1 is missing"
               % what)
    return False


def cmd_setip(rws, log, task, value):
    path = "/rw/rapid/symbol/data/RAPID/%s/%s/%s" % (task, MODULE_NAME, IP_SYMBOL)
    try:
        rws.post_form(path, {"value": '"%s"' % value}, {"action": "set"})
        got = state(rws.get_json(path))[0].get("value", "")
    except RwsError as exc:
        extra = " - is %s loaded?" % MODULE_NAME if exc.kind == "NOTFOUND" else ""
        log.failed("setip", mastership_explain(exc) + extra)
        return False
    except IndexError:
        log.failed("setip", "wrote the value but the read-back payload had no state")
        return False
    if got.strip('"') == value:
        log.passed("setip", "%s = %s on the controller (takes effect on the next run, no reload)"
                   % (IP_SYMBOL, got))
        return True
    log.failed("setip", "wrote %s but read back %r" % (value, got))
    return False


PCP_ATTEMPTS = (
    ("set-pp-routine", lambda: {"routine": ROUTINE_NAME, "userlevel": "FALSE"}),
    ("set-pp-cursor", lambda: {"module": MODULE_NAME, "routine": ROUTINE_NAME, "line": "1", "column": "1"}),
    ("routine", lambda: {"routine": ROUTINE_NAME}),
)


def cmd_run(rws, log, task):
    errors = []

    def try_pcp():
        # The verdict is the PP read-back, not the HTTP status: on RW 6.15 the
        # first action moved the PP while urllib still reported an error for it
        # (VC 2026-09-23), so each attempt is followed by a pcp read.
        for action, fields in PCP_ATTEMPTS:
            try:
                rws.post_form("/rw/rapid/tasks/%s/pcp" % task, fields(), {"action": action})
                errors.append("%s: accepted" % action)
            except RwsError as exc:
                errors.append("%s: %s" % (action, exc))
            if pcp(rws, task)[1] == ROUTINE_NAME:
                return action
        return None

    try:
        if not require_stopped(rws, log, "run"):
            return False
        used = with_mastership(rws, try_pcp)
        mod, rout = pcp(rws, task)
    except RwsError as exc:
        log.failed("run", mastership_explain(exc))
        return False
    log.detail("\n".join(errors))
    if rout != ROUTINE_NAME:
        log.failed("run", "could not set PP to %s over RWS (PP is at %s/%s) - "
                          "use the pendant: Debug -> PP to Routine" % (ROUTINE_NAME, mod, rout))
        return False
    log.info("run", "PP verified at %s/%s (set by pcp action '%s')" % (mod, rout, used))
    try:
        fields = dict(START_FIELDS, cycle="once")
        rws.post_form("/rw/rapid/execution", fields, {"action": "start"})
        st = wait_exec(rws, "running", 5.0)
    except RwsError as exc:
        log.failed("run", "start refused: " + exc.explain())
        return False
    if st == "running":
        log.passed("run", "RAPID running - TG_SocketProbe should be listening; run 'socket' now")
    else:
        log.failed("run", "RAPID is '%s' after start - check the Operator Window / event log" % st)
    return st == "running"


def cmd_unload(rws, log, task):
    try:
        if not require_stopped(rws, log, "unload"):
            return False
        with_mastership(rws, lambda: rws.post_form("/rw/rapid/tasks/" + task,
                                                   {"module": MODULE_NAME},
                                                   {"action": "unloadmod"}))
        time.sleep(1.0)
        names = module_names(rws, task)
    except RwsError as exc:
        log.failed("unload", mastership_explain(exc))
        return False
    if MODULE_NAME in names:
        log.failed("unload", "%s is still loaded" % MODULE_NAME)
        return False
    log.passed("unload", "%s unloaded from %s" % (MODULE_NAME, task))
    return True


def cmd_restore(rws, log, task):
    try:
        if not require_stopped(rws, log, "restore"):
            return False
        with_mastership(rws, lambda: rws.post_form("/rw/rapid/execution", {}, {"action": "resetpp"}))
        time.sleep(1.0)
        mod, rout = pcp(rws, task)
        rws.post_form("/rw/rapid/execution", dict(START_FIELDS, cycle="forever"), {"action": "start"})
        st = wait_exec(rws, "running", 5.0)
    except RwsError as exc:
        log.failed("restore", mastership_explain(exc))
        return False
    (log.passed if st == "running" else log.failed)(
        "restore", "PP reset to %s/%s, RAPID %s" % (mod, rout, st))
    return st == "running"


# ============================================================ 'all'

def print_pendant_instructions(args, log):
    for ln in (
        "----- FlexPendant steps (MANUAL, motors on; hold the enabling device while it runs) -----",
        " 1. Program Editor -> Modules -> File -> Load Module... -> HOME:/TGS/%s" % MODULE_FILE,
        " 2. Program Data -> string -> %s : must read %s (the IP this PC connects to)" % (IP_SYMBOL, args.ip),
        " 3. Debug -> PP to Routine... -> %s" % ROUTINE_NAME,
        " 4. Press Start. Operator Window: 'TG PROBE: listening - run comms_probe.py socket on the PC'",
        "-------------------------------------------------------------------------------------------",
    ):
        log.say(ln)


def cmd_all(args, log):
    if not step_ping(args, log):
        log.skipped("rws", "no ping reply - not attempted (run the 'rws' command to force)")
        log.skipped("socket", "no ping reply - not attempted")
        return
    rws, res = run_rws_suite(args, log, write=not args.no_write)
    if not res["ok"]:
        log.skipped("upload", "RWS unavailable")
    elif args.no_upload:
        log.skipped("upload", "--no-upload")
    else:
        step_upload(rws, log)
    if args.skip_socket:
        log.skipped("socket", "--skip-socket")
        return
    if res["has_616"] is False and not args.force_socket:
        log.skipped("socket", "616-1 absent per the option list - nothing can listen "
                              "(--force-socket overrides)")
        return
    print_pendant_instructions(args, log)
    step_socket(args, log, args.wait, quit_after=not args.no_quit)


# ============================================================ CLI

def build_parser():
    p = argparse.ArgumentParser(
        prog="comms_probe.py",
        description="PC <-> IRC5 communication check (ping, Robot Web Services, TCP socket).",
        epilog=EPILOG, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ip", required=True,
                   help="controller IP on the port the PC is plugged into "
                        "(WAN port address, or 192.168.125.1 on the service port)")
    p.add_argument("--user", default=DEFAULT_USER, help="UAS user for RWS (default: '%s')" % DEFAULT_USER)
    p.add_argument("--password", default=DEFAULT_PASSWORD, help="UAS password (default: RobotWare factory)")
    p.add_argument("--rws-port", type=int, default=80, help="RWS HTTP port (default 80)")
    p.add_argument("--socket-port", type=int, default=2000,
                   help="TCP port TG_SocketProbe binds (nTG_ProbePort, default 2000)")
    p.add_argument("--task", default="T_ROB1", help="RAPID motion task (default T_ROB1)")
    p.add_argument("--timeout", type=float, default=10.0, help="per-request HTTP timeout, seconds")
    p.add_argument("--log-dir", default=os.path.join(HERE, "logs"), help="where the run log goes")
    p.add_argument("-v", "--verbose", action="store_true",
                   help="echo raw controller payloads to the console (they are always in the log)")

    sub = p.add_subparsers(dest="cmd", required=True, metavar="COMMAND")
    sub.add_parser("ping", help="ICMP reachability")
    s = sub.add_parser("rws", help="Robot Web Services checklist")
    s.add_argument("--no-write", action="store_true", help="skip the fileservice write round trip")
    sub.add_parser("upload", help="rapid_check + PUT TG_SocketProbe.mod to HOME:/TGS/")
    s = sub.add_parser("socket", help="TCP echo test against a running TG_SocketProbe")
    s.add_argument("--wait", type=int, default=180, help="seconds to keep retrying the connect (default 180)")
    s.add_argument("--no-quit", action="store_true", help="leave the probe routine listening afterwards")
    s = sub.add_parser("all", help="ping -> rws -> upload -> socket")
    s.add_argument("--no-write", action="store_true")
    s.add_argument("--no-upload", action="store_true")
    s.add_argument("--wait", type=int, default=180)
    s.add_argument("--no-quit", action="store_true")
    s.add_argument("--skip-socket", action="store_true", help="stop after upload")
    s.add_argument("--force-socket", action="store_true", help="try the socket even if 616-1 reads absent")
    for name, text in (("stop", "bench: stop RAPID execution"),
                       ("load", "bench: loadmod HOME:/TGS/TG_SocketProbe.mod (needs RAPID stopped + AUTO)"),
                       ("run", "bench: PP to TG_SocketProbe and start, cycle once"),
                       ("unload", "bench: unloadmod TG_SocketProbe_Mod"),
                       ("restore", "bench: resetpp + start forever (back to the cell's main)")):
        sub.add_parser(name, help=text)
    s = sub.add_parser("setip", help="bench: write stTG_ProbeIP over RWS")
    s.add_argument("value", help="the controller's own IP to bind, e.g. 192.168.125.1")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    log = Log(args.log_dir, args.verbose)
    log.say("comms_probe '%s' | controller %s | RWS %s:%d as '%s' | socket port %d | task %s | %s"
            % (args.cmd, args.ip, args.ip, args.rws_port, args.user, args.socket_port, args.task,
               _dt.date.today().isoformat()))
    try:
        if args.cmd == "ping":
            step_ping(args, log)
        elif args.cmd == "rws":
            run_rws_suite(args, log, write=not args.no_write)
        elif args.cmd == "upload":
            step_upload(Rws(args.ip, args.rws_port, args.user, args.password, args.timeout), log)
        elif args.cmd == "socket":
            step_socket(args, log, args.wait, quit_after=not args.no_quit)
        elif args.cmd == "all":
            cmd_all(args, log)
        else:
            rws = Rws(args.ip, args.rws_port, args.user, args.password, args.timeout)
            {
                "stop": lambda: cmd_stop(rws, log),
                "load": lambda: cmd_load(rws, log, args.task),
                "setip": lambda: cmd_setip(rws, log, args.task, args.value),
                "run": lambda: cmd_run(rws, log, args.task),
                "unload": lambda: cmd_unload(rws, log, args.task),
                "restore": lambda: cmd_restore(rws, log, args.task),
            }[args.cmd]()
    except KeyboardInterrupt:
        log.say("interrupted by operator")
    except RwsError as exc:
        log.failed(args.cmd, exc.explain())
    return log.summary()


if __name__ == "__main__":
    sys.exit(main())
