"""Read-only RWS access for the run recorder.

The recorder must be safe to point at a production controller, so this class
only ever issues GET requests: it cannot write a symbol, move the program
pointer, take mastership or toggle a signal. Everything goes through ONE
cookie session (TG_RoboCal lessons_learned L9: a fresh digest session per
request exhausts the controller's pool, 503 then 401).

Parsing is regex-light on purpose: RWS 1.0 on RW 6.15 returns JSON whose
values are all strings, and the recorder wants raw values it can store
verbatim and compare later.
"""

import json
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "hmi_prototype"))
from rws_session import RwsSession  # noqa: E402

DEFAULT_URL = os.environ.get("TG_VC_RWS_URL", "http://127.0.0.1:80")
TASK = "T_ROB1"


class RwsReader:
    def __init__(self, base_url=DEFAULT_URL, timeout=5.0):
        self.base_url = base_url
        self._timeout = timeout
        self._session = RwsSession(base_url, timeout=timeout)
        self.requests = 0
        self.errors = 0

    def reconnect(self):
        """A fresh session object recovers at once after a 503/401 lockout (L9)."""
        self._session = RwsSession(self.base_url, timeout=self._timeout)

    def _get(self, path, query=None, tries=3):
        """One GET, retried on a fresh session after 503: the controller caps RWS sessions, and
        other clients (the HMI, the bench tools) use them too. Measured in a VC campaign: one 503
        on the module watcher's first read lost a whole recording."""
        for attempt in range(tries):
            self.requests += 1
            try:
                return self._session._request("GET", path, query=query)
            except Exception as exc:
                self.errors += 1
                if "503" not in str(exc) or attempt == tries - 1:
                    raise
                self.reconnect()
                time.sleep(0.3)

    def get_text(self, path, query=None):
        q = {"json": "1"}
        q.update(query or {})
        return self._get(path, q).decode("utf-8", "replace")

    def get_states(self, path, query=None):
        return json.loads(self.get_text(path, query))["_embedded"]["_state"]

    def get_file(self, path):
        """Raw bytes of a controller file, e.g. ``$home/TGS/<prog>.mod``."""
        return self._get("/fileservice/" + path)

    # ---- RAPID ----------------------------------------------------------

    def symbol(self, module, name, task=TASK):
        """The value literal of a RAPID data symbol (LOCAL ones included), as text."""
        st = self.get_states("/rw/rapid/symbol/data/RAPID/%s/%s/%s" % (task, module, name))
        return st[0]["value"]

    def exec_state(self):
        return self.get_states("/rw/rapid/execution")[0]["ctrlexecstate"]

    def pointers(self, task=TASK):
        """(program pointer, motion pointer), each (module, routine, line) or None."""
        out = []
        for s in self.get_states("/rw/rapid/tasks/%s/pcp" % task)[:2]:
            pos = s.get("beginposition")
            if not pos:
                out.append(None)
                continue
            # RWS spells it "modulemame".
            out.append((s.get("modulemame", ""), s.get("routinename", ""), int(pos.split(",")[0])))
        while len(out) < 2:
            out.append(None)
        return tuple(out)

    # ---- motion ---------------------------------------------------------

    def tcp(self, tool, wobj="wobj0", unit="ROB_1"):
        """TCP (x, y, z) in mm of ``tool`` in ``wobj``."""
        st = self.get_states("/rw/motionsystem/mechunits/%s/robtarget" % unit,
                             {"tool": tool, "wobj": wobj, "coordinate": "Wobj"})[0]
        return tuple(float(st[k]) for k in ("x", "y", "z"))

    def joints(self, unit):
        """{"rax_1": "12.5", ...} of a mechanical unit's own axes, as text. A deactivated unit
        reports 9E9 on every axis (read as 8999999488), which is left out, so it reads as {}."""
        st = self.get_states("/rw/motionsystem/mechunits/%s/jointtarget" % unit)[0]
        return {k: v for k, v in st.items()
                if k.startswith("rax_") and abs(float(v)) < 8.9e9}

    # ---- I/O ------------------------------------------------------------

    def signal(self, path):
        return self.get_states("/rw/iosystem/signals/" + path)[0]["lvalue"]

    # ---- event log ------------------------------------------------------

    def elog(self, domain=0):
        """Entries of one event-log domain as dicts: seq, code, type, tstamp, title, desc."""
        text = self.get_text("/rw/elog/%d" % domain, {"lang": "en"})
        out = []
        for s in json.loads(text)["_embedded"]["_state"]:
            m = re.search(r"/(\d+)$", s.get("_title", ""))
            if not m:
                continue
            out.append({
                "seq": int(m.group(1)),
                "code": s.get("code", ""),
                "type": s.get("msgtype", ""),
                "tstamp": s.get("tstamp", ""),
                "title": re.sub(r"\s+", " ", s.get("title", "")).strip(),
                "desc": re.sub(r"\s+", " ", s.get("desc", "")).strip(),
            })
        return sorted(out, key=lambda e: e["seq"])

    def elog_mark(self, domain=0):
        entries = self.elog(domain)
        return entries[-1]["seq"] if entries else 0


def now():
    return time.perf_counter()
