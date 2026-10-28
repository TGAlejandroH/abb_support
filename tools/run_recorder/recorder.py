"""Record one TG run on an ABB controller: what the controller did, read over RWS, read-only.

    python recorder.py [--profile profiles/monarc_vc_a_miller.json] [--out run.json]
                       [--max-seconds 900] [--sent sent.json]

Start it before the run. It records until the task has run and stopped, or until the program
pointer is back in the profile's idle module (Production Manager's ``gapMain``) after the weld
module was seen, or until ``--max-seconds``. The output is one JSON file: the raw samples, the
weld module text as the controller had it, the event-log entries of the run, and the analysis
(per weld span: when it ran, how far and how fast the TCP went, the weld data in effect, the
welder outputs). ``checks.py`` turns that into PASS/FAIL lines against expected values.

What is sampled (measured on VC A, 2026-10-28, step0_probe.py):
  every tick  program pointer (RWS does not fill the motion pointer on RW 6.15), TCP in wobj0,
              wdTG_Weld, the weld-on and arc-detect signals, and - once the weld module is
              loaded - the TCP in each work object its arc spans use, and the joints of the
              positioner a coordinated work object moves with
  in turn     execution state, the profile's slower signals and symbols
"""

import argparse
import json
import os
import re
import sys
import threading
import time

from analysis import analyse
from rapid_module import ModuleInfo
from rws_reader import RwsReader, now
from sampler import Sampler

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PROFILE = os.path.join(HERE, "profiles", "monarc_vc_a_miller.json")


def load_profile(path):
    with open(path) as fh:
        return json.load(fh)


class ModuleWatcher(threading.Thread):
    """On its own RWS session: when the weld module appears under the program pointer, fetch its
    file, read the module's weave and one other LOCAL datum while it is loaded, and add the TCP
    in each arc-span work object to the fast channels.

    The weld module is the one whose routine under the pointer is named by ``stTG_ProgName``
    (TG_Main calls ``%stTG_ProgName%``). Other modules the pointer passes through that were not
    listed at the start (the positioner package loads ``StnIndCfg12`` on the fly, measured on
    VC A) are passed over.

    A work object that moves with a mechanical unit (``wobjTG_WeldActStn``) gets that unit's
    joints as a channel too, and its TCP is read only in ticks where the unit reads active:
    reading a robtarget in the work object of a DEACTIVATED unit makes the controller log
    50066 "Robot not active" (measured on VC A, 2026-10-28) - the recorder would put an error
    into the run it is judging. Reading a deactivated unit's joints logs nothing."""

    def __init__(self, sampler, known_modules, profile, base_url):
        super().__init__(daemon=True)
        self.sampler = sampler
        self.known = set(known_modules)
        self.profile = profile
        self.reader = RwsReader(base_url)
        self.result = {"module": None, "file": None, "text": None, "local_reads": [], "error": None,
                       "seen_at": None, "mech_units": {}, "passed_over": []}

    def _new_module(self):
        for s in reversed(self.sampler.samples[-5:]):
            for p in s.get("pointers") or []:
                if p and p[0] not in self.known:
                    return p[0], p[1], s["t"]
        return None, None, None

    def run(self):
        while self.sampler.is_alive():
            module, routine, t = self._new_module()
            if module:
                try:
                    prog = self._read_prog_name()
                    if routine != prog:
                        self.known.add(module)
                        self.result["passed_over"].append(module)
                        continue
                    self.result["module"], self.result["seen_at"] = module, t
                    self._probe(module, prog)
                except Exception as exc:
                    self.result["error"] = str(exc)[:300]
                return
            time.sleep(0.1)

    def _read_prog_name(self, tries=5):
        """stTG_ProgName, with a fresh session between tries: losing the weld module to one
        refused request (503, a session cap reached) loses the whole recording."""
        for attempt in range(tries):
            try:
                return self.reader.symbol("TG_Comms", "stTG_ProgName").strip('"')
            except Exception:
                if attempt == tries - 1:
                    raise
                self.reader.reconnect()
                time.sleep(0.3)

    def _probe(self, module, prog):
        self.result["file"] = "$home/TGS/%s.mod" % prog
        text = self.reader.get_file(self.result["file"]).decode("utf-8", "replace")
        self.result["text"] = text
        info = ModuleInfo(text)
        self._read_locals(module, info)
        self._wait_past_activation(module, text)
        for wobj in sorted({sp["wobj"] for sp in info.arc_spans()}):
            where, unit = self._locate(module, wobj)
            self.result["mech_units"][wobj] = unit
            if where:
                # The frame the HMI served lands in this record's oframe (TG_ReqWeldFrame).
                self.sampler.add_slow(("symbol", where, wobj))
            if unit:
                self.sampler.add_fast(("joints", unit))
                self.sampler.add_fast(("tcp", self.profile["tool"], wobj, unit))
            else:
                self.sampler.add_fast(("tcp", self.profile["tool"], wobj))

    def _wait_past_activation(self, module, text, wait_s=60.0):
        """Wait until the program pointer is in the module past its ``TG_ActMechUnit;`` line.
        That call binds wobjTG_WeldActStn to the presented station; before it, the work object
        still names the station of the previous part."""
        lines = [n for n, line in enumerate(text.splitlines(), start=1) if line.strip() == "TG_ActMechUnit;"]
        if not lines:
            return
        deadline = now() + wait_s
        while self.sampler.is_alive() and now() < deadline:
            for s in self.sampler.samples[-5:]:
                for p in s.get("pointers") or []:
                    if p and p[0] == module and p[2] > lines[0]:
                        return
            time.sleep(0.05)

    def _locate(self, module, wobj):
        """(module the work object is declared in, the mechanical unit it moves with - its ufmec -
        or "" for a fixed one); (None, "") when the work object cannot be read. Called after
        TG_ActMechUnit ran, so a moving work object is bound by then."""
        if wobj == "wobj0":
            return None, ""
        for candidate in (module, "TG_Comms"):
            try:
                literal = self.reader.symbol(candidate, wobj)
            except Exception:
                continue
            m = re.match(r'\[\s*\w+\s*,\s*\w+\s*,\s*"([^"]*)"', literal)
            return candidate, m.group(1) if m else ""
        return None, ""

    def _read_locals(self, module, info):
        weave =[n for n, (_, typ, _) in info.declarations.items() if typ == "weavedata"]
        other = [n for n, (storage, typ, _) in info.declarations.items()
                 if storage in ("PERS", "CONST") and typ != "weavedata"]
        for name in weave + other[:1]:
            t = round(now() - self.sampler.t0, 4)
            try:
                value = self.reader.symbol(module, name)
                self.result["local_reads"].append({"t": t, "name": name, "type": info.declarations[name][1],
                                                   "value": value, "declared": info.declarations[name][2]})
            except Exception as exc:
                self.result["local_reads"].append({"t": t, "name": name, "error": str(exc)[:200]})


class Recorder:
    def __init__(self, profile, max_seconds=900.0):
        self.profile = profile
        self.max_seconds = max_seconds
        self.reader = RwsReader(profile.get("rws_url"))

    def record(self, on_ready=None):
        p = self.profile
        reader = self.reader
        started = time.strftime("%Y-%m-%d %H:%M:%S")
        mark = reader.elog_mark()
        known = {s["_title"].split("/")[-1] for s in reader.get_states("/rw/rapid/modules", {"task": p["task"]})}
        before = {"wdTG_Weld": reader.symbol("TG_Weld", "wdTG_Weld")}

        fast = [("pointers",), ("tcp", p["tool"], "wobj0"), ("symbol", "TG_Weld", "wdTG_Weld"),
                ("signal", p["weld_on_signal"]), ("signal", p["arc_detect_signal"])]
        slow = [("exec",)] + [("signal", s) for s in p.get("slow_signals", [])] \
            + [("symbol", m, n) for m, n in p.get("slow_symbols", [])]
        state = {"ran": False}

        def should_stop(sample):
            ex = sample.get("exec")
            if ex == "running":
                state["ran"] = True
            pp = (sample.get("pointers") or [None])[0]
            if state["ran"] and ex == "stopped":
                return True
            return watcher.result["module"] is not None and pp is not None and pp[0] == p.get("idle_module")

        sampler = Sampler(reader, fast, slow, should_stop=should_stop, max_seconds=self.max_seconds)
        watcher = ModuleWatcher(sampler, known, p, p.get("rws_url"))
        sampler.start()
        watcher.start()
        if on_ready:
            on_ready()
        try:
            while sampler.is_alive():
                sampler.join(0.5)
        except KeyboardInterrupt:
            sampler.halt.set()
            sampler.join(5.0)
        watcher.join(10.0)

        after = {"wdTG_Weld": reader.symbol("TG_Weld", "wdTG_Weld"),
                 "sdTG_Weld": reader.symbol("TG_Weld", "sdTG_Weld")}
        events = [e for e in reader.elog() if e["seq"] > mark]
        recording = {
            "recorder": "abb_support/tools/run_recorder",
            "profile": p,
            "started": started,
            "ended": time.strftime("%Y-%m-%d %H:%M:%S"),
            "ended_because": ("task stopped" if state["ran"] and sampler.samples
                              and sampler.samples[-1].get("exec") == "stopped"
                              else "idle module" if watcher.result["module"] and sampler.samples
                              and (sampler.samples[-1].get("pointers") or [None])[0]
                              and sampler.samples[-1]["pointers"][0][0] == p.get("idle_module")
                              else "time limit or interrupt"),
            "rws": {"requests": reader.requests + watcher.reader.requests,
                    "read_errors": sampler.read_errors},
            "known_modules": sorted(known),
            "module": watcher.result,
            "before": before,
            "after": after,
            "events": events,
            "samples": sampler.samples,
        }
        recording["analysis"] = analyse(recording)
        return recording


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", default=DEFAULT_PROFILE)
    ap.add_argument("--out", default=os.path.join(HERE, "results", time.strftime("run_%Y%m%d_%H%M%S.json")))
    ap.add_argument("--max-seconds", type=float, default=900.0)
    args = ap.parse_args(argv)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    rec = Recorder(load_profile(args.profile), args.max_seconds)
    print("recorder: waiting for the run (profile %s)" % rec.profile["name"], flush=True)
    recording = rec.record()
    with open(args.out, "w") as fh:
        json.dump(recording, fh, indent=1)
    from report import summary_lines
    for line in summary_lines(recording):
        print(line)
    print("recording: %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
