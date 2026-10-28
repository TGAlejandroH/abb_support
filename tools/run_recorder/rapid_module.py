"""What the recorder needs to know about an exported TG weld module.

The motion pointer reports a line number; this maps line numbers to the
motion instruction on that line, and finds the arc spans: an ``ArcLStart`` /
``ArcCStart`` up to and including the next ``ArcLEnd`` / ``ArcCEnd``. It also
reads the ``LOCAL CONST robtarget`` literals so a span's planned length can be
computed from the module itself (the module is the planner's export, so this is
what the robot was told to do, not what the HMI thinks it sent).
"""

import math
import re

MOTION = re.compile(r"^\s*(MoveL|MoveJ|MoveC|MoveAbsJ|ArcLStart|ArcL|ArcLEnd|ArcCStart|ArcC|ArcCEnd)\s+(.*);")
ROBTARGET = re.compile(r"^\s*(?:LOCAL\s+)?(?:CONST|PERS|VAR)\s+robtarget\s+(\w+)\s*:=\s*\[\[([^\]]*)\]")
WOBJ = re.compile(r"\\WObj:=(\w+)")
WEAVE = re.compile(r"\\Weave:=(\w+)")
DECL = re.compile(r"^\s*(?:LOCAL\s+)?(CONST|PERS|VAR)\s+(\w+)\s+(\w+)\s*:=\s*(.*);\s*$")


class ModuleInfo:
    def __init__(self, text):
        self.text = text
        self.module_name = ""
        self.motions = {}          # line -> dict(instr, args, targets, wobj, weave)
        self.robtargets = {}       # name -> (x, y, z)
        self.extax = {}            # name -> {"eax_a": deg, ...}, 9E9 axes left out
        self.declarations = {}     # name -> (storage, type, literal)
        for number, line in enumerate(text.splitlines(), start=1):
            m = re.match(r"^\s*MODULE\s+(\w+)", line)
            if m:
                self.module_name = m.group(1)
            m = ROBTARGET.match(line)
            if m:
                self.robtargets[m.group(1)] = tuple(float(v) for v in m.group(2).split(","))
            m = DECL.match(line)
            if m:
                self.declarations[m.group(3)] = (m.group(1), m.group(2), m.group(4))
                if m.group(2) == "robtarget":
                    groups = re.findall(r"\[([^\[\]]*)\]", m.group(4))
                    if len(groups) == 4:
                        self.extax[m.group(3)] = {"eax_" + "abcdef"[k]: float(v)
                                                  for k, v in enumerate(groups[3].split(","))
                                                  if "9E9" not in v.upper()}
            m = MOTION.match(line)
            if m:
                instr, args = m.group(1), m.group(2)
                first = [a.strip() for a in args.split(",")]
                targets = first[:2] if instr in ("MoveC", "ArcC", "ArcCStart", "ArcCEnd") else first[:1]
                w = WOBJ.search(args)
                v = WEAVE.search(args)
                self.motions[number] = {"instr": instr, "targets": targets,
                                        "wobj": w.group(1) if w else "wobj0",
                                        "weave": v.group(1) if v else None}

    def arc_spans(self):
        """List of dicts: start_line, end_line, lines, targets (in order), wobj, weave."""
        spans = []
        current = None
        for number in sorted(self.motions):
            mo = self.motions[number]
            if mo["instr"] in ("ArcLStart", "ArcCStart"):
                current = {"start_line": number, "lines": [number], "targets": list(mo["targets"]),
                           "wobj": mo["wobj"], "weave": mo["weave"]}
            elif current is not None and mo["instr"].startswith("Arc"):
                current["lines"].append(number)
                current["targets"].extend(mo["targets"])
                if mo["instr"] in ("ArcLEnd", "ArcCEnd"):
                    current["end_line"] = number
                    spans.append(current)
                    current = None
        return spans

    def planned_length(self, span):
        """Straight-line length through the span's targets, mm (an arc's chord path for ArcC)."""
        pts = [self.robtargets[t] for t in span["targets"] if t in self.robtargets]
        return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
