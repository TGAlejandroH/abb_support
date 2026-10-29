"""Turn a recording into per-weld facts: when each arc span ran, how far and how fast the TCP
went, how long the welder's weld-on output was high, and the weld data in effect.

Three clocks, all reported, because they answer different questions:

* POINTER window: from the first sample whose program pointer is on one of the span's lines
  to the first sample where it has left them. (RWS on RW 6.15 never fills the motion pointer,
  measured 2026-10-28; the program pointer stays on a fine ArcLEnd until the robot arrives.)
* TCP travel: the TCP, read in the span's own work object, leaving the span's first target
  and reaching its last. That is the length the arc welds; E59 (ArcLStart and ArcLEnd on the
  same target) shows up as 0 mm in 0 s.
* WELD-ON: the welder's weld-on output high, and its arc-detect input high (both simulated by
  the VC's Arc option when welding is inhibited, measured on VC A).

Every edge carries an uncertainty of half the gap between the two samples that bracket it.
"""

import math
import re

from rapid_module import ModuleInfo

TARGET_TOL_MM = 0.5      # TCP within this of a target = at the target
IPM_TO_MMS = 0.4233333   # TG_Weld.sys nTG_IpmToMmS


def tcp_key(tool, wobj):
    return "tcp:%s:%s" % (tool, wobj)


def parse_welddata(literal):
    """welddata literal -> dict, or None. [weld_speed, org_weld_speed, main_arc, org_arc],
    arcdata = [sched, mode, voltage, wirefeed, control, current, voltage2, wirefeed2, control2]."""
    if not literal:
        return None
    nums = [float(v) for v in re.findall(r"-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?", literal)]
    if len(nums) < 11:
        return None
    arc = nums[2:11]
    return {"weld_speed": nums[0], "org_weld_speed": nums[1], "sched": arc[0], "mode": arc[1],
            "voltage": arc[2], "wirefeed": arc[3], "control": arc[4], "current": arc[5]}


def expected_outputs(profile, welddata):
    """{short signal name: the value the welder output should carry} for the weld data in effect.
    The profile maps each output to a weld-data field ("sched", "wirefeed_ipm", "voltage")."""
    out = {}
    for path, spec in profile.get("welder_outputs", {}).items():
        name = path.split("/")[-1]
        if not welddata:
            out[name] = None
        elif spec["from"] == "wirefeed_ipm":
            out[name] = round(welddata["wirefeed"] / IPM_TO_MMS)
        else:
            out[name] = welddata.get(spec["from"])
    return out


def _pointer(sample):
    """The pointer to follow: the motion pointer when RWS fills it, else the program pointer."""
    p = sample.get("pointers")
    if not p:
        return None
    return p[1] or p[0]


def _edge(samples, i):
    """Time of an edge between samples i-1 and i, and its uncertainty."""
    if i <= 0:
        return samples[0]["t"], 0.0
    a, b = samples[i - 1]["t"], samples[i]["t"]
    return round((a + b) / 2, 4), round((b - a) / 2, 4)


def span_window(samples, module, span):
    """(first index on the span, first index after it, first index on its welding lines or None)."""
    lines = set(span["lines"])
    weld_lines = set(span["lines"][1:])
    first = weld = None
    for i, s in enumerate(samples):
        p = _pointer(s)
        on = p is not None and p[0] == module and p[2] in lines
        if first is None:
            if on:
                first = i
                if p[2] in weld_lines:
                    weld = i
        elif on:
            if weld is None and p[2] in weld_lines:
                weld = i
        else:
            return first, i, weld
    return (first, len(samples), weld) if first is not None else None


def _progress(p, polyline):
    """Distance along the planned polyline of the point on it closest to p."""
    best, along, done = None, 0.0, 0.0
    for a, b in zip(polyline, polyline[1:]):
        seg = math.dist(a, b)
        if seg > 0:
            t = max(0.0, min(1.0, sum((pi - ai) * (bi - ai) for pi, ai, bi in zip(p, a, b)) / seg ** 2))
            q = [ai + t * (bi - ai) for ai, bi in zip(a, b)]
            d = math.dist(p, q)
            if best is None or d < best:
                best, along = d, done + t * seg
        done += seg
    return along


def _travel(samples, lo, hi, key, polyline):
    """When the TCP left the span's first target and reached its last, between samples lo..hi-1,
    read in the span's work object. ``polyline`` = the span's targets in order (mm, that wobj).

    Each sample is placed on the planned polyline (its progress along it, so a weave's
    side-to-side motion is not counted as travel). The samples strictly inside the seam give the
    steady speed (least squares of progress against time); the departure and the arrival are
    extrapolated from the first and last of them at that speed. Sampling is ~20 Hz and a fast
    weld covers 0.6 mm per tick, so waiting for the TCP to be seen AT a target would miss it.
    """
    pts = [(samples[i]["t"], samples[i][key]) for i in range(lo, hi) if key in samples[i]]
    out = {"move_on": None, "move_off": None, "travel_seconds": None, "edge_uncertainty": None,
           "path_mm": None, "speed_mm_s": None, "tcp_samples": len(pts),
           "start_err_mm": None, "end_err_mm": None}
    if len(pts) < 2:
        return out
    out["start_err_mm"] = round(min(math.dist(p, polyline[0]) for _, p in pts), 3)
    out["end_err_mm"] = round(min(math.dist(p, polyline[-1]) for _, p in pts), 3)
    length = sum(math.dist(a, b) for a, b in zip(polyline, polyline[1:]))
    if length <= TARGET_TOL_MM:
        # The span starts and ends at the same place: there is nothing to travel (E59).
        out.update(move_on=pts[0][0], move_off=pts[0][0], travel_seconds=0.0, edge_uncertainty=0.0,
                   path_mm=0.0)
        return out
    prog = [(t, _progress(p, polyline), p) for t, p in pts]
    inner = [k for k, (_, d, _) in enumerate(prog) if TARGET_TOL_MM < d < length - TARGET_TOL_MM]
    if len(inner) < 2:
        return out
    k0, k1 = inner[0], inner[-1]
    ts = [prog[k][0] for k in inner]
    ds = [prog[k][1] for k in inner]
    mt, md = sum(ts) / len(ts), sum(ds) / len(ds)
    var = sum((t - mt) ** 2 for t in ts)
    if var <= 0:
        return out
    speed = sum((t - mt) * (d - md) for t, d in zip(ts, ds)) / var
    if speed <= 0:
        return out
    depart = prog[k0][0] - prog[k0][1] / speed
    arrive = prog[k1][0] + (length - prog[k1][1]) / speed
    gaps = [prog[k0][0] - prog[k0 - 1][0] if k0 > 0 else 0.0,
            prog[k1 + 1][0] - prog[k1][0] if k1 + 1 < len(prog) else 0.0]
    first = max(k0 - 1, 0)
    last = min(k1 + 1, len(prog) - 1)
    path = sum(math.dist(prog[k][2], prog[k + 1][2]) for k in range(first, last))
    out.update(move_on=round(depart, 4), move_off=round(arrive, 4), travel_seconds=round(arrive - depart, 4),
               edge_uncertainty=round(max(gaps) / 2, 4), path_mm=round(path, 3), speed_mm_s=round(speed, 3))
    return out


def _high_interval(samples, key, t_from, t_to):
    """First 0->1 rise in [t_from, t_to] of a 0/1 channel and the 1->0 fall after it (the fall
    may come after t_to). Edge times are midpoints between that channel's own readings."""
    pts = [(s["t"], s[key]) for s in samples if key in s and s["t"] >= t_from]
    out = {"on": None, "off": None, "seconds": None, "uncertainty": None}
    rise = None
    for k in range(1, len(pts)):
        (ta, va), (tb, vb) = pts[k - 1], pts[k]
        if rise is None:
            if tb > t_to:
                break
            if va == "0" and vb == "1":
                rise = ((ta + tb) / 2, (tb - ta) / 2)
        elif va == "1" and vb == "0":
            fall = ((ta + tb) / 2, (tb - ta) / 2)
            out.update(on=round(rise[0], 4), off=round(fall[0], 4), seconds=round(fall[0] - rise[0], 4),
                       uncertainty=round(max(rise[1], fall[1]), 4))
            return out
    if rise is not None:
        out["on"] = round(rise[0], 4)
    return out


def _positioner(samples, key, info, span, axis_map, t_on, t_off):
    """The positioner a coordinated span's work object moves with: where the module puts its
    axes at the span's first and last target (the robtargets' extax) and where they were. The
    span starts and ends with a fine stop (ArcLStart, ArcLEnd), so the readings just before the
    TCP left and just after it arrived are standstill readings, free of sampling skew.
    ``axis_map`` = the profile's {"eax_b": "rax_1", ...} for this unit; without it the axes are
    reported but cannot be paired. With it, only the mapped axes are reported: the module's
    other extax belong to other units (MONARC exports eax_a for a rail the cell does not have)."""
    pts = [(s["t"], s[key]) for s in samples if s.get(key)]
    before = [v for t, v in pts if t <= t_on]
    after = [v for t, v in pts if t >= t_off]
    during = [v for t, v in pts if t_on <= t <= t_off]
    out = {"readings": len(pts), "axes": {}}
    first = info.extax.get(span["targets"][0], {})
    last = info.extax.get(span["targets"][-1], {})
    for eax in sorted(set(first) | set(last)):
        if axis_map and eax not in axis_map:
            continue
        rax = (axis_map or {}).get(eax)
        row = {"rax": rax, "planned": [first.get(eax), last.get(eax)], "observed": [None, None],
               "range_during": None}
        if rax:
            row["observed"][0] = _standstill([float(v[rax]) for v in before if rax in v], last=True)
            row["observed"][1] = _standstill([float(v[rax]) for v in after if rax in v], last=False)
            vals = [float(v[rax]) for v in during if rax in v]
            if vals:
                row["range_during"] = [min(vals), max(vals)]
        out["axes"][eax] = row
    return out


def _standstill(values, last, tol_deg=0.01):
    """The axis value at the fine stop next to the seam: of the readings before the seam
    (``last``) the last one that repeats its neighbour, of those after it the first. The seam's
    start and end times are fitted from the TCP, so the reading nearest them can already be on
    the move: a plate turning 64 deg/s moves 9 deg in the fit's 0.14 s (part 461784, weld 3).
    Falls back to the nearest reading when the axis never stood still in the window."""
    if not values:
        return None
    pairs = list(zip(values, values[1:]))
    if last:
        pairs.reverse()
    for a, b in pairs:
        if abs(a - b) <= tol_deg:
            return b if not last else a
    return values[-1] if last else values[0]


def oframe_of(wobjdata):
    """[x, y, z, q1, q2, q3, q4] of a wobjdata literal's object frame, or None."""
    groups = re.findall(r"\[([^\[\]]*)\]", wobjdata or "")
    if len(groups) < 4:
        return None
    return [float(v) for v in groups[2].split(",")] + [float(v) for v in groups[3].split(",")]


def module_oframe(text, wobj):
    """The object frame the module itself assigns to ``wobj`` (the planner's seed / nominal)."""
    m = re.search(r"\b%s\.oframe\s*:=\s*\[\[([^\]]*)\],\[([^\]]*)\]\]" % re.escape(wobj), text or "")
    if not m:
        return None
    return [float(v) for v in m.group(1).split(",")] + [float(v) for v in m.group(2).split(",")]


def _distinct(samples, key, t_from, t_to):
    seen = []
    for s in samples:
        if key in s and t_from <= s["t"] <= t_to and s[key] not in seen:
            seen.append(s[key])
    return seen


def analyse(recording):
    samples = recording["samples"]
    profile = recording["profile"]
    tool = profile["tool"]
    mod = recording["module"]
    out = {"spans": [], "sampling": {}, "events": {}}

    dts = sorted(s["dt"] for s in samples)
    if dts:
        gaps = [b["t"] - a["t"] for a, b in zip(samples, samples[1:])]
        out["sampling"] = {"ticks": len(samples), "median_tick_s": dts[len(dts) // 2],
                           "max_gap_s": round(max(gaps), 4) if gaps else None,
                           "rws_requests": recording["rws"]["requests"],
                           "read_errors": len(recording["rws"]["read_errors"])}

    codes = {}
    for e in recording["events"]:
        codes.setdefault(e["code"], []).append(e["title"])
    out["events"] = {"motion_50xxx": sorted(c for c in codes if c.startswith("50")),
                     "process_errors": sorted(c for c in codes if c.startswith("11") or c.startswith("80001")),
                     "errors": [e for e in recording["events"] if e["type"] == "3"],
                     "warnings": [e for e in recording["events"] if e["type"] == "2"]}

    if not mod.get("text"):
        return out
    info = ModuleInfo(mod["text"])
    reads = {r["name"]: r for r in mod.get("local_reads", [])}
    on_key = "signal:" + profile["weld_on_signal"]
    ad_key = "signal:" + profile["arc_detect_signal"]
    for n, span in enumerate(info.arc_spans(), start=1):
        row = {"n": n, "start_line": span["start_line"], "end_line": span.get("end_line"),
               "targets": span["targets"], "wobj": span["wobj"], "weave": span["weave"],
               "planned_mm": round(info.planned_length(span), 3), "observed": False}
        if span["weave"]:
            decl = info.declarations.get(span["weave"])
            row["weave_declared"] = decl[2] if decl else None
            row["weave_read"] = reads.get(span["weave"], {}).get("value")
        window = span_window(samples, mod["module"], span)
        if window is None:
            out["spans"].append(row)
            continue
        i0, i1, iw = window
        row["observed"] = True
        row["pointer_on"], _ = _edge(samples, i0)
        row["pointer_off"], _ = _edge(samples, i1) if i1 < len(samples) else (samples[-1]["t"], 0.0)
        row["pointer_weld_on"] = _edge(samples, iw)[0] if iw is not None else None
        row["pointer_seconds"] = round(row["pointer_off"] - row["pointer_on"], 4)
        polyline = [info.robtargets.get(t) for t in span["targets"]]
        if polyline and all(polyline):
            row.update(_travel(samples, max(i0 - 1, 0), i1, tcp_key(tool, span["wobj"]), polyline))
        t_from = samples[max(i0 - 1, 0)]["t"]
        t_to = samples[min(i1, len(samples) - 1)]["t"]
        frames = [s[k] for s in samples for k in s
                  if k.startswith("symbol:") and k.endswith(":" + span["wobj"])
                  and row["pointer_on"] <= s["t"] <= t_to]
        seen = []
        for f in (oframe_of(v) for v in frames):
            if f is not None and f not in seen:
                seen.append(f)
        row["frames_seen"] = seen
        row["frame_module"] = module_oframe(mod["text"], span["wobj"])
        unit = mod.get("mech_units", {}).get(span["wobj"])
        if unit:
            t_on = row.get("move_on") if row.get("move_on") is not None else t_from
            t_off = row.get("move_off") if row.get("move_off") is not None else t_to
            row["positioner"] = dict(unit=unit, **_positioner(
                samples, "joints:" + unit, info, span, profile.get("positioner_axes", {}).get(unit),
                t_on, t_off))
        row["weld_on"] = _high_interval(samples, on_key, t_from - 1.0, t_to + 1.0)
        row["arc_detect"] = _high_interval(samples, ad_key, t_from - 1.0, t_to + 1.0)
        t_weld = row["pointer_weld_on"] if row["pointer_weld_on"] is not None else row["pointer_on"]
        wd = _distinct(samples, "symbol:TG_Weld:wdTG_Weld", t_weld, t_to)
        row["welddata_seen"] = wd
        row["welddata"] = parse_welddata(wd[-1]) if wd else None
        row["outputs_seen"] = {k.split("/")[-1]: _distinct(samples, "signal:" + k, t_weld, t_to)
                               for k in profile.get("welder_outputs", {})}
        out["spans"].append(row)
    return out
