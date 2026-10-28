"""Human-readable lines for a recording (printed by recorder.py and step0_probe.py)."""


def _f(v, fmt="%.3f"):
    return "-" if v is None else fmt % v


def summary_lines(recording):
    a = recording["analysis"]
    s = a.get("sampling", {})
    mod = recording["module"]
    lines = ["run: %s .. %s, ended because %s" % (recording["started"], recording["ended"], recording["ended_because"]),
             "module: %s (%s)" % (mod.get("module"), mod.get("file")),
             "sampling: %s ticks, median tick %s s, max gap %s s, %s RWS requests, %s read errors"
             % (s.get("ticks"), s.get("median_tick_s"), s.get("max_gap_s"), s.get("rws_requests"),
                s.get("read_errors"))]
    for sp in a.get("spans", []):
        if not sp["observed"]:
            lines.append("weld %d (lines %s-%s, %s): NOT OBSERVED - the program pointer never reached it"
                         % (sp["n"], sp["start_line"], sp["end_line"], " -> ".join(sp["targets"])))
            continue
        wd = sp.get("welddata") or {}
        lines.append("weld %d (lines %s-%s, %s in %s): planned %s mm"
                     % (sp["n"], sp["start_line"], sp["end_line"], " -> ".join(sp["targets"]), sp["wobj"],
                        _f(sp["planned_mm"], "%.1f")))
        lines.append("   travel %s mm in %s s (+-%s) at %s mm/s; weld-on %s s (+-%s); arc detect %s s; pointer on span %s s"
                     % (_f(sp.get("path_mm"), "%.2f"), _f(sp.get("travel_seconds")), _f(sp.get("edge_uncertainty")),
                        _f(sp.get("speed_mm_s")), _f(sp["weld_on"]["seconds"]), _f(sp["weld_on"]["uncertainty"]),
                        _f(sp["arc_detect"]["seconds"]), _f(sp.get("pointer_seconds"))))
        lines.append("   weld data: weld_speed %s mm/s, sched %s, wirefeed %s mm/s, voltage %s, control %s%s"
                     % (wd.get("weld_speed"), wd.get("sched"), wd.get("wirefeed"), wd.get("voltage"), wd.get("control"),
                        "" if len(sp.get("welddata_seen", [])) <= 1 else
                        "  (CHANGED during the weld: %s)" % sp["welddata_seen"]))
        lines.append("   welder outputs: %s" % sp.get("outputs_seen"))
        pos = sp.get("positioner")
        if pos:
            for eax, ax in sorted(pos["axes"].items()):
                rng = ax.get("range_during")
                lines.append("   positioner %s %s (%s): %s -> %s deg, planned %s -> %s, range during the seam %s"
                             % (pos["unit"], eax, ax["rax"] or "unmapped", _f(ax["observed"][0], "%.2f"),
                                _f(ax["observed"][1], "%.2f"), _f(ax["planned"][0], "%.2f"),
                                _f(ax["planned"][1], "%.2f"),
                                "-" if not rng else "%.2f..%.2f" % tuple(rng)))
        if sp.get("weave"):
            lines.append("   weave %s: declared %s, read %s" % (sp["weave"], sp.get("weave_declared"), sp.get("weave_read")))
    ev = a.get("events", {})
    lines.append("events: %d new; errors %s; warnings %s; motion %s"
                 % (len(recording["events"]), [e["code"] for e in ev.get("errors", [])],
                    [e["code"] for e in ev.get("warnings", [])], ev.get("motion_50xxx")))
    return lines
