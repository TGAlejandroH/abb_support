"""Stand-in for a teach-pendant Modify Position (VC only): read the program the HMI sent from the
controller ($home/TGS/<prog>.mod), move named robtargets, and write the result as the controller's
own save would (CRLF). Optionally stage it where the HMI's retrieve looks ($home/TGS/edited/) and
raise nTG_ProgEdited, as TG's unload-keep-edits does after a pendant edit.

    python pendant_edit.py <prog> <out.mod> pTG0010:0,0,2 pTG0037:1.5,0,0 [--stage]

The pendant itself is not used: Modify Position needs MANUAL mode and a FlexPendant client
(plan section 16.4.1 of the planner's teach-pendant touch-up plan); the owner ran that path on
this VC on 2026-10-06. What follows the edit (retrieve, import, regenerate) is the real code.
"""
import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from rws_reader import RwsReader  # noqa: E402


def move_target(text, name, dxyz):
    pat = re.compile(r"(\b%s\s*:=\s*\[\[)([^\]]*)(\])" % re.escape(name))
    m = pat.search(text)
    if not m:
        raise SystemExit("no robtarget %s in the module" % name)
    xyz = [float(v) for v in m.group(2).split(",")]
    new = [round(a + b, 3) for a, b in zip(xyz, dxyz)]
    # The controller prints a ModPos'ed robtarget with its own number format; 3 decimals here.
    text = text[:m.start(2)] + ",".join("%.3f" % v for v in new) + text[m.end(2):]
    return text, xyz, new


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("prog")
    ap.add_argument("out")
    ap.add_argument("moves", nargs="+", help="robtarget:dx,dy,dz in mm (its work object)")
    ap.add_argument("--stage", action="store_true")
    args = ap.parse_args(argv)
    r = RwsReader()
    raw = r.get_file("$home/TGS/%s.mod" % args.prog)
    text = raw.decode("utf-8") if isinstance(raw, bytes) else raw
    text = text.replace("\r\n", "\n")
    for mv in args.moves:
        name, d = mv.split(":")
        text, before, after = move_target(text, name, [float(v) for v in d.split(",")])
        print("%s: %s -> %s" % (name, before, after))
    data = text.replace("\n", "\r\n").encode("utf-8")
    with open(args.out, "wb") as fh:
        fh.write(data)
    print("written %s (%d bytes)" % (args.out, len(data)))
    if args.stage:
        sys.path.insert(0, os.path.join(os.path.dirname(HERE), "..", "..", "hmi_prototype"))
        from rws_session import RwsSession
        import bench_cycle
        s = RwsSession(bench_cycle.URL)
        try:
            s.create_directory("$home/TGS", "edited")
        except Exception:
            pass  # already there
        s.put_file("$home/TGS/edited/%s.mod" % args.prog, data)
        s.request_mastership()
        try:
            s.set_symbol("nTG_ProgEdited", "1")
        finally:
            s.release_mastership()
        print("staged $home/TGS/edited/%s.mod, nTG_ProgEdited := 1" % args.prog)


if __name__ == "__main__":
    main(sys.argv[1:])
