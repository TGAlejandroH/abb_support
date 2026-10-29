"""Robtargets that differ between a module file and the main module baked in a .tgs, and the
.tgs's touch-up archive entries (teach-pendant import check).

    python compare_bakes.py <before.mod> <project.tgs>
"""
import pathlib
import re
import sqlite3
import sys


def targets(text):
    return dict(re.findall(r"(pTG\d+)\s*:=\s*\[\[([^\]]*)\]", text))


def main(before_path, tgs):
    con = sqlite3.connect(pathlib.Path(tgs).as_uri() + "?mode=ro", uri=True)
    rows = [r[0] for r in con.execute("select path from robot_bundle_files where path like 'ROBOT_PROGRAM/%'")]
    print("archived touch-ups:", [r for r in rows if "/touchups/" in r])
    mains = [r for r in rows if r.count("/") == 1 and not re.search(r"_\d{8}_\d{6}\.mod$", r)]
    after = con.execute("select content_blob from robot_bundle_files where path=?", (mains[0],)).fetchone()[0]
    after = after.decode("utf-8", "replace")
    with open(before_path, encoding="utf-8") as fh:
        before = fh.read()
    a, b = targets(before), targets(after)
    for k in sorted(set(a) | set(b)):
        if a.get(k) != b.get(k):
            print("%s: %s -> %s" % (k, a.get(k), b.get(k)))
    return after


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
