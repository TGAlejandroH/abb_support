#!/usr/bin/env python3
"""tg_deploy.py - replace the resident TG RAPID modules on a controller over RWS.

Built for the MONARC real cell (4600-804589, WAN 10.8.8.56), where the
2026-10-21 build sits in HOME:/TGS and is loaded in T_ROB1. Standard library
only; reuses comms_probe.py (same folder) for the RWS session and the log.
Runbook: README.md in this folder, section "Replacing the TG modules".

THE SET (load order): TG_CellCal.sys, TG_Comms.sys, TG_Cell.sys, TG_Touch.sys,
TG_Weld.sys, TG_Main.mod, TG_Parts.mod. Leftover from the 2026-09-24 comms test:
TG_SocketProbe_Mod / TG_SocketProbe.mod (unloaded and deleted with the set).

COMMANDS, in the order they are used on site (all need --ip)

  status   Read-only. Controller identity, opmode, RAPID state, PP; which TG
           modules are loaded; HOME:/TGS listing compared byte-for-byte with
           the latest stage; live PERS values compared with the stage files;
           global-name clashes between the new files and non-TG modules.
           Saves a PERS snapshot (JSON) every time.
  backup   Read-only on the controller. Downloads every file in HOME:/TGS to
           logs/deploy_<stamp>/backup/ and saves a PERS snapshot beside it.
           Run it BEFORE unloading anything: unloading a module drops its
           live PERS values (tool calibration, taught part data).
  stage    Local only. Copies the set from abb/rapid into logs/deploy_<stamp>/
           stage/, carries the cell's live configuration PERS from the latest
           snapshot into the copies (tTG_Cam/tTG_Weld calibration, touch I/O
           names, PM part data, ...), checks the site edits (server IP,
           tgs_main, camera SetDO off), runs rapid_check (per file + F-5
           duplicate globals across the set), writes manifest.json.
  unload   RWS, needs AUTO + RAPID stopped (MANUAL: do it on the pendant).
           unloadmod the TG modules present, reverse load order.
  delete   Deletes the old TG files from HOME:/TGS - only those that the
           latest backup holds byte-identical. Other files are left alone.
  upload   PUTs the latest stage to HOME:/TGS and reads every file back.
  load     RWS, needs AUTO + RAPID stopped (MANUAL: do it on the pendant).
           loadmod the set in load order; verdict from the event log.

Nothing here starts RAPID: Production Manager owns the cell. After a load,
reset PP / start from the pendant as usual.

  python tg_deploy.py --ip 10.8.8.56 status
"""

import argparse
import datetime as _dt
import glob
import hashlib
import importlib.util
import json
import os
import re
import shutil
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import comms_probe as cp  # noqa: E402

REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
SRC_DIR = os.path.join(REPO, "abb", "rapid")
CTRL_DIR = "$home/TGS"

SET = ["TG_CellCal.sys", "TG_Comms.sys", "TG_Cell.sys", "TG_Touch.sys",
       "TG_Weld.sys", "TG_Main.mod", "TG_Parts.mod"]
LEFTOVER = {"TG_SocketProbe.mod": "TG_SocketProbe_Mod"}


def mod_name(fname):
    return LEFTOVER.get(fname, os.path.splitext(fname)[0])


ALL_FILES = SET + list(LEFTOVER)
ALL_MODULES = [mod_name(f) for f in ALL_FILES]

# PERS that hold the CELL's configuration: when the live value differs from the
# file, the live value wins and 'stage' writes it into the copy. Everything else
# (counters, flags, served frames, weld parameters sent per run) is run-time
# state that a fresh load may reset.
CONFIG_PERS = [
    "tTG_Cam", "tTG_Weld",                                    # TCP calibration
    "stTG_ServerIP", "nTG_Port", "nTG_HandshakePort", "nTG_SettleTime",
    "stTG_TouchDI", "stTG_TouchOnDO", "stTG_TouchActiveDI", "nTG_TouchActiveWait",
    "stTG_TeachModeDO", "bTG_TouchLive",
    "nTG_TouchSpeed", "nTG_TouchStroke", "nTG_TouchClearWait",
    "pdTG_9011", "padvTG_9011", "pdTG_9022", "padvTG_9022",   # PM part data
]
# The generic TG_CellCal.sys values. A cell must never run on these.
PLACEHOLDER_TOOLS = {
    "tTG_Weld": "[TRUE,[[0,0,200],[1,0,0,0]],[1,[0,0,100],[1,0,0,0],0,0,0]]",
    "tTG_Cam": "[TRUE,[[0,0,100],[1,0,0,0]],[1,[0,0,50],[1,0,0,0],0,0,0]]",
}
# Where the 2026-10-21 build declared a PERS that has since moved module.
OLD_HOMES = {"tTG_Cam": "TG_Comms", "tTG_Weld": "TG_Comms"}

FATAL_ELOG = ("40322", "40223")      # syntax error / module not loaded
LINK_ELOG = "40160"                  # unresolved reference (transient between loads)


# ============================================================ RAPID text helpers

_PERS_RE = re.compile(
    r"^[ \t]*(?:(LOCAL|TASK)[ \t]+)?PERS[ \t]+(\w+)[ \t]+(\w+)(\{[^}]*\})?[ \t]*:=(.*?);[ \t\r]*(?:![^\n]*)?$",
    re.M | re.S)


def pers_decls(text):
    """{name: (type, initial value text, single_line, local)} for module-level PERS."""
    out = {}
    for m in _PERS_RE.finditer(text):
        scope, typ, name, dims, value = m.groups()
        out[name] = (typ, value.strip(), "\n" not in m.group(0), scope == "LOCAL")
    return out


_TOKEN = re.compile(r'"(?:[^"\\]|\\.)*"|[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?|[A-Za-z_]\w*')


def norm_value(text):
    """Comparable form of a RAPID literal: whitespace gone, numbers to 6 significant digits
    (the controller prints 9E+09 where the file says 9E9), TRUE/FALSE case-folded, strings
    kept exactly."""
    text = re.sub(r"\s+", "", text or "")

    def sub(m):
        tok = m.group(0)
        if tok.startswith('"'):
            return tok
        if tok[0].isalpha() or tok[0] == "_":
            return tok.upper()
        return "%.6g" % float(tok)
    return _TOKEN.sub(sub, text)


def same_value(a, b):
    return norm_value(a) == norm_value(b)


def read_bytes(path):
    with open(path, "rb") as fh:
        return fh.read()


def sha(data):
    return hashlib.sha256(data).hexdigest()[:16]


def load_rapid_check():
    path = os.path.join(REPO, "tools", "rapid_check.py")
    spec = importlib.util.spec_from_file_location("rapid_check", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ============================================================ local folders

def new_run_dir(log_dir):
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(log_dir, "deploy_" + stamp)
    os.makedirs(path, exist_ok=True)
    return path


def latest(log_dir, sub):
    hits = sorted(glob.glob(os.path.join(log_dir, "deploy_*", sub)))
    return hits[-1] if hits else None


def latest_snapshot_with_values(log_dir):
    """Newest snapshot that holds PERS values. A 'status' run after the old modules were
    unloaded writes an empty one, which must not hide the backup's values."""
    hits = sorted(glob.glob(os.path.join(log_dir, "deploy_*", "pers_snapshot.json")))
    for path in reversed(hits):
        try:
            with open(path, encoding="utf-8") as fh:
                if json.load(fh).get("pers"):
                    return path
        except (OSError, ValueError):
            continue
    return hits[-1] if hits else None


def stage_files(stage_dir):
    """{file name: bytes} of a stage folder, or of abb/rapid when there is no stage."""
    base = stage_dir or SRC_DIR
    return {f: read_bytes(os.path.join(base, f)) for f in SET if os.path.isfile(os.path.join(base, f))}


# ============================================================ controller reads

def ctrl_listing(rws):
    try:
        return {e.get("_title"): e for e in rws.list_dir(CTRL_DIR) if e.get("_type") == "fs-file"}
    except cp.RwsError as exc:
        if exc.kind == "NOTFOUND":
            return None
        raise


def read_pers(rws, task, module, name):
    path = "/rw/rapid/symbol/data/RAPID/%s/%s/%s" % (task, module, name)
    try:
        st = cp.state(rws.get_json(path))
    except cp.RwsError as exc:
        if exc.kind in ("NOTFOUND", "HTTP"):
            return None
        raise
    return st[0].get("value") if st else None


def snapshot_pers(rws, task, loaded, decl_home):
    """Live value of every non-LOCAL PERS the new set declares, from whichever loaded TG
    module holds it now (new home first, then the 2026-10-21 home)."""
    snap = {}
    for name, home in sorted(decl_home.items()):
        for module in [home, OLD_HOMES.get(name)]:
            if module and module in loaded:
                value = read_pers(rws, task, module, name)
                if value is not None:
                    snap[name] = {"module": module, "value": value}
                    break
    return snap


def new_set_decls(files):
    decls, home = {}, {}
    for f, data in files.items():
        for name, d in pers_decls(data.decode("ascii", "replace")).items():
            if not d[3]:
                decls[name] = d
                home[name] = mod_name(f)
    return decls, home


def save_snapshot(path, rws, task, loaded, snap, ident):
    doc = {"taken": _dt.datetime.now().isoformat(timespec="seconds"), "controller": ident,
           "rws": rws.base, "task": task, "loaded_tg_modules": [m for m in ALL_MODULES if m in loaded],
           "pers": snap}
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(doc, fh, indent=2)
    return path


def identity(rws):
    name = cp.state(rws.get_json("/rw/system"))[0].get("name", "?")
    opmode = cp.state(rws.get_json("/rw/panel/opmode"))[0].get("opmode", "?")
    ex = cp.exec_state(rws)
    return {"system": name, "opmode": opmode, "exec": ex}


def program_builds(rws, task):
    """Ask the controller to link the task's program. True = no error, False = semantic / link
    error (the controller names no symbol), None = could not ask. No mastership needed (VC
    2026-11-16); a 400 is the controller's 'program has errors'."""
    try:
        rws.post_form("/rw/rapid/tasks/" + task, {}, {"action": "build"})
        return True, ""
    except cp.RwsError as exc:
        said = cp.rws_status_message(exc.body)
        if exc.code == 400:
            return False, said
        return None, exc.explain()


def report_build(rws, task, log, step):
    ok, why = program_builds(rws, task)
    if ok:
        log.passed(step, "the controller links the whole %s program without error" % task)
    elif ok is False:
        log.failed(step, "the %s program has link/semantic errors (%s) - it will not run; the pendant "
                         "Program Editor shows where" % (task, why or "no detail"))
    else:
        log.warn(step, "could not ask the controller to build the program: %s" % why)
    return ok


def symbol_clashes(rws, task, files, log):
    """Global names of the new set that a non-TG module of the task already declares."""
    rc = load_rapid_check()
    ours = {}
    for f, data in files.items():
        for sym, _ in rc.global_symbols(data.decode("ascii", "replace")):
            ours.setdefault(sym.lower(), (sym, f))
    clashes, start = [], 0
    fields = {"view": "block", "blockurl": "RAPID/%s" % task, "recursive": "TRUE"}
    while True:
        _, body = rws.post_form("/rw/rapid/symbols", fields,
                                {"action": "search-symbols", "json": "1", "start": str(start), "limit": "500"})
        doc = json.loads(body.decode("utf-8", "replace"))
        page = cp.state(doc)
        for s in page:
            parts = s.get("symburl", "").split("/")
            if len(parts) != 4 or s.get("local") == "true":       # RAPID/task/module/name only
                continue
            module, name = parts[2], parts[3]
            if module in ALL_MODULES:
                continue
            if name.lower() in ours:
                clashes.append("%s (ours: %s) is also global in module %s"
                               % (ours[name.lower()][0], ours[name.lower()][1], module))
        if not page or "next" not in doc.get("_links", {}):
            break
        start += len(page)
    return clashes


# ============================================================ commands

def cmd_status(args, rws, log):
    ident = identity(rws)
    log.info("controller", "%s | opmode %s | RAPID %s" % (ident["system"], ident["opmode"], ident["exec"]))
    if args.expect_system and ident["system"] != args.expect_system:
        log.warn("controller", "system name is %s, expected %s - is this the right controller?"
                 % (ident["system"], args.expect_system))
    mod, rout = cp.pcp(rws, args.task)
    log.info("pp", "%s / %s" % (mod, rout))
    report_build(rws, args.task, log, "program")

    loaded = cp.module_names(rws, args.task)
    tg = [m for m in ALL_MODULES if m in loaded]
    log.info("modules", "TG modules loaded in %s: %s" % (args.task, ", ".join(tg) or "none"))
    if "TG_Comms" in loaded and "TG_CellCal" not in loaded:
        log.info("modules", "TG_CellCal not loaded: this is the pre-2026-10-24 build "
                            "(tools still declared in TG_Comms)")

    stage = latest(args.log_dir, "stage")
    files = stage_files(stage)
    log.info("reference", "comparing with %s" % (stage or SRC_DIR + " (no stage yet)"))

    listing = ctrl_listing(rws)
    if listing is None:
        log.info("tgs", "HOME:/TGS does not exist")
    else:
        others = sorted(n for n in listing if n not in ALL_FILES)
        for f in ALL_FILES:
            if f not in listing:
                log.detail("%-20s absent" % f)
                continue
            data = rws.get_file(CTRL_DIR + "/" + f)
            ref = files.get(f)
            verdict = ("identical to reference" if ref == data else
                       "DIFFERS from reference" if ref is not None else "not in the new set")
            log.say("  HOME:/TGS/%-20s %6d bytes  sha %s  %s  (mdate %s)"
                    % (f, len(data), sha(data), verdict, listing[f].get("fs-mdate", "?")))
        log.info("tgs", "other files in HOME:/TGS (left alone): %s" % (", ".join(others) or "none"))

    decls, home = new_set_decls(files)
    snap = snapshot_pers(rws, args.task, loaded, home)
    path = save_snapshot(os.path.join(new_run_dir(args.log_dir), "pers_snapshot.json"),
                         rws, args.task, loaded, snap, ident)
    report_pers(log, snap, decls)
    log.info("snapshot", path)

    try:
        clashes = symbol_clashes(rws, args.task, files, log)
    except cp.RwsError as exc:
        log.warn("names", "symbol search failed: " + exc.explain())
    else:
        if clashes:
            for c in clashes:
                log.detail(c)
            log.failed("names", "%d global name(s) clash with non-TG modules: %s"
                       % (len(clashes), "; ".join(clashes[:5])))
        else:
            log.passed("names", "no global name of the new set is declared by a non-TG module")
    return True


def report_pers(log, snap, decls):
    if not snap:
        log.info("pers", "no TG PERS readable (no TG module loaded)")
        return
    for name in CONFIG_PERS:
        if name not in snap or name not in decls:
            continue
        live, fileval = snap[name]["value"], decls[name][1]
        if same_value(live, fileval):
            log.detail("config %-20s live == file" % name)
        else:
            log.say("  CONFIG %-20s live %s\n         %-20s file %s" % (name, live, "", fileval))
    diff_cfg = [n for n in CONFIG_PERS if n in snap and n in decls
                and not same_value(snap[n]["value"], decls[n][1])]
    diff_run = [n for n in snap if n not in CONFIG_PERS and n in decls
                and not same_value(snap[n]["value"], decls[n][1])]
    log.info("pers", "%d PERS read; configuration differing from the files: %s"
             % (len(snap), ", ".join(diff_cfg) or "none"))
    log.info("pers", "run-time PERS differing (a fresh load resets them, expected): %s"
             % (", ".join(diff_run) or "none"))


def cmd_backup(args, rws, log):
    ident = identity(rws)
    log.info("controller", "%s | opmode %s | RAPID %s" % (ident["system"], ident["opmode"], ident["exec"]))
    run = new_run_dir(args.log_dir)
    bdir = os.path.join(run, "backup")
    os.makedirs(bdir)
    listing = ctrl_listing(rws)
    if listing is None:
        log.info("backup", "HOME:/TGS does not exist - nothing to download")
        listing = {}
    for name in sorted(listing):
        data = rws.get_file(CTRL_DIR + "/" + name)
        with open(os.path.join(bdir, name), "wb") as fh:
            fh.write(data)
        log.detail("saved %-24s %7d bytes sha %s" % (name, len(data), sha(data)))
    log.passed("backup", "%d file(s) from HOME:/TGS saved to %s" % (len(listing), bdir))

    loaded = cp.module_names(rws, args.task)
    decls, home = new_set_decls(stage_files(None))
    snap = snapshot_pers(rws, args.task, loaded, home)
    path = save_snapshot(os.path.join(run, "pers_snapshot.json"), rws, args.task, loaded, snap, ident)
    report_pers(log, snap, decls)
    log.passed("snapshot", "%d live PERS saved to %s" % (len(snap), path))
    return True


def cmd_stage(args, log):
    snap_path = args.snapshot or latest_snapshot_with_values(args.log_dir)
    snap = {}
    if args.no_carry:
        log.info("carry", "--no-carry: the files go up exactly as in abb/rapid")
    elif snap_path:
        with open(snap_path, encoding="utf-8") as fh:
            doc = json.load(fh)
        system = doc.get("controller", {}).get("system")
        taken = doc.get("taken", "")
        log.info("carry", "live values from %s (taken %s on %s)" % (snap_path, taken, system))
        # A snapshot from another controller (a VC rehearsal) or an old visit would carry the
        # wrong calibration into this cell's files: refuse unless named on purpose.
        if not args.snapshot:
            age_h = (_dt.datetime.now() - _dt.datetime.fromisoformat(taken)).total_seconds() / 3600
            if system != args.expect_system:
                log.failed("carry", "latest snapshot is from %s, not %s - run 'backup' on the cell "
                                    "first, or name a snapshot with --snapshot" % (system, args.expect_system))
                return False
            if age_h > 12:
                log.failed("carry", "latest snapshot is %.0f h old - run 'backup' again first, or name "
                                    "it with --snapshot" % age_h)
                return False
        snap = doc.get("pers", {})
        if not snap:
            log.info("carry", "the snapshot holds no PERS (no TG module was loaded) - nothing to carry")
    else:
        log.failed("carry", "no PERS snapshot found - run 'backup' against the controller first, "
                            "or pass --no-carry for a controller that never had the TG modules")
        return False

    # Three-way merge per configuration PERS: base = the value in the OLD file the controller
    # loaded (from the backup taken with the snapshot), live = the controller now, new = the
    # file going up. live == base: nobody changed it on the controller, the new file wins (this
    # is how a TCP updated in the repo replaces a dummy still on the cell). live != base: it was
    # set on the controller, the live value is carried - unless the repo changed it too, which
    # is a conflict the operator settles with --prefer-file NAME (or by fixing the file).
    base = {}
    bdir = os.path.join(os.path.dirname(snap_path), "backup") if snap_path else None
    if snap and bdir and os.path.isdir(bdir):
        for fname in sorted(os.listdir(bdir)):
            if fname in ALL_FILES:
                base.update(pers_decls(read_bytes(os.path.join(bdir, fname)).decode("latin-1")))
        log.info("carry", "base values from the old files in %s" % bdir)
    elif snap:
        log.warn("carry", "no backup beside the snapshot - every live value that differs from the "
                          "new files is carried (two-way); run 'backup' to get the three-way merge")
    prefer_file = {n.strip() for n in (args.prefer_file or "").split(",") if n.strip()}

    run = new_run_dir(args.log_dir)
    sdir = os.path.join(run, "stage")
    os.makedirs(sdir)
    carried = []
    texts = []
    conflicts = []
    for f in SET:
        raw = read_bytes(os.path.join(SRC_DIR, f))
        text = raw.decode("latin-1")        # byte-exact; rapid_check below rejects non-ASCII
        for name, (typ, value, single, local) in pers_decls(text).items():
            if local or name not in CONFIG_PERS or name not in snap:
                continue
            live = snap[name]["value"]
            if same_value(live, value):
                continue
            if name in base:
                old = base[name][1]
                if same_value(live, old):
                    log.say("  new   %-20s -> %s : %s (controller still had the old file's %s)"
                            % (name, f, value, old))
                    continue
                if not same_value(value, old):
                    if name in prefer_file:
                        log.say("  new   %-20s -> %s : %s (--prefer-file; controller had %s)"
                                % (name, f, value, live))
                        continue
                    conflicts.append(name)
                    log.say("  CONFLICT %-17s set on the controller to %s AND changed in the repo "
                            "to %s" % (name, live, value))
                    continue
            if not single:
                log.warn("carry", "%s differs live but spans several lines in %s - not carried" % (name, f))
                continue
            pat = re.compile(r"(PERS[ \t]+%s[ \t]+%s[ \t]*:=)%s;" % (typ, name, re.escape(value)))
            text, n = pat.subn(lambda m: m.group(1) + live + ";", text, count=1)
            if n != 1:
                log.failed("carry", "could not place %s in %s" % (name, f))
                return False
            carried.append({"name": name, "file": f, "file_value": value, "live_value": live})
            log.say("  carry %-20s -> %s : %s" % (name, f, live))
        data = text.encode("latin-1")
        with open(os.path.join(sdir, f), "wb") as fh:
            fh.write(data)
        texts.append((f, text))
    log.info("carry", "%d configuration PERS carried from the controller" % len(carried))

    ok = site_checks(args, dict(texts), log)
    if conflicts:
        log.failed("carry", "conflict on %s: keep the repo value with --prefer-file %s, or put the "
                            "controller's value into abb/rapid" % (", ".join(conflicts), ",".join(conflicts)))
        ok = False

    rc = load_rapid_check()
    findings = []
    for f in SET:
        findings += rc.check_file(os.path.join(sdir, f))
    findings += rc.check_duplicate_globals(texts)
    if findings:
        for fi in findings:
            log.detail(fi)
        log.failed("rapid_check", "%d finding(s) - see the log; fix before uploading" % len(findings))
        ok = False
    else:
        log.passed("rapid_check", "7 files clean, no duplicate globals across the set")

    manifest = {"built": _dt.datetime.now().isoformat(timespec="seconds"), "source": SRC_DIR,
                "git_head": git_head(), "snapshot": snap_path if snap else None, "carried": carried,
                "files": [{"name": f, "bytes": len(read_bytes(os.path.join(sdir, f))),
                           "sha256_16": sha(read_bytes(os.path.join(sdir, f)))} for f in SET]}
    with open(os.path.join(sdir, "manifest.json"), "w", encoding="utf-8") as fh:
        json.dump(manifest, fh, indent=2)
    for e in manifest["files"]:
        log.say("  %-16s %6d bytes  sha %s" % (e["name"], e["bytes"], e["sha256_16"]))
    if not ok:
        shutil.move(sdir, sdir + "_REJECTED")
        log.failed("stage", "stage rejected (renamed *_REJECTED so 'upload' cannot pick it)")
        return False
    log.passed("stage", "upload set ready in %s" % sdir)
    return True


def git_head():
    try:
        with open(os.path.join(REPO, ".git", "HEAD")) as fh:
            ref = fh.read().strip()
        if ref.startswith("ref: "):
            with open(os.path.join(REPO, ".git", *ref[5:].split("/"))) as fh:
                return ref[5:] + " @ " + fh.read().strip()[:10]
        return ref[:10]
    except OSError:
        return "?"


def site_checks(args, texts, log):
    ok = True
    ip = re.search(r'PERS string stTG_ServerIP:="([^"]*)"', texts["TG_Comms.sys"])
    ip = ip.group(1) if ip else "?"
    if ip != args.server_ip:
        log.failed("site", "TG_Comms.sys stTG_ServerIP is %s, expected %s (--server-ip)" % (ip, args.server_ip))
        ok = False
    else:
        log.passed("site", "stTG_ServerIP = %s" % ip)
    main_text = texts["TG_Main.mod"]
    if re.search(r"^\s*PROC\s+main\s*\(", main_text, re.M | re.I):
        log.failed("site", "TG_Main.mod still declares PROC main() - Production Manager owns main; "
                           "rename it tgs_main")
        ok = False
    elif not re.search(r"^\s*PROC\s+tgs_main\s*\(", main_text, re.M):
        log.failed("site", "TG_Main.mod has no PROC tgs_main()")
        ok = False
    else:
        log.passed("site", "TG_Main.mod: tgs_main(), no main()")
    tools = pers_decls(texts["TG_CellCal.sys"])
    for name, placeholder in PLACEHOLDER_TOOLS.items():
        value = tools.get(name, (None, ""))[1]
        if same_value(value, placeholder):
            log.failed("site", "%s in TG_CellCal.sys is still the placeholder %s - put the cell's "
                               "calibrated TCP in abb/rapid/TG_CellCal.sys" % (name, placeholder))
            ok = False
        else:
            log.passed("site", "%s = %s" % (name, value))
    live_setdo = [ln for ln in texts["TG_Cell.sys"].splitlines()
                  if "SetDO doTG_Camera" in ln and not ln.lstrip().startswith("!")]
    if live_setdo and not args.camera_signal:
        log.failed("site", "TG_Cell.sys still sets doTG_Camera, which the real cell does not have "
                           "(comment both SetDO lines out, or pass --camera-signal if it now exists)")
        ok = False
    else:
        log.passed("site", "TG_Cell.sys camera SetDO %s" % ("active (--camera-signal)" if live_setdo
                                                            else "commented out"))
    return ok


def require_auto_stopped(rws, log, step):
    ident = identity(rws)
    if ident["opmode"] != "AUTO":
        log.failed(step, "opmode is %s: in MANUAL the FlexPendant holds RAPID mastership - "
                         "do this step on the pendant (README)" % ident["opmode"])
        return False
    return cp.require_stopped(rws, log, step)


def cmd_unload(args, rws, log):
    if not require_auto_stopped(rws, log, "unload"):
        return False
    loaded = cp.module_names(rws, args.task)
    todo = [m for m in reversed(ALL_MODULES) if m in loaded]
    if not todo:
        log.passed("unload", "no TG module loaded - nothing to do")
        return True
    mark = cp.elog_mark(rws)
    errors = []

    def go():
        for m in todo:
            try:
                rws.post_form("/rw/rapid/tasks/" + args.task, {"module": m}, {"action": "unloadmod"})
                log.detail("unloadmod %s accepted" % m)
            except cp.RwsError as exc:
                errors.append("%s: %s" % (m, exc.explain()))
    try:
        cp.with_mastership(rws, go)
    except cp.RwsError as exc:
        log.failed("unload", cp.mastership_explain(exc))
        return False
    time.sleep(1.0)
    left = [m for m in ALL_MODULES if m in cp.module_names(rws, args.task)]
    for e in cp.elog_after(rws, mark):
        log.detail("elog %d [%s] %s | %s" % e)
    if left:
        log.failed("unload", "still loaded: %s %s" % (", ".join(left), "; ".join(errors)))
        return False
    log.passed("unload", "unloaded %s" % ", ".join(todo))
    return True


def cmd_delete(args, rws, log):
    bdir = latest(args.log_dir, "backup")
    if not bdir:
        log.failed("delete", "no backup folder in %s - run 'backup' first" % args.log_dir)
        return False
    listing = ctrl_listing(rws) or {}
    loaded = cp.module_names(rws, args.task)
    still = [m for m in ALL_MODULES if m in loaded]
    if still:
        log.warn("delete", "modules still loaded (the program keeps running from memory, but unload "
                           "them before loading the new set): %s" % ", ".join(still))
    deleted, kept = [], []
    for f in ALL_FILES:
        if f not in listing:
            continue
        data = rws.get_file(CTRL_DIR + "/" + f)
        local = os.path.join(bdir, f)
        if not os.path.isfile(local) or read_bytes(local) != data:
            kept.append(f)
            log.warn("delete", "%s not deleted: the backup in %s does not hold this exact file" % (f, bdir))
            continue
        rws.delete_file(CTRL_DIR + "/" + f)
        deleted.append(f)
    left = [f for f in ALL_FILES if f in (ctrl_listing(rws) or {})]
    gone_wrong = [f for f in deleted if f in left]
    if gone_wrong:
        log.failed("delete", "still present after DELETE: %s" % ", ".join(gone_wrong))
        return False
    log.passed("delete", "deleted from HOME:/TGS: %s%s (backup %s)"
               % (", ".join(deleted) or "nothing (no old TG file there)",
                  ("; kept: " + ", ".join(kept)) if kept else "", bdir))
    return not kept


def cmd_upload(args, rws, log):
    sdir = args.stage or latest(args.log_dir, "stage")
    if not sdir or not os.path.isfile(os.path.join(sdir, "manifest.json")):
        log.failed("upload", "no stage with a manifest - run 'stage' first")
        return False
    with open(os.path.join(sdir, "manifest.json"), encoding="utf-8") as fh:
        manifest = json.load(fh)
    log.info("upload", "stage %s (built %s, %s, %d carried)"
             % (sdir, manifest["built"], manifest["git_head"], len(manifest["carried"])))
    cp.ensure_ctrl_dir(rws, log, "upload")          # creates HOME:/TGS when missing
    bad = []
    for e in manifest["files"]:
        data = read_bytes(os.path.join(sdir, e["name"]))
        if sha(data) != e["sha256_16"]:
            log.failed("upload", "%s changed after staging - re-run 'stage'" % e["name"])
            return False
        rws.put_file(CTRL_DIR + "/" + e["name"], data)
        back = rws.get_file(CTRL_DIR + "/" + e["name"])
        same = back == data
        log.say("  PUT HOME:/TGS/%-16s %6d bytes  read-back %s" % (e["name"], len(data),
                                                                    "identical" if same else "DIFFERS"))
        if not same:
            bad.append(e["name"])
    if bad:
        log.failed("upload", "read-back differs: %s" % ", ".join(bad))
        return False
    log.passed("upload", "%d files on the controller, byte-identical to the stage" % len(manifest["files"]))
    return True


def cmd_load(args, rws, log):
    if not require_auto_stopped(rws, log, "load"):
        return False
    loaded = cp.module_names(rws, args.task)
    present = [m for m in ALL_MODULES if m in loaded]
    if present:
        log.failed("load", "still loaded: %s - run 'unload' first (loading over them would mix "
                           "builds)" % ", ".join(present))
        return False
    listing = ctrl_listing(rws) or {}
    missing = [f for f in SET if f not in listing]
    if missing:
        log.failed("load", "not in HOME:/TGS: %s - run 'upload' first" % ", ".join(missing))
        return False
    mark = cp.elog_mark(rws)
    errors = []

    def go():
        for f in SET:
            try:
                rws.post_form("/rw/rapid/tasks/" + args.task, {"modulepath": CTRL_DIR + "/" + f},
                              {"action": "loadmod"})
                log.detail("loadmod %s accepted (not a verdict)" % f)
            except cp.RwsError as exc:
                errors.append("%s: %s" % (f, exc.explain()))
    try:
        cp.with_mastership(rws, go)
    except cp.RwsError as exc:
        log.failed("load", cp.mastership_explain(exc))
        return False
    time.sleep(2.0)
    events = cp.elog_after(rws, mark)
    for e in events:
        log.detail("elog %d [%s] %s | %s" % e)
    fatal = [e for e in events if e[1] in FATAL_ELOG]
    links = [e for e in events if e[1] == LINK_ELOG]
    now = cp.module_names(rws, args.task)
    absent = [mod_name(f) for f in SET if mod_name(f) not in now]
    if fatal or absent or errors:
        log.failed("load", "; ".join(["[%s] %s %s" % (e[1], e[2], e[3][:120]) for e in fatal]
                                     + ["not loaded: " + ", ".join(absent)] * bool(absent) + errors))
        return False
    log.passed("load", "7 modules loaded in %s, no fatal event (%d 40160 while loading; "
                       "the build check below is the verdict)" % (args.task, len(links)))
    return report_build(rws, args.task, log, "program") is not False


# ============================================================ CLI

def build_parser():
    p = argparse.ArgumentParser(prog="tg_deploy.py", description=__doc__.split("\n")[0],
                                epilog="Runbook: README.md, 'Replacing the TG modules'.",
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ip", help="controller IP (MONARC WAN port: 10.8.8.56); not needed for 'stage'")
    p.add_argument("--user", default=cp.DEFAULT_USER)
    p.add_argument("--password", default=cp.DEFAULT_PASSWORD)
    p.add_argument("--rws-port", type=int, default=80)
    p.add_argument("--task", default="T_ROB1")
    p.add_argument("--timeout", type=float, default=15.0)
    p.add_argument("--log-dir", default=os.path.join(HERE, "logs"))
    p.add_argument("-v", "--verbose", action="store_true")
    sub = p.add_subparsers(dest="cmd", required=True, metavar="COMMAND")
    s = sub.add_parser("status", help="read-only: what is on the controller vs the stage")
    s.add_argument("--expect-system", default="4600-804589",
                   help="warn when the controller's system name differs (default MONARC real cell)")
    sub.add_parser("backup", help="download HOME:/TGS + PERS snapshot")
    s = sub.add_parser("stage", help="build the upload set locally (no controller)")
    s.add_argument("--server-ip", default="10.8.8.56", help="stTG_ServerIP the set must carry")
    s.add_argument("--expect-system", default="4600-804589",
                   help="the latest snapshot must come from this controller (default MONARC real cell)")
    s.add_argument("--snapshot", help="PERS snapshot to carry from (default: the latest)")
    s.add_argument("--no-carry", action="store_true", help="carry no live value")
    s.add_argument("--prefer-file", metavar="NAMES",
                   help="comma list of PERS where the repo value wins a conflict with the controller")
    s.add_argument("--camera-signal", action="store_true",
                   help="allow active SetDO doTG_Camera (only once the cell has that signal)")
    sub.add_parser("unload", help="unloadmod the TG modules (AUTO, RAPID stopped)")
    sub.add_parser("delete", help="delete the old TG files from HOME:/TGS (backed-up ones only)")
    s = sub.add_parser("upload", help="PUT the stage to HOME:/TGS with read-back")
    s.add_argument("--stage", help="stage folder (default: the latest)")
    sub.add_parser("load", help="loadmod the set in order (AUTO, RAPID stopped)")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    log = cp.Log(args.log_dir, args.verbose)
    log.say("tg_deploy '%s' | controller %s | task %s | %s"
            % (args.cmd, args.ip or "-", args.task, _dt.datetime.now().isoformat(timespec="seconds")))
    try:
        if args.cmd == "stage":
            cmd_stage(args, log)
        else:
            if not args.ip:
                log.failed(args.cmd, "--ip is required")
                return log.summary()
            rws = cp.Rws(args.ip, args.rws_port, args.user, args.password, args.timeout)
            {"status": cmd_status, "backup": cmd_backup, "unload": cmd_unload, "delete": cmd_delete,
             "upload": cmd_upload, "load": cmd_load}[args.cmd](args, rws, log)
    except KeyboardInterrupt:
        log.say("interrupted by operator")
    except cp.RwsError as exc:
        log.failed(args.cmd, exc.explain())
    return log.summary()


if __name__ == "__main__":
    sys.exit(main())
