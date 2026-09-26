"""Offline sanity check for RAPID source, to be run BEFORE pushing it to a controller.

Why this exists: on 2026-09-20 an edit to `TG_Comms.sys` added `⚠` to a comment. RAPID source
must be ASCII, so the module failed to load -- and because it is a SysMod that `TG_Weld` and
every `.tgs` program depend on, the failed load erased it and put the task into *system failure
state*. Recovering cost a controller restart. A three-line check would have caught it.

This is NOT a RAPID compiler. It catches the classes of defect that are cheap to detect
textually and expensive to discover on a live controller:

* **non-ASCII bytes** -- the one above;
* **unbalanced block keywords** (MODULE/PROC/FUNC/TRAP/RECORD/IF/FOR/WHILE/TEST), which is
  what a stray edit to a long routine produces;
* **module-name / routine-name collision** -- finding **F-1** in
  `docs/rapid_validation_findings_v1.md`: RAPID module names and global routine names share
  one namespace, so `MODULE TD05Test` containing `PROC TD05Test` is "Name error(45): Module
  name ambiguous" at load;
* **unterminated string literals**, which silently swallow the rest of a line;
* **tab characters**, which some RobotWare editors reject;
* **a component of a function result** (``CJointT().extax``) -- a RAPID syntax error
  (40322 at load, VC-found 2026-09-26 in TD05Touch.mod); copy the result to a variable first;
* **the same global name declared in two of the checked modules** -- finding **F-5**: a
  global PERS (or any global symbol) declared by two modules of one task is a semantic
  error (40160) that blocks PP-to-main for the WHOLE task, and the controller never names
  the symbol. Checked across all files given on one command line.

A clean run is not proof the module loads -- only the controller can say that. A dirty run is
proof it will not.

Usage:
    python tools/rapid_check.py abb/rapid/*.sys abb/rapid/TGS/*.mod
    python tools/rapid_check.py --all        # every .sys/.mod under abb/
Exit code 0 = clean, 1 = findings.
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import sys

#: Openers and the keyword that closes them. `IF` is special-cased: a single-line
#: `IF x=1 GOTO y;` has no ENDIF, only a multi-line `IF ... THEN` does.
BLOCK_PAIRS = {
    "MODULE": "ENDMODULE",
    "PROC": "ENDPROC",
    "FUNC": "ENDFUNC",
    "TRAP": "ENDTRAP",
    "RECORD": "ENDRECORD",
    "FOR": "ENDFOR",
    "WHILE": "ENDWHILE",
    "TEST": "ENDTEST",
}
CLOSERS = {v: k for k, v in BLOCK_PAIRS.items()}

_WORD = re.compile(r"\b([A-Z]+)\b")
#: `Func(...).component` - RAPID has no member access on a call result.
_CALL_COMPONENT = re.compile(r"\b[A-Za-z_]\w*\([^()]*\)\.[A-Za-z_]\w*")
_MODULE_DECL = re.compile(r"^\s*MODULE\s+([A-Za-z_]\w*)", re.IGNORECASE)
_ROUTINE_DECL = re.compile(
    r"^\s*(?:LOCAL\s+)?(?:PROC|FUNC\s+\w+|TRAP)\s+([A-Za-z_]\w*)", re.IGNORECASE
)


def _scan_line(line: str) -> tuple[str, bool]:
    """Split one line into (executable part, string_is_open).

    Character-wise, because the two constructs nest and the order matters in BOTH directions:
    a `!` INSIDE a string does not start a comment (`TPWrite "hi! there"`), and a `"` inside a
    comment does not open a string. Stripping comments first with a regex gets the first case
    wrong and reports a false unterminated-string on a perfectly good line -- which the
    checker's own test caught.

    The returned code has string BODIES blanked (so an "IF" in a message cannot move the block
    balance) and the comment removed.
    """
    out = []
    in_string = False
    index = 0
    while index < len(line):
        char = line[index]
        if in_string:
            if char == '"':
                # RAPID escapes a literal quote by doubling it.
                if index + 1 < len(line) and line[index + 1] == '"':
                    index += 2
                    continue
                in_string = False
                out.append('"')
            # string body contributes nothing
        elif char == '"':
            in_string = True
            out.append('"')
        elif char == "!":
            break  # comment to end of line
        else:
            out.append(char)
        index += 1
    return "".join(out), in_string


def check_text(text: str, name: str = "<text>") -> list[str]:
    """Return a list of human-readable findings; empty means clean."""
    findings: list[str] = []
    lines = text.splitlines()

    # --- non-ASCII (the 2026-09-20 outage) --------------------------------------
    for number, line in enumerate(lines, start=1):
        bad = [(i, ch) for i, ch in enumerate(line) if ord(ch) > 127]
        if bad:
            shown = ", ".join(f"{ch!r} (U+{ord(ch):04X}) at col {i + 1}" for i, ch in bad[:4])
            findings.append(
                f"{name}:{number}: non-ASCII character(s) -- RAPID source must be ASCII: {shown}"
            )

    for number, line in enumerate(lines, start=1):
        if "\t" in line:
            findings.append(f"{name}:{number}: tab character -- use spaces")

    # --- component of a function result: CJointT().extax --------------------------
    for number, raw in enumerate(lines, start=1):
        code = _scan_line(raw)[0]
        match = _CALL_COMPONENT.search(code)
        if match:
            findings.append(
                f"{name}:{number}: {match.group(0)!r} takes a component of a function result -- "
                "a RAPID syntax error (40322 at load); copy the result into a variable first"
            )

    # --- unterminated string literal --------------------------------------------
    for number, line in enumerate(lines, start=1):
        if _scan_line(line)[1]:
            findings.append(f"{name}:{number}: unterminated string literal")

    # --- block balance -----------------------------------------------------------
    stack: list[tuple[str, int]] = []
    module_name = ""
    routines: list[tuple[str, int]] = []
    for number, raw in enumerate(lines, start=1):
        line = _scan_line(raw)[0]
        if not line.strip():
            continue

        match = _MODULE_DECL.match(raw)
        if match:
            module_name = match.group(1)
        match = _ROUTINE_DECL.match(raw)
        if match:
            routines.append((match.group(1), number))

        upper = line.upper()
        for word in _WORD.findall(upper):
            if word in CLOSERS:
                opener = CLOSERS[word]
                if not stack:
                    findings.append(f"{name}:{number}: {word} with no matching {opener}")
                elif stack[-1][0] != opener:
                    findings.append(
                        f"{name}:{number}: {word} closes {opener}, but the open block is "
                        f"{stack[-1][0]} from line {stack[-1][1]}"
                    )
                    stack.pop()
                else:
                    stack.pop()
            elif word == "ENDIF":
                if stack and stack[-1][0] == "IF":
                    stack.pop()
                else:
                    findings.append(f"{name}:{number}: ENDIF with no matching IF ... THEN")
            elif word == "IF":
                # Only a multi-line IF (one ending in THEN) needs an ENDIF; a one-line
                # `IF nTG_WeldStatus=2 GOTO abort_end;` does not, and counting it would
                # report a false imbalance on every guard clause in the file.
                if upper.rstrip().endswith("THEN"):
                    stack.append(("IF", number))
            elif word in BLOCK_PAIRS:
                # `FUNC num tgClampCorr(...)` -- the opener is the keyword, not the type.
                stack.append((word, number))

    for opener, number in stack:
        findings.append(f"{name}:{number}: {opener} is never closed by {BLOCK_PAIRS.get(opener, 'ENDIF')}")

    # --- F-1: module name vs routine name ---------------------------------------
    for routine, number in routines:
        if module_name and routine.lower() == module_name.lower():
            findings.append(
                f"{name}:{number}: routine {routine!r} has the same name as its MODULE -- "
                "RAPID shares one namespace for both, so this is 'Name error(45): Module name "
                "ambiguous' at load (finding F-1)"
            )

    return findings


_GLOBAL_DATA = re.compile(r"^\s*(PERS|VAR|CONST)\s+\w+\s+([A-Za-z_]\w*)", re.IGNORECASE)
_ROUTINE_OPEN = re.compile(r"^\s*(?:LOCAL\s+)?(PROC|FUNC|TRAP|RECORD)\b", re.IGNORECASE)
_ROUTINE_CLOSE = re.compile(r"^\s*(ENDPROC|ENDFUNC|ENDTRAP|ENDRECORD)\b", re.IGNORECASE)


def global_symbols(text: str) -> list[tuple[str, int]]:
    """(name, line) of every module-level symbol WITHOUT the LOCAL attribute.

    Data declared inside a routine is routine-local, whatever its keyword, so a routine body
    is skipped. RECORD names count (they are global types)."""
    found: list[tuple[str, int]] = []
    depth = 0
    for number, raw in enumerate(text.splitlines(), start=1):
        line = _scan_line(raw)[0]
        if not line.strip():
            continue
        if _ROUTINE_CLOSE.match(line):
            depth = max(0, depth - 1)
            continue
        opened = _ROUTINE_OPEN.match(line)
        if opened:
            if depth == 0 and not line.lstrip().upper().startswith("LOCAL"):
                match = _ROUTINE_DECL.match(raw)
                if match:
                    found.append((match.group(1), number))
                elif opened.group(1).upper() == "RECORD":
                    name = line.split()[1] if len(line.split()) > 1 else ""
                    found.append((name, number))
            depth += 1
            continue
        if depth == 0:
            match = _GLOBAL_DATA.match(line)
            if match:
                found.append((match.group(2), number))
    return found


def check_duplicate_globals(named_texts: list[tuple[str, str]]) -> list[str]:
    """F-5 across modules: one finding per global name declared in more than one file.

    RAPID identifiers are case-insensitive, so the comparison is too."""
    seen: dict[str, list[tuple[str, int, str]]] = {}
    for name, text in named_texts:
        for symbol, number in global_symbols(text):
            seen.setdefault(symbol.lower(), []).append((name, number, symbol))
    findings = []
    for places in seen.values():
        files = {place[0] for place in places}
        if len(files) > 1:
            where = ", ".join(f"{f}:{n}" for f, n, _ in places)
            findings.append(
                f"global {places[0][2]!r} is declared in {len(files)} modules ({where}) -- two "
                "global declarations of one name in a task are a semantic error that blocks "
                "PP-to-main for the whole task (finding F-5); make all but one LOCAL or rename"
            )
    return findings


def _read_text(path: str) -> str:
    with open(path, "rb") as handle:
        return handle.read().decode("utf-8", errors="replace")


def check_file(path: str) -> list[str]:
    with open(path, "rb") as handle:
        raw = handle.read()
    try:
        text = raw.decode("ascii")
    except UnicodeDecodeError:
        # Decode leniently so the per-line reporter can point at the offending column
        # instead of the whole file failing with one opaque message.
        text = raw.decode("utf-8", errors="replace")
    return check_text(text, os.path.basename(path))


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("paths", nargs="*", help=".sys / .mod files to check")
    parser.add_argument("--all", action="store_true", help="check every .sys/.mod under abb/")
    args = parser.parse_args(argv[1:])

    paths = list(args.paths)
    if args.all or not paths:
        root = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "abb")
        paths = sorted(
            glob.glob(os.path.join(root, "**", "*.sys"), recursive=True)
            + glob.glob(os.path.join(root, "**", "*.mod"), recursive=True)
        )

    total = 0
    for path in paths:
        findings = check_file(path)
        total += len(findings)
        status = "OK  " if not findings else "FAIL"
        print(f"{status} {path}")
        for finding in findings:
            print(f"       {finding}")
    if len(paths) > 1:
        cross = check_duplicate_globals([(os.path.basename(p), _read_text(p)) for p in paths])
        total += len(cross)
        print(f"{'OK  ' if not cross else 'FAIL'} cross-module globals (F-5), {len(paths)} files")
        for finding in cross:
            print(f"       {finding}")
    print(f"\n{len(paths)} file(s), {total} finding(s)")
    return 1 if total else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
