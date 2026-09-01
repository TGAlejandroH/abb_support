"""Retrieve an operator-edited .tgs program from the controller (Phase 5).

Prototype stand-in for the HMI's Tools-menu "Retrieve robot program from
controller" (TGuideWeldingHMI GUIMainWindow.cpp:325 on FANUC). The ABB flow
differs from FANUC because a pendant touch-up lives in program memory, not in
the file (docs/abb_program_touchup_and_retrieval_v1.md section 2.1): TG_Main
detects the edit at unload (``UnLoad \\ErrIfChanged``) and STAGES the edited
module in ``HOME:/TGS/edited/<prog>.mod``. This tool then:

  1. fetches the staged file (RWS fileservice, or a direct read of the
     virtual controller's HOME folder - the same fallback pair as sending),
  2. validates it (MODULE <prog>_Mod header + a PROC <prog> - the same kind
     of fingerprint the FANUC HMI applies, FTPManager::IsFileATGProject),
  3. backs up the current master to <dest>/retrieved_backups/<prog>_<ts>.mod
     (mirroring the HMI's timestamped ROBOT_PROGRAM/ backups),
  4. adopts the retrieved bytes as the new master <dest>/<prog>.mod, and
  5. cleans up: deletes the staged file and clears PERS nTG_ProgEdited
     (RWS only; both best-effort).

If nothing is staged there is nothing to retrieve - the controller-side file
HOME:/TGS/<prog>.mod is by definition the copy the HMI itself sent.

Usage:
    python tg_retrieve.py <prog_name> --rws http://127.0.0.1:80 [options]
    python tg_retrieve.py <prog_name> --vc-home <path to VC HOME> [options]

Options: --dest DIR (default: this repo's abb/rapid/TGS), --no-cleanup,
--user/--password (RWS credentials, default "Default User"/"robotics").

Exit codes: 0 = program retrieved (or nothing staged), 2 = retrieval failed
(transport or validation error; the master is never touched on failure).
"""

import argparse
import os
import re
import sys
import time

from rws_client import RwsClient, RwsError, RwsFileNotFoundError

STAGED_DIR_RWS = "$home/TGS/edited"
EDITED_FLAG_SYMBOL = "nTG_ProgEdited"


class RetrieveError(Exception):
    """Retrieval failed; the project master was not modified."""


# ---------------------------------------------------------------------------
# Sources: where the staged module comes from and how to clean it up
# ---------------------------------------------------------------------------

class RwsSource:
    """Fetch/clean up through Robot Web Services (real cell and VC)."""

    def __init__(self, client):
        self.client = client

    def describe(self):
        return f"RWS at {self.client.base_url}"

    def fetch_staged(self, prog_name):
        try:
            return self.client.get_file(f"{STAGED_DIR_RWS}/{prog_name}.mod")
        except RwsFileNotFoundError:
            return None
        except RwsError as exc:
            raise RetrieveError(str(exc)) from exc

    def cleanup(self, prog_name, log):
        try:
            self.client.delete_file(f"{STAGED_DIR_RWS}/{prog_name}.mod")
        except RwsError as exc:
            log(f"WARNING: staged file not deleted ({exc}); the next retrieve "
                f"would fetch it again")
        try:
            self.client.set_symbol(EDITED_FLAG_SYMBOL, 0)
        except RwsError as exc:
            log(f"WARNING: {EDITED_FLAG_SYMBOL} not cleared ({exc}); clear it "
                f"from the FlexPendant/RobotStudio data view")


class VcHomeSource:
    """Fetch/clean up by reading the virtual controller's HOME folder
    directly - the retrieval twin of abb_server's copy-based transfer
    fallback (kept per the Phase 5 decision; the VC's HOME: is a plain
    Windows folder)."""

    def __init__(self, home_dir):
        self.home_dir = home_dir

    def describe(self):
        return f"VC HOME folder {self.home_dir}"

    def _staged_path(self, prog_name):
        return os.path.join(self.home_dir, "TGS", "edited", f"{prog_name}.mod")

    def fetch_staged(self, prog_name):
        path = self._staged_path(prog_name)
        if not os.path.isfile(path):
            return None
        try:
            with open(path, "rb") as f:
                return f.read()
        except OSError as exc:
            raise RetrieveError(f"cannot read staged file {path}: {exc}") from exc

    def cleanup(self, prog_name, log):
        try:
            os.remove(self._staged_path(prog_name))
        except OSError as exc:
            log(f"WARNING: staged file not deleted ({exc})")
        log(f"NOTE: PERS {EDITED_FLAG_SYMBOL} stays set (no RWS in --vc-home "
            f"mode) - reset it in the RobotStudio data view")


# ---------------------------------------------------------------------------
# Validation and adoption
# ---------------------------------------------------------------------------

def validate_tgs_module(data, prog_name):
    """The naming-contract fingerprint (touch-up doc section 6.3): file =
    PROC = <prog_name>, MODULE = <prog_name>_Mod. Raises RetrieveError."""
    if not data or not data.strip():
        raise RetrieveError("retrieved file is empty")
    text = data.decode("utf-8", errors="replace")
    module_re = re.compile(
        r"^\s*MODULE\s+%s_Mod\s*($|\()" % re.escape(prog_name),
        re.IGNORECASE)
    proc_re = re.compile(
        r"\bPROC\s+%s\s*\(" % re.escape(prog_name), re.IGNORECASE)
    first_code_line = None
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("!"):
            continue
        first_code_line = stripped
        break
    if first_code_line is None or not module_re.match(first_code_line):
        raise RetrieveError(
            f"not a {prog_name} module: expected 'MODULE {prog_name}_Mod' as "
            f"the first code line, got {first_code_line!r}")
    if not proc_re.search(text):
        raise RetrieveError(
            f"module has no 'PROC {prog_name}(' - TG_Main's late-bound call "
            f"%{prog_name}% would fail (ERR_REFUNKPRC)")


def retrieve_program(prog_name, source, dest_dir, cleanup=True, log=print):
    """Fetch + validate + backup + adopt. Returns the master path, or None
    when nothing is staged. Raises RetrieveError on any failure - the master
    file is only replaced after successful validation."""
    log(f"retrieving {prog_name} from {source.describe()}")
    data = source.fetch_staged(prog_name)
    if data is None:
        log(f"no edited program staged for {prog_name} - nothing to retrieve "
            f"(the robot only stages a file when a touch-up was detected)")
        return None
    validate_tgs_module(data, prog_name)

    master_path = os.path.join(dest_dir, f"{prog_name}.mod")
    if os.path.isfile(master_path):
        backup_dir = os.path.join(dest_dir, "retrieved_backups")
        os.makedirs(backup_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        backup_path = os.path.join(backup_dir, f"{prog_name}_{stamp}.mod")
        with open(master_path, "rb") as f:
            previous = f.read()
        with open(backup_path, "wb") as f:
            f.write(previous)
        log(f"backed up current master to {backup_path}")

    os.makedirs(dest_dir, exist_ok=True)
    with open(master_path, "wb") as f:
        f.write(data)
    log(f"adopted retrieved program as master: {master_path}")

    if cleanup:
        source.cleanup(prog_name, log)
    return master_path


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _default_dest():
    return os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "abb", "rapid", "TGS"))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Retrieve an operator-edited .tgs program (Phase 5).")
    parser.add_argument("prog_name", help=".tgs program name, e.g. TD05Test")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--rws", metavar="BASE_URL",
                       help="controller RWS base URL, e.g. http://127.0.0.1:80")
    group.add_argument("--vc-home", metavar="DIR",
                       help="virtual controller HOME folder (direct file read)")
    parser.add_argument("--dest", default=_default_dest(),
                        help="master program directory (default: abb/rapid/TGS)")
    parser.add_argument("--no-cleanup", action="store_true",
                        help="keep the staged file and the nTG_ProgEdited flag")
    parser.add_argument("--user", default=None, help="RWS username")
    parser.add_argument("--password", default=None, help="RWS password")
    args = parser.parse_args(argv)

    if args.rws:
        client_kwargs = {}
        if args.user is not None:
            client_kwargs["username"] = args.user
        if args.password is not None:
            client_kwargs["password"] = args.password
        source = RwsSource(RwsClient(args.rws, **client_kwargs))
    else:
        source = VcHomeSource(args.vc_home)

    try:
        retrieve_program(args.prog_name, source, args.dest,
                         cleanup=not args.no_cleanup)
    except RetrieveError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
