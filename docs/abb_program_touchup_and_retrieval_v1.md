# Operator touch-ups and program retrieval — ABB vs FANUC — v1 (PLAN, no code yet)

Status: **research complete 2026-08-31, nothing implemented.** Opened because the
`Load \Dynamic` lifecycle in
[abb_weld_motion_and_data_design_v1.md](abb_weld_motion_and_data_design_v1.md) §3.1
collides with a production workflow that repo never modelled: *operators touch up
positions on the teach pendant, then the HMI pulls the program back and adopts it as
the new master.*

Two corrections to existing docs came out of this (§7).

Sources: RW6 manuals read directly (Appendix A quotes them verbatim), the HMI repo
`TGuideWeldingHMI` at commit `18aa6bf` (every FANUC claim below is a file:line in it),
the RWS reference on ABB Developer Center, and `ros-industrial/abb_librws` as an
implementation cross-check for the RWS URIs. ⚠ items are for the VC loop (§8).

---

## 1. What the FANUC side does today (verified in the HMI repo)

| Step | Where |
|---|---|
| Tools menu → **"Retrieve robot program from controller"** | `GUIMainWindow.cpp:325` (QAction), wired at `:1182` |
| → `WeldingProject::retrieveLastSentProgram()` | `WeldingProject.cpp:2755` |
| FTP GET `ftp://<robot-ip>/md:\/<progname>.LS` | `FTPManager.cpp:203` `RetrieveFileFromMDFile`; URL base at `FTPManager.cpp:20` |
| Confirm dialog, timestamped backup of the old program, then replace inside the project archive under `ROBOT_PROGRAM/` | `WeldingProject.cpp:381` `ReplaceTgsRobotProgramWithConfirmation` (v2 twin at `:423`) |
| Next run: byte-compare project copy vs `MD:` copy; **skip the transfer if equal**, else push | `RobotCell.cpp:1853` → `WeldingProject.cpp:2848` `CompareCurrentWithProgramInTheController` |

So the description is accurate, and one detail matters more than it looks: the send is
**conditional on a byte-exact compare**, not unconditional. That is what makes the
workflow coherent — after a retrieve, both sides hold *the controller's own
serialization* of the program, the compare matches, and the touch-up survives the next
run. Without the retrieve the compare differs and the HMI re-pushes its copy, wiping the
touch-up. The compare being byte-exact is also the sharpest ABB porting risk (§6.2).

Program identity: `GetRobotProgramName()` is the project password
(`WeldingProject.cpp:2812`), extension `.LS` hardcoded (`:2817`). A brand switch already
exists — `ROBOT_BRAND == "FANUC"` at ~12 sites in `RobotCell.cpp` — so there is a seam for
ABB, but `.LS` and `md:` are baked into the FTP layer.

---

## 2. Why this does not port 1:1

### 2.1 On FANUC the memory *is* the file; on ABB it is not

`MD:` is FANUC's memory device — programs living in controller memory are addressed as
files, and `.LS` is the ASCII rendering of the in-memory `.TP` (which is why an `.LS`
dropped into `MD:` is translated on arrival). A pendant touch-up changes the in-memory
program, so an FTP GET of `MD:<prog>.LS` returns the touched-up program **with no save
step**.

ABB has no such device. `HOME:/TGS/<name>.mod` is an ordinary file on the controller
disk, and a pendant touch-up modifies the copy in **program memory** only:

> "A loaded program is automatically saved in the program memory, but saving to the
> controller hard disk is an extra precaution." — Operating manual 3HAC050941 §5.3.1

`ModPos` rewrites the position's *declaration* in memory (hence it works on `CONST
robtarget`, which is what our `.tgs` targets are — `TGS/TD05Test.mod:31`), and PERS
current values are only folded back into their declarations at save time (Appendix A.2).
Nothing writes any of it to disk on its own; there is no auto-save system parameter
(`ModPos Settings` only bounds how far a position may move — 3HAC050948 §3.9).

**Consequence: the ABB retrieve is two steps — save the module to a file, then fetch the
file.** A one-step "GET the program" does not exist.

And because `TG_Main` unloads the module at the end of every cycle
([TG_Main.mod:108](../abb/rapid/TG_Main.mod#L108), a bare `UnLoad`), the window in which
the touch-up still exists closes at the end of that cycle — or instantly if the operator
moves PP to main, since `\Dynamic` modules unload on that (Appendix A.4).

### 2.2 An IRC5 has no FTP server to pull from

The port plan's decision — "plan on purchasing/enabling the FTP option"
([abb_port_plan_v1.md](abb_port_plan_v1.md) item 13, repeated at §7 item 3) — does not do
what we assumed. The option is **FTP & SFTP Client [614-1]**, and it is a *client*:

> "The option makes it possible to read information from a remote computer, directly from
> the controller. Once the application protocol is configured, the remote computer can be
> accessed in the same way as the controller's internal hard disk."
> — Product specification 3HAC050945 §10.1

It gives the controller access to a share on *our* PC. It does not let the HMI pull files
off the controller. It also is **not in the cell's installed option list** (`616-1`,
`841-1`, `604-1`, `623-1`, `633-4 Arc`, Fronius TPS, `657-1`, EtherNet/IP —
[abb_weld_motion_and_data_design_v1.md](abb_weld_motion_and_data_design_v1.md) §2.4).
⚠ Confirm against the real cell's key before quoting anything.

**Robot Web Services is the answer, and it costs nothing.** RWS is documented in chapter
3 **RobotWare-OS** of the same product specification — base functionality, not a purchased
option — with client-side requirements only ("Knowledge of HTTP… a programming library
which can initiate HTTP requests"). It is REST over HTTP with digest auth, and the HMI
already links libcurl, which speaks both. Two transports (614-1 one way, RWS the other)
would be strictly worse than one.

---

## 3. Three ways to trigger the save. Pick one.

| # | Trigger | Pros | Cons |
|---|---|---|---|
| **A** | **RAPID does it**: `UnLoad \Save` (or `Save` then `UnLoad`) in `tgRunTgsProgram` | No mastership dance, no mode restriction, always saves the version that just ran. Cheapest change | Saves on *every* cycle even when nothing changed (rewrites the file, extra churn); "saved" and "the operator wanted it kept" get conflated |
| **B** | **HMI does it over RWS**: `POST /rw/rapid/modules/<mod>?action=save` with `path`/`name`, then `GET /fileservice/...` | Save happens only when the operator presses Retrieve — exact parity with the FANUC button. One transport for everything | Needs RAPID **mastership**; the FlexPendant holds mastership in manual mode, which is exactly the mode the operator just used. ⚠ Highest-risk unknown (§8 T2) |
| **C** | **Operator does it**: pendant *Program Editor → Modules → File → Save Module As…* into `HOME:/TGS/`, then presses Retrieve on the HMI | Zero controller-side code; documented pendant function (3HAC050941 §5.3.2) | Extra operator step, and "Save Module As" makes them choose the path — save to the wrong folder and the retrieve silently returns the stale file |

**Recommendation: A + B, with `\ErrIfChanged` as the gate.** Replace the bare `UnLoad`
with `UnLoad \ErrIfChanged`; on `ERR_NOTSAVED` (module modified since it was loaded)
`Save` it to a retrieval staging path and raise a flag the HMI can see. That way:

- nothing is written on a normal cycle (no churn, no spurious re-sends),
- a touch-up is **never** silently lost — the controller parks the edited module on disk
  before unloading it,
- the HMI's Retrieve button becomes a plain `GET /fileservice/...` with no mastership
  needed, and can even prompt on its own ("the program on the robot was modified"),
- option C keeps working as the manual fallback.

C is also the honest zero-code answer if the workflow is needed before any RAPID change
ships.

---

## 4. Proposed ABB flow

```
operator jogs + Modify Position on the pendant   (manual mode, program stopped)
        │
        │  edits live in PROGRAM MEMORY only
        ▼
end of cycle: TG_Main  UnLoad \ErrIfChanged  ──► ERR_NOTSAVED
        │
        ├─ Save "<name>_Mod" \FilePath:="HOME:/TGS/edited/<name>.mod"
        ├─ set PERS nTG_ProgEdited := 1        (HMI-visible)
        └─ UnLoad (plain) — module leaves memory, file on disk survives
        │
        ▼
HMI "Retrieve robot program from controller"
        GET /fileservice/$HOME/TGS/edited/<name>.mod      (digest auth)
        │
        ├─ validate (§6.3)
        ├─ backup current under ROBOT_PROGRAM/<name>_<ts>.mod
        └─ store retrieved bytes as the new master   ← same as the FANUC path
        │
        ▼
next run: compare (normalized, §6.2) → equal → no push → touch-up survives
```

RWS calls, cross-checked against `abb_librws` (`src/rws_client.cpp:477` `getFile`, `:502`
`uploadFile`, URI built at `:798` from `Services::FILESERVICE = "/fileservice"`):

| Purpose | Request |
|---|---|
| Download a file | `GET /fileservice/<dir>/<file>` → 200, raw body |
| Upload a file (replaces the send path too) | `PUT /fileservice/<dir>/<file>` → 200/201 |
| Save a module from memory to disk (option B) | `POST /rw/rapid/modules/<module>?action=save`, params `path`, `name` |
| Mastership, if B is used | `POST /rw/mastership?action=request` … `?action=release` |

Auth: HTTP digest, default UAS user `Default User` / `robotics`. In libcurl that is
`CURLOPT_HTTPAUTH = CURLAUTH_DIGEST` — the existing `FTPManager` read/write callbacks port
over unchanged; only the URL and auth mode differ.

**RWS also replaces the send direction.** `PUT /fileservice/$HOME/TGS/<name>.mod` removes
the 614-1 dependency from the plan entirely, and the HMI then needs one transport, one
credential, one failure mode. `SCWrite`/socket messaging (616-1) stays as-is for the
request protocol — RWS does not touch it.

---

## 5. What changes in the HMI (scope only, no code yet)

1. **A transport behind `ROBOT_BRAND`.** `FTPManager` is FANUC-shaped down to its URL
   builder (`md%3A%5C`) and its `.LS` naming. An `RWSManager` sibling exposing
   `retrieveFile`/`transferFile` in the same shape keeps `WeldingProject` brand-agnostic.
2. **Extension and path.** `.LS` → `.mod`; `md:\` → `$HOME/TGS/`.
   `GetRobotProgramNameWithExtension()` (`WeldingProject.cpp:2817`) becomes brand-aware,
   and the temp/backup keys in `ReplaceTgsRobotProgramWithConfirmation` inherit it.
3. **Module name ≠ program name.** Our naming contract is *file name = PROC name = program
   name*, but the `MODULE` identifier carries a `_Mod` suffix, because RAPID modules and
   global routines share one namespace (`TGS/TD05Test.mod:11`,
   [abb_port_plan_v1.md](abb_port_plan_v1.md) §4.1 notes). RWS `action=save` addresses the
   module by **module name**, so option B must save `<name>_Mod` while writing
   `<name>.mod`. Easy to get wrong once and never notice.
4. **Compare normalization** — see §6.2.
5. **Error strings.** `ERROR_WHEN_RETRIVING_ROBOT_PROGRAM_FROM_CONTROLLER`
   (`ErrorManager.h:137`) says "ensure … the program is not selected on the Teach
   Pendant", which is a FANUC constraint. The ABB equivalents: a module cannot be unloaded
   while executing, `Save` must not run during motion (3HAC050917 §1.229 Limitations), and
   mastership may be held elsewhere.

---

## 6. Consequences for the `.tgs` contract

### 6.1 Keeping weld data out of the `.tgs` is now load-bearing, not just tidy

§3.1 of the weld design doc argued for controller-resident weld data on namespace and
lifecycle grounds. Retrieval adds a harder reason: **a module save serializes PERS current
values into the declarations** (Appendix A.2). A `.tgs` carrying PERS would therefore
serialize *differently on every save* — the retrieved file would never byte-match the
stored master, so the HMI would re-push every run, defeating both the compare optimization
and the touch-up. A `.tgs` of `CONST` targets + request calls round-trips stably.

### 6.2 The byte-exact compare will not survive a save/retrieve round trip unchanged

`CompareCurrentWithProgramInTheController` (`WeldingProject.cpp:2873`) is
`current_program == controller_program`, byte for byte. A module saved by the controller is
re-serialized by RAPID's own writer — formatting, indentation and line breaks are the
controller's, not our exporter's. So on the **first** run of a freshly generated program
the compare differs and the HMI pushes (correct). After a retrieve, both sides hold the
controller's serialization and the compare should match — **provided the controller's
writer is deterministic across saves.** ⚠ That is test T4 (§8). If it is not byte-stable,
the compare must normalize (strip trailing whitespace/CRLF, ignore comment lines) or hash
the semantic content instead.

### 6.3 Validate what comes back

The FANUC path fingerprints a program with a regex for `CALL SET_ROB_S_SR('Ok')`
(`FTPManager.cpp:283` `IsFileATGProject`). The ABB analog, given how `TG_Main` late-binds:

- first non-comment line is `MODULE <name>_Mod`,
- a `PROC <name>()` exists (else `%stTG_ProgName%` fails at runtime),
- file is non-empty and parses as text.

Retrieving and adopting an unvalidated `.mod` would put a program on the master path that
`TG_Main` cannot call.

---

## 7. Corrections to existing docs

1. **[abb_weld_motion_and_data_design_v1.md](abb_weld_motion_and_data_design_v1.md) §3.1
   item 2** claimed a PERS in a `Load \Dynamic` module is "never written back to the .mod".
   Too strong — `UnLoad \Save` and `Save` both write back, and PERS current values *do* land
   in the declarations on save. The true statement is narrower: *plain* `UnLoad` discards,
   silently. Fixed in place.
2. **[abb_port_plan_v1.md](abb_port_plan_v1.md) item 13 / §7 item 3** — the "FTP option" is
   `614-1`, an FTP/SFTP/NFS **client**, not a server, and it is absent from the cell's
   option list. RWS (base RobotWare-OS) covers both directions with no option. Flagged in
   the plan, pointing here; the decision itself is the user's to re-make.

---

## 8. VC verification (⚠ = blocks the design if it fails)

Setup: the Phase 3/4 VC from [robotstudio_setup.md](robotstudio_setup.md), manual mode,
`TD05Test` loaded through `TG_Main`.

**T1 — does the pendant offer Modify Position inside a `\Dynamic` module?**
Run one cycle, stop inside the `.tgs`, Program Editor → select `jtCap1` → *Debug → Modify
Position*. *Expect:* the button is enabled and the confirm dialog appears.
*Pass:* position modified in the editor. *If it fails,* only option C is viable, and only
for resident modules.

**T2 — ⚠ can RWS save a module while the pendant is in manual mode?**
`POST /rw/mastership?action=request`, then
`POST /rw/rapid/modules/TD05Test_Mod?action=save` with `path=$HOME/TGS/edited`,
`name=TD05Test.mod`. *Expect:* 200 and the file appears. *Pass:* the file on disk contains
the modified target. *If mastership is refused in manual mode,* option B cannot be the
primary trigger — fall back to A (`\ErrIfChanged` + `Save`), which runs inside RAPID and
needs no mastership. Also record the VC's RWS port (80, or the configured 888x).

**T3 — does `\ErrIfChanged` fire for a ModPos edit, and for a PERS change?**
Two runs: (a) ModPos a target, then `UnLoad \ErrIfChanged`; (b) no edit, but a PERS in a
resident module changed, then the same unload. *Expect:* (a) `ERRNO = ERR_NOTSAVED`;
(b) ⚠ unknown — the manual defers PERS init-value updates to save time, so the changed flag
may not be set. *Pass:* (a) trips. (b) is information, not a blocker — the `.tgs` carries no
PERS by design (§6.1).

**T4 — ⚠ is the controller's module serialization byte-stable?**
`Save` the same unmodified module twice to two paths; `cmp` them. *Expect:* identical.
*Pass:* byte-identical, twice in a row. *If not,* §6.2's normalization is mandatory.

**T5 — full round trip.** ModPos → `UnLoad \ErrIfChanged` → save → `GET /fileservice/...` →
compare against the pre-edit file. *Expect:* exactly one changed declaration, the touched-up
target, numerically equal to the jogged pose. *Pass:* the diff is that one literal and
nothing else.

---

## 9. Open questions

1. **Which programs get touched up** — the HMI-generated `.tgs` weld programs, or resident
   utility/service programs? This doc assumes the former (matching the FANUC button). If it
   is the latter, the answer is much simpler: keep them resident and out of the dynamic-load
   path entirely.
2. **Trigger choice** — A + `\ErrIfChanged` (recommended), B, or C-only for now?
3. **Does the send path move to RWS too** (dropping 614-1 from the plan), or stay FTP with
   the option purchased?
4. Terminology: the HMI already uses "touch-up" for the per-weld offset widget
   (`WidgetTouchupOffsets.cpp`, `onTouchUpButtonClick` at `GUIMainWindow.cpp:1858`). This
   doc means *pendant position touch-ups*. Worth naming them apart in the UI before both
   exist for ABB.

---

## Appendix A — manual quotes

**A.1 `UnLoad` has both a save and a change detector** — Technical reference manual, RAPID
Instructions, Functions and Data types, 3HAC050917-001 rev H, §1.329:

```
UnLoad [\ErrIfChanged] | [\Save] FilePath [\File]
```

> `\Save` — "If this argument is used then the program module is saved before the unloading
> starts. The program module will be saved at the original place specified in the `Load` or
> `StartLoad` instruction."
>
> `\ErrIfChanged` — "If this argument is used, and the module has been changed since it was
> loaded into the system, then the instruction will generate the error recovery code
> `ERR_NOTSAVED`."

**A.2 PERS current values are folded into the declarations at save time** — Technical
reference manual, RAPID overview, 3HAC050947-001 §1.2.2:

> "Note that if the current value of a persistent is changed, this causes the initialization
> value (if not omitted) of the persistent declaration to be updated. However, due to
> performance issues this update will not take place during program execution. **The initial
> value will be updated when the module is saved (Backup, Save Module, Save Program).** It
> will also be updated when editing program. The FlexPendant will always show the current
> value of the persistent."

**A.3 `Save` needs an explicit path for anything not loaded by RAPID** — 3HAC050917 §1.229:

> "The argument `\FilePath` `\File` can only be omitted for program modules loaded with
> `Load` or `StartLoad`-`WaitLoad` … The argument `\FilePath` `\File` **must** be used to be
> able to save a program module that previously was loaded from the FlexPendant, external
> computer, or system configuration."

Limitations: TRAP routines, system I/O events and other tasks are delayed during the save;
"Avoid ongoing robot movements during the saving."

**A.4 `\Dynamic` modules are dropped when PP is set to main** — 3HAC050917 §1.138, table
"how different operations affect dynamic loaded program or system modules": *Set PP to main
from FlexPendant → Unloaded*; *Open new RAPID program → Unloaded*. (Static: not affected /
unloaded.)

**A.5 Pendant edits do not reach the disk on their own** — Operating manual, IRC5 with
FlexPendant, 3HAC050941-001 rev G §5.3.1 (quoted in §2.1). `Save Module As…` is §5.3.2.
Modify Position requires manual mode and a target with an initial value (§6.4.2) — the
example given is a `CONST robtarget`.

**A.6 Config-driven unloads have a recovery net; the `UnLoad` instruction does not** —
Technical reference manual, System parameters, 3HAC050948-001 §3.4.1:

> "If a changed and unsaved user-loaded module is unloaded due to configuration changes, it
> will be saved to a recovery directory and pointed out in an ELOG message."

**A.7 `614-1` is a client; RWS is base OS** — Product specification, Controller software
IRC5, 3HAC050945-001 §10.1 (quoted in §2.2) and §3.10, the latter inside chapter 3
*RobotWare-OS*.

**A.8 Installed modules are invisible to the operator** — 3HAC050948 §3.4.4: "An installed
module is not visible, that is, it does not occur in the list of modules." Anything
pendant-tunable must be loaded (`Installed := No`), not installed.
