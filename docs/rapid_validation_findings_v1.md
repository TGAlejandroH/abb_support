# RAPID Validation Findings (v1 prototype)

Three defects were found while validating the v1 prototype on the RobotStudio
virtual controller (IRB 4600-20/2.5, RobotWare 6.15.08.0, IRC5), all on
2026-08-28. **None of them could be caught by the Python-side test suite** —
they are properties of the RAPID language and the controller, not of the wire
protocol. They are recorded here because each one generalizes into a rule for
the production port and for the future Weld-Planner RAPID exporter.

The three form a chain: F-1 blocked the module from loading at all, F-2 was
found in the first run that got that far, and F-3 was introduced by F-2's fix.

Inline cross-references live next to the affected design decisions in
[abb_port_plan_v1.md](abb_port_plan_v1.md) §2.10, §4.1 and §4.3.

---

## F-1 — Module name collides with its own PROC name

**Symptom** (RobotStudio program check, at load time):

```
Controller1/RAPID/T_ROB1/TD05Test(1,1): Name error(45): Module name TD05Test ambiguous
Checked: Controller1/RAPID/T_ROB1: 1 semantic errors.
```

**What the code did.** The sample .tgs program followed the FANUC convention of
"file name = program name" literally:

```rapid
MODULE TD05Test          ! <-- collides
    PROC TD05Test()      ! <-- with this
```

**Root cause.** In RAPID, **module names and global routine names share a single
namespace per task**. A module whose name equals a global symbol — including a
routine declared inside that very module — is ambiguous, and the error points at
the module declaration (line 1, column 1), not at the routine.

**Fix.** Suffix the module, keep the routine:

```rapid
MODULE TD05Test_Mod
    PROC TD05Test()
```

Nothing else had to change: late binding (`%stTG_ProgName%`) resolves the
**PROC** name, and `Load`/`UnLoad` address the **file path**. The module name is
never transmitted or referenced, so it is free to carry a suffix.

**Rule for the exporter.** The emitted module name must differ from every global
symbol in the task, including the program's own entry PROC. Convention adopted:
`<ProgramName>_Mod`. Also applies: RAPID identifiers are ≤ 32 characters, start
with a letter, allow `[A-Za-z0-9_]`, and are case-insensitive — so two program
names differing only in case would also collide.

---

## F-2 — Stale frame: a PERS-to-PERS copy froze the received frame

**Symptom.** No error, no warning — **silently wrong geometry**. In the first
successful Phase 2 run, the capture poses of cycle 1 were reported in the robot
base frame instead of the camera frame that had just been received, and cycle 2
reported poses in *cycle 1's* frame. The received frame was always one request
too late.

**How it was found.** Only by analyzing the pasted transcript numerically:
cycle 2's capture pose `[1186.89, -91.53, 1051.11]` turned out to be exactly
cycle 1's base-frame pose transformed into the camera frame the HMI had served
one cycle earlier. Nothing on the controller or in the Python console flagged it.

**What the code did.** The FANUC modal `UFRAME_NUM`/`UTOOL_NUM` was emulated by
copying the wanted frame into an "active frame" persistent, which `tgSendPose`
then read:

```rapid
! in the .tgs program
wobjTG_Act := wobjTG_Cam;                      ! copy taken HERE
...
TG_ReqCamFrame;                                 ! writes wobjTG_Cam.uframe LATER
! -> wobjTG_Act still holds the frame as it was before the request
```

**Root cause.** RAPID `:=` on an aggregate type is a **copy by value**; the
language has no references or pointers. `wobjTG_Act` was therefore a snapshot,
and updating the source afterwards could not reach it.

**Fix.** Remove the copy. The active tool/frame is selected **by number** and
resolved to the live data at the moment the pose is read:

```rapid
PERS num nTG_ActTool  := 8;   ! 2 = camera, 8 = torch
PERS num nTG_ActFrame := 0;   ! 5 = camera, 6 = weld, else base
! tgActTool()/tgActWobj() map the number to tTG_Cam/tTG_Weld/wobjTG_Cam/... on
! every call, so a frame updated by a request is used by the very next report.
```

**FANUC parallel — the same hazard, a different remedy.** This is precisely what
FANUC's "frame-PR-writing routine" invariant defends against: because `R_C_F`
and `R_W_F` rewrite `PR[n]` and not `UFRAME_NUM`, the exporter must re-emit
`UFRAME[n]=PR[n]` after every such call (see the warning in
[fanuc_hmi_request_program_calls_v1.md](fanuc_hmi_request_program_calls_v1.md)
and `FanucTranslator.FRAME_PR_PREP_ROUTINE_NAMES_DEFAULT`). The original ABB
design reintroduced the hazard as a data copy. Selecting by number removes it
**by construction**, so the ABB exporter needs no re-emit rule at all.

**Rules.**
- Never copy a `wobjdata`/`tooldata` that a request PROC may update; reference it
  by name, or select it by number and resolve at point of use.
- The exporter must not emit frame copies (the ABB counterpart of the FANUC
  `UFRAME[n]=PR[n]` re-emit is *nothing*, not a translation of it).
- Verification lesson: a wrong-frame defect produces plausible numbers and no
  diagnostics. Transcript poses must be checked arithmetically against the
  frames served, not merely eyeballed for "looks reasonable".

---

## F-3 — `CRobT`'s `\Tool` / `\WObj` are PERS parameters

**Symptom** (RobotStudio program check, after applying F-2's fix):

```
Controller1/RAPID/T_ROB1/TG_Comms(247,20): Argument error(123): Argument for
'PERS' parameter Tool is not a persistent reference or is read only.
```

**What the code did.** F-2's fix resolved the active tool/frame through
functions, and the results were staged in local variables before the call:

```rapid
VAR tooldata tTmp;
VAR wobjdata wTmp;
tTmp := tgActTool();
wTmp := tgActWobj();
rt := CRobT(\Tool:=tTmp \WObj:=wTmp);           ! <-- rejected
```

**Root cause.** `CRobT` declares `\Tool` and `\WObj` as **`PERS` parameters** —
the same rule motion instructions follow. Such a parameter binds to a persistent
variable, so a `VAR`, a `CONST`, or a function result is not acceptable. (This is
also why the RoboDK RW6 driver keeps its `progTool`/`progWObj` as `LOCAL PERS`.)

**Fix.** Stage the computed values in scratch persistents, refreshed at the point
of use:

```rapid
LOCAL PERS tooldata tTG_Scratch  := ...;   ! internal - not for .tgs programs
LOCAL PERS wobjdata wobjTG_Scratch := ...;
...
tTG_Scratch := tgActTool();                 ! refreshed on every call, so the
wobjTG_Scratch := tgActWobj();              ! copy cannot go stale (cf. F-2)
rt := CRobT(\Tool:=tTG_Scratch \WObj:=wobjTG_Scratch);
```

**Note the tension with F-2.** F-2 says "do not copy frame data"; F-3 forces a
copy in order to call `CRobT` at all. The two are reconciled by *when* the copy
is taken: inside `tgSendPose`, immediately before use, so the snapshot lives for
one pose report only. A copy held across a request call is the bug; a copy taken
at read time is not.

**Rule.** When a tool or work object has to be computed or chosen at runtime,
stage it in a scratch `PERS` and re-assign that scratch immediately before every
use — never pass a `VAR` or a function result to `CRobT`, `CPos`, or a motion
instruction.

---

## Related observation (not a defect) — FIXED 2026-08-28

Within a cycle, `R_C_F` and `R_C` reported poses differing by roughly 3 mm and
0.08° at the same joint target; the weld-frame demo later showed the same
effect directly (TCP reported `[1001.08, -0.29, 600.72]` at a fine point whose
programmed target is `[1000, 0, 600]`). Cause: `CRobT` is read a moment before
the servos have fully settled — `fine` releases program execution on the
stop-point criteria, and convergence continues briefly afterwards. `R_C` read
settled values only because of the `WaitTime 0.2` preceding it (FANUC used
`DELAY` in the same places, for vibration damping).

This matters beyond cosmetics: **captures register against the pose reported at
that instant**, so the error flows straight into registration and weld placement.

**Fix (promoted ahead of Phase 4, per team decision)** — applied in
`tgSendPose`, one chokepoint covering every pose-reporting request
(1, 2, 4, 5, 100). Arrived at in two measured steps:

1. `WaitRob \InPos;` alone (tested on the VC 2026-08-28): request 4's pose
   improved from `[1001.08, -0.29, 600.72]` to `[1000.24, -0.08, 600.14]`
   against the programmed `[1000, 0, 600]` — error 1.3 mm → 0.28 mm — but a
   pose read immediately after a move (`R_C_F`) was still ~2.2 mm away from
   the settled value its `R_C` reported 200 ms later. As suspected, \InPos
   shares its convergence criteria with the `fine`-point release, so it does
   not guarantee standstill.
2. Escalated to the full ladder:
   ```rapid
   WaitRob \InPos;
   WaitRob \ZeroSpeed;
   IF nTG_SettleTime>0 WaitTime nTG_SettleTime;
   ```
   `nTG_SettleTime` is a `PERS` (default **0.2 s** — the value the VC
   measurements showed reaches full settle, and the FANUC parity value:
   `WAIT 0.20` before `R_C`). Tunable from the data view without a code
   reload; set to 0 to disable.

**Validated on the VC 2026-08-28** with the full ladder: request 4 reported
exactly `[[1000.00,0.00,600.00],[0,0,1,0]]` (zero error), and `R_C_F`/`R_C` at
the same joints reported bit-identical poses. Summary of the measurements:

| Configuration | Request-4 error | R_C_F vs R_C at same joints |
|---|---|---|
| no wait | 1.3 mm | ~2–3.5 mm apart |
| `WaitRob \InPos` only | 0.28 mm | ~2.2 mm apart |
| full ladder (`\InPos` + `\ZeroSpeed` + 0.2 s) | **0** | **identical** |

The demo's raw `CPos` TPWrites (deliberately not behind the ladder) still show
the unsettled value in the same run — a built-in control group demonstrating
the ladder is what closes the gap.

Exporter note either way: pose-reporting requests should follow a **stop point**
(`fine`), not a fly-by zone — the FANUC sample does the same (`FINE` before
`R_C`).

---

## How the three were caught (verification takeaway)

| Finding | Caught by | Would the Python tests catch it? |
|---|---|---|
| F-1 module name ambiguous | RAPID program check, at load | No — never reaches the wire |
| F-2 stale frame copy | **Numeric analysis of the run transcript** | No — the protocol was correct, the geometry was not |
| F-3 PERS parameter | RAPID program check, after the F-2 fix | No — RAPID semantics only |

Two of three surfaced as controller error messages and were mechanical to fix.
The one that mattered most, F-2, produced a perfectly well-formed protocol
exchange with wrong numbers in it — the kind of defect that reaches production
unless transcripts are verified arithmetically. That practice is part of the
validation loop described in [robotstudio_setup.md](robotstudio_setup.md).
