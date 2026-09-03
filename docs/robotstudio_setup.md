# RobotStudio Setup & Phase 1 Smoke Test

How to run the ABB side (`abb/rapid/`) in a RobotStudio virtual controller (VC)
against the Python HMI prototype (`hmi_prototype/abb_server.py`).

Target: IRB 4600-20/2.5 · RobotWare 6.15.08.0 · IRC5 (matches the real cell).

## 1. Create the station / virtual controller

1. RobotStudio → **File → New → Solution with Station and Virtual Controller**.
2. RobotWare: **6.15.08** (install via Add-Ins tab → RobotWare 6.15.08 if missing).
3. Robot model: **IRB 4600-20/2.50**.
4. Check **Customize options**, then in the option editor make sure
   **Communication → 616-1 PC Interface** is selected (required for the
   `Socket*` RAPID instructions; free on a VC).
5. Create, wait for the controller to start (green play state in the
   Controller tab).

For an existing VC: Controller tab → *Change Options* → add 616-1, restart.

## 2. Load the RAPID modules

Controller tab → **RAPID** → expand `T_ROB1`:

1. Right-click `T_ROB1` → **Load Module…** → `abb/rapid/TG_Comms.sys`.
2. Right-click `T_ROB1` → **Load Module…** → `abb/rapid/TG_Main.mod`.
3. **Apply** (Ctrl+Shift+S) — the RAPID editor now compiles both. Fix-ups, if
   any, will show in the Output window. Expect zero errors.

Notes:
- `TG_Comms` is a SYSMODULE: it stays loaded across "PP to Main" and is hidden
  from the operator's program list — intended.
- The VC binds `127.0.0.1` (default of `stTG_ServerIP`). On a **real IRC5**,
  edit the PERS value to the controller's LAN IP (RAPID → TG_Comms →
  `stTG_ServerIP`), or change it from the FlexPendant (Program Data → string).
  Port lives in `nTG_Port` (default 2000).

## 3. Run the smoke test

Robot side:
1. RAPID tab → **Program Pointer → Set Program Pointer to Main** (task T_ROB1).
2. Controller tab → Operator Window open (to see the `TPWrite` traces).
3. Press **Start** (Simulation → Play, or the RAPID Start button; controller in
   Auto). Expected trace:
   ```
   TG: main started
   TG: socket disconnected
   TG: waiting for HMI on port 2000
   ```

PC side (from the repo root):
```
python hmi_prototype/abb_server.py            # defaults: 127.0.0.1 2000, 2 cycles
python hmi_prototype/abb_server.py 127.0.0.1 2000 5   # explicit host/port/cycles
```

Expected on the Python console (per cycle):
```
--- cycle 1/2 ---
connected to robot at 127.0.0.1:2000
  robot prompts 'Give me the program ID'
  hmi   -> '1'
  robot -> '100'
serving request 100
  robot -> '[[<x>,<y>,<z>],[<q1>,<q2>,<q3>,<q4>]]'
  robot -> 'none'
  end request: pose(xyzwpr)=[...] sub='none'
robot disconnected (end of cycle)
```

Expected on the Operator Window (per cycle):
```
TG: waiting for HMI on port 2000
TG: HMI connected: 127.0.0.1
TG: program ID = 1
TG: end request served
TG: socket disconnected
TG: waiting for HMI on port 2000
```

**Pass criterion (Phase 1)**: two consecutive cycles complete without manual
intervention on either side ("all cycles complete" on the Python console).

The Python client retries connecting for up to 30 s, so the ~2 s the robot
spends between cycles (socket teardown `WaitTime`) is bridged automatically —
start order does not matter, robot-first is simplest.

## 4. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| RAPID error on `SocketCreate` (feature not supported) | PC Interface 616-1 missing from the VC system — §1 step 4. |
| Python `ConnectionRefusedError` after 30 s | Robot program not running, or wrong port — check the Operator Window shows "waiting for HMI". |
| Windows firewall prompt on first run | Allow python.exe on private networks (only matters for real-robot use; loopback usually unaffected). |
| Robot stuck > 60 s waiting and errors out | Should not happen — all receives use `\Time:=WAIT_MAX`. If it does, a receive is missing that argument. |
| `TG: cycle error, ERRNO = …` then a new cycle | Expected recovery path: the HMI side died mid-cycle; the robot resets the sockets and listens again. |
| Port seems dead after stopping RAPID mid-cycle | Restart the program from main — `main()` starts with `TG_SocketDisc` and `TG_SocketCom` closes before re-creating. Worst case: warm-start the controller. |

## 5. Phase 2: full request set + sample .tgs program

Additional module to load (same way as §2):

3. Right-click `T_ROB1` → **Load Module…** → `abb/rapid/TGS/TD05Test.mod`.

In Phase 2 the .tgs module is loaded manually; `TG_Main` calls it by the name
the HMI sends, via late binding (`%stTG_ProgName%`). Phase 3 adds `Load`/
`UnLoad` from `HOME:/TGS/` so nothing is pre-loaded.

⚠ **The robot moves in this phase** (`MoveAbsJ` between safe joint poses at
`v100`). Run it in the simulated station; keep the default joint targets
unless you have changed the cell.

Run exactly as in §3. One Python cycle now exercises every priority request:

```
python hmi_prototype/abb_server.py 127.0.0.1 2000 1
```

Expected request order on the Python console:
`10` (file transfer) → `5` (pass check) → `1`,`2`,`1`,`2` (two camera
frames + captures) → `11` (global captures done) → `4` (weld frame) →
`14` (weld params) → `100` (end).

Things worth checking after the run (Controller tab → RAPID → `TG_Comms` data,
or a RAPID Watch on the PERS variables):

- `wobjTG_Cam.oframe` ≈ `[[850,-120,400],[…]]` and `wobjTG_Weld.oframe` ≈
  `[[900,80,350],[…]]` — the dummy frames served by `abb_server.py`
  (`cam_frame_xyzwpr` / `weld_frame_xyzwpr`), proving the received frames
  landed in the work objects the .tgs program uses.
- ⚠ **`wobjTG_Cam.uframe` and `wobjTG_Weld.uframe` are both identity** — check
  this *before* reading anything into the `oframe` values above. The received
  frame now lands in `oframe`, and that is only equivalent to the old `uframe`
  write while `uframe` is identity (`weld_frame_update_strategy_v1.md` §4). A
  `PERS` keeps what it was last assigned, across loads and across a saved
  station, so **a station that ever ran the pre-2026-09-03 code still holds a
  frame in `uframe`** — and then every pose is double-transformed while the
  `oframe` watch reads exactly right. `TD05Test.mod` normalizes both at entry;
  if the values disagree with that, the module did not load.
- `nTG_TravelSpeed`=17.5, `nTG_WeldProc`=5, `nTG_WireFeed`=250,
  `nTG_ArcLength`=2.5, `nTG_PassOK`=1, `nTG_GlobalCapOK`=1.

To exercise the robot-side branches, edit the canned values at the top of
`AbbTgsHmi.__init__` (e.g. `pass_ok = 0` → the program terminates without the
end request, exactly like the FANUC `END`; `weld_status = 2` → abort path;
`udwp_flag = 0` → predefined weld schedule). The automated tests in
`hmi_prototype/test_phase2.py` cover the same branches against a fake robot.

## 6. Phase 3: dynamic loading (nothing pre-loaded)

The VC's `HOME:` maps to a plain Windows folder inside the solution:
`<solution>\Virtual Controllers\<controller-name>\HOME\`. In Phase 3 the .tgs
module is **not** loaded in RobotStudio at all — the Python side copies it
into `HOME:/TGS/` during the file-transfer request (the FTP stand-in), and
`TG_Main` runs `Load \Dynamic` → `%name%` → `UnLoad` around the call.

Setup changes from Phase 2:

1. **Remove the manually loaded `TD05Test_Mod` module** from `T_ROB1`
   (right-click → Delete, Apply). Only `TG_Comms` and `TG_Main` stay loaded.
   (If you forget, the robot logs `TG WARN: module already loaded - using it`
   and runs the pre-loaded copy — tolerated, but then the dynamic path is
   not what is being tested.)
2. Reload the updated `TG_Main.mod` (it now contains the Load/UnLoad logic).

Run (note the 4th argument — adjust the path to your solution):

```
python hmi_prototype/abb_server.py 127.0.0.1 2000 2 "C:\...\<solution>\Virtual Controllers\Controller1\HOME"
```

Expected additions to the transcripts:

- Python: `transferred ...\abb\rapid\TGS\TD05Test.mod -> ...\HOME\TGS\TD05Test.mod`
  during request 10.
- Operator window: `TG: loading HOME:/TGS/TD05Test.mod` before
  `TG: calling program TD05Test`.
- In the RAPID browser you can watch the `TD05Test_Mod` module appear during
  the run and disappear after `UnLoad`.

Error paths (all end the cycle cleanly and reconnect):
missing file → `TG ERROR: cannot load ...`; failed copy on the Python side →
ftp status 0 → `TG ERROR: file transfer failed - skipping program`;
module lacking the expected PROC → `TG ERROR: no PROC named ...`.

**Pass criterion (Phase 3)**: two consecutive cycles with the module loaded
from `HOME:/TGS/` each time (delete `HOME:\TGS\TD05Test.mod` beforehand to
prove it is the fresh copy being run).

## 7. Weld-frame demonstration (visible proof the received frame takes effect)

`TD05Test.mod` moves to the **same target twice** — once before
`TG_ReqWeldFrame` and once immediately after it — using
`MoveJ rtWeldDemo,v200,fine,tTG_Weld\WObj:=wobjTG_Weld`. `rtWeldDemo` is
expressed in `wobjTG_Weld`, so the identical instruction lands the robot in a
different place once the HMI's frame has been written into the work object.

With the default frame served by `abb_server.py`
(`weld_frame_xyzwpr = [900, 80, 350, -2.5, 3.5, 90]`) and
`rtWeldDemo` at `[1000, 0, 600]` in the work object:

| | TCP in base coordinates |
|---|---|
| before `R_W_F` (frame reset to identity) | `[1000.00, 0.00, 600.00]` |
| after `R_W_F` (frame received from the HMI) | `[873.83, 1114.73, 887.26]` |

The two positions are **1158 mm apart** — the robot visibly swings left and up.
Both are inside the IRB 4600-20/2.50 envelope (horizontal radius 1416 mm).
The Operator Window prints both, so the move can be checked numerically:

```
TG DEMO: before R_W_F, TCP =[1000,0,600]
TG: weld frame set, weld status = 1
TG DEMO: after  R_W_F, TCP =[873.83,1114.73,887.26]
```

Change `weld_frame_xyzwpr` in `abb_server.py` and the "after" position follows
it — that is the whole point of the frame request.

Two demo-only concessions, marked as such in the module:

- The weld frame is **reset to identity** right before the first move, so the
  "before" position is identical on every cycle. `wobjTG_Weld` otherwise
  persists between runs (exactly like FANUC `UFRAME[6]`), which would make
  cycle 2's "before" equal to cycle 1's "after". A production .tgs program
  must never clear a received frame.
- `ConfJ\Off` / `ConfL\Off` around the demo (restored at the end): one stored
  `confdata` cannot be valid for the same target evaluated in two different
  frames, so configuration control is relaxed for the demonstration only.

## 8. Cell I/O macros: TG_CamOpen / TG_CamClose (FANUC CAM_OPEN / CAM_CLOSE)

`abb/rapid/TG_Cell.sys` ports the FANUC utility programs that flip the camera
flap output (the cover shielding the lens from weld spatter). The signal is a
dummy (`doTG_Camera`) and **must exist in the I/O configuration before the
module can pass the program check** — otherwise Apply fails with an
unknown-symbol error on `doTG_Camera`.

Create the signal (once per controller system):

1. Controller tab → **Configuration → I/O System** → type **Signal** →
   right-click → *New Signal…*
2. Name: `doTG_Camera` · Type of Signal: **Digital Output** ·
   **Assigned to Device: leave blank** (an unassigned signal is simulated —
   exactly right for the VC; the real cell maps it to the flap output).
3. OK → **restart the controller** (warm start) when prompted — I/O config
   changes only take effect after a restart.

Then load `abb/rapid/TG_Cell.sys` into `T_ROB1` the same way as `TG_Comms.sys`
(it is resident, like TG_Comms — the .tgs programs call it via the task-wide
global scope) and Apply.

What to watch during a run:

- Operator Window: `TG: camera flap open` inside each capture branch,
  `TG: camera flap closed` after global-captures-done, after the weld frame
  request, and before rest home — the same call sites as FANUC lines 38/50,
  62, 201 and 409 of `TD05tRJYQd.ls`.
- I/O window (Controller tab → Inputs/Outputs, filter `doTG_Camera`): the
  signal toggles 1/0 live during the cycle.

No wire-protocol change: these macros are controller-local, so the Python side
and its tests are unaffected.

## 9. Settling fix verification (settle ladder in tgSendPose)

Before every reported pose, `tgSendPose` now runs the full settle ladder —
`WaitRob \InPos` → `WaitRob \ZeroSpeed` → `WaitTime nTG_SettleTime` — so no
pose is reported while the servos are still converging (findings doc, "related
observation"). `nTG_SettleTime` is a PERS num (default 0.2 s, the FANUC-parity
value); tune it from the RAPID data view without reloading code, 0 disables it.

Measured history on this VC, request 4's pose vs the programmed demo target
`[1000, 0, 600]`:

| Configuration | Reported pose | Error |
|---|---|---|
| no wait | `[1001.08, -0.29, 600.72]` | 1.3 mm |
| `WaitRob \InPos` only | `[1000.24, -0.08, 600.14]` | 0.28 mm |
| full ladder (expected) | `[1000.00, 0.00, 600.00]` | ≪ 0.1 mm |

**Pass criteria** (reload `TG_Comms.sys`, run one cycle):

1. Request 4's pose on the Python console reads `[[1000.00,0.00,600.00],[0,0,1,0]]`
   (residuals ≪ 0.1 mm are fine).
2. `R_C_F` and `R_C` at the same joint target report identical poses (with
   `WaitRob \InPos` alone they still differed by ~2.2 mm).

Cost: ~0.25 s per reported pose (5–9 per program). If that ever matters, lower
`nTG_SettleTime`; keep the two `WaitRob` calls.

## 10. Explicit \Tool/\WObj request parameters (plan §7.6, style b)

*Status 2026-08-28: **all criteria met** — program check clean, two
consecutive cycles, no fallback warning, every reported pose verified
numerically at the 0.01 mm wire-quantization floor. Frame persistence through
the parameter path confirmed: after the frames had been served once, the
first `R_C_F` of every later cycle (including across a program restart)
reported the predicted persisted value `[[1186.89,-91.53,1051.11],...]`.
Note: reloading `TG_Comms.sys` resets the PERS frames to identity, so the
very first post-reload `R_C_F` pose is in base — expected. Note for
transcript diffing: `CRobT` may return the sign-flipped equivalent quaternion
(one run's `R_W_F` gave `[0,-0,1,0]`, the next `[0,0,-1,0]` — same rotation)
and signed zeros; compare poses numerically with q ≡ −q equivalence, never
byte-wise on quaternion strings.*

The pose-touching requests (`TG_ReqPassCheck`, `TG_ReqCamFrame`, `TG_ReqCapture`,
`TG_ReqWeldFrame`, `TG_ReqEnd`) now take explicit `\PERS tooldata Tool,\PERS
wobjdata WObj` arguments; `TD05Test.mod` and `TG_Main` pass them on every call.
The FANUC-style modal numbers (`nTG_ActTool`/`nTG_ActFrame`) are still present
as a **deprecated fallback** for argument-less calls — nothing exercises them
in a normal cycle. The wire format is unchanged, so transcripts must match the
Phase 3 / §9 runs exactly.

Setup: reload `TG_Comms.sys`, `TG_Main.mod` (the .tgs module needs nothing —
request 10 copies the updated `abb/rapid/TGS/TD05Test.mod` from the repo into
`HOME:/TGS/` each cycle). Run two consecutive cycles.

New syntax exercised (⚠ first VC contact for these, plan §2.14): optional
`\PERS` parameters on user PROCs, conditional argument propagation
(`tgSendPose \Tool?Tool \WObj?WObj`), and a component write through a PERS
parameter (`WObj.oframe:=...`).

⚠ **Before loading `TG_Comms.sys` on this VC — it has no external axis.** The
file declares `wobjTG_WeldStn1` with `ufmec:="STN1"`, a mechanical unit that
does not exist here. Nothing references it, but if the controller resolves
`ufmec` at load or at Check Program rather than at first use, the whole shared
SYSMODULE is refused and the validated non-coordinated path goes with it.
**Comment that one declaration out** (or rename the station to this cell's
unit) if the load or the program check complains about it, and record which it
was — that answers a live question in
`weld_frame_update_strategy_v1.md` §5.3.

**Pass criteria:**

1. RAPID program check is clean after loading (this alone validates the §2.14
   syntax).
2. Two consecutive cycles with the request order of §5 — transcripts identical
   to the §9 run: request 4's pose still `[[1000.00,0.00,600.00],[0,0,1,0]]`
   (residuals ≪ 0.1 mm fine), and the weld-frame demo still reports two
   different TCPs before/after `R_W_F` (this is the proof that the served
   frame written *through the PERS parameter* reaches `wobjTG_Weld`).
3. The Operator Window shows **no**
   `TG WARNING: tgSendPose needs BOTH Tool and WObj` line — that warning means
   some call passed only one argument and silently fell back to the deprecated
   modal numbers.

Optional fallback check (the back-pocket path still works): in the RAPID
watch set `nTG_ActTool:=2`, `nTG_ActFrame:=5`, then from the FlexPendant
call `TG_ReqEnd` with no arguments during a connected cycle — the reported
pose should be the camera TCP in the camera frame. Not part of the standard
pass criteria.

## 11. Error-recovery fixes I1 + I4 (error matrix)

*Status 2026-08-28: **all checks validated**. Check 1 covered by the §12
clean-cycle runs. Check 3 (corrupt-weld): request order `10,5,1,2,1,2,11,4,100`
with no 14, both error lines + `weld status = 2`, "before" demo TPWrite only,
R_E pose exactly the predicted `[[1000.00,0.00,600.00],[0,0,±1,0]]`. Check 2
(corrupt-cam, revised abort build): request order `10,5,1,100`, error lines +
`do capture = 2`, no capture / no camera-flap / no demo lines, clean R_E.
Its R_E pose was verified arithmetically — `[[1752.16,469.49,1565.17],…]` is
the **torch** TCP at the capture-1 joint position, i.e. the camera-tool pose
of the same cycle's R_C_F displaced 100 mm along the tool z axis (the
tTG_Cam→tTG_Weld tframe offset), matching to **0.007 mm** with identical
orientation. That confirms the abort path reports through its explicit
\Tool/\WObj arguments (plan 7.6 style b) rather than any stale selection.*

I1: `ResetRetryCount` in `TG_SocketCom`'s reconnect retries (the RAPID retry
counter is bounded by system parameter *No Of Retry*, default 4 — without the
reset a run of consecutive connect failures halts the server loop). I4: a
malformed frame payload now ABORTS the program (R_C_F → `do capture = 2`,
a robot-side sentinel the wire never carries; R_W_F → `weld status = 2`;
both → clean `R_E`) instead of proceeding against the stale frame — the cam
case was revised 2026-08-28 from skip to abort to match the weld policy.
Happy-path behavior and transcripts are unchanged.

Setup: reload `TG_Comms.sys` only. Note the reload resets the PERS frames to
identity, so first-cycle `R_C_F` poses are in base (expected, see §10).

**Check 1 — program check + clean run.** RAPID program check clean (validates
`ResetRetryCount` and the new branches), then one normal cycle: transcript
identical to §10 (no new lines, no behavior change).

**Check 2 — corrupt camera frame.**

```
python abb_server.py 127.0.0.1 2000 1 "<...>\HOME" corrupt-cam
```

Expected (revised abort behavior): request order `10, 5, 1, 100` — the
first R_C_F completes its full choreography (frame + capture-status
exchanges, keeping the HMI in sync), then the program aborts. Operator
Window: `TG ERROR: bad pose payload:` + `[[850.00,-120.00,4` +
`TG ERROR: bad cam frame payload - aborting program` +
`TG: cam frame set, do capture = 2`, then `R_E` — no captures, no weld
section (no demo TPWrites). The R_E pose (reported from the first capture
position in the persisted weld frame) is ignored by the HMI as always.

**Check 3 — corrupt weld frame.**

```
python abb_server.py 127.0.0.1 2000 1 "<...>\HOME" corrupt-weld
```

Expected: request order `10, 5, 1, 2, 1, 2, 11, 4, 100` — captures normal,
**no request 14** despite the HMI sending `weld status = 1`. Operator Window:
`TG ERROR: bad pose payload:` + payload +
`TG ERROR: bad weld frame payload - aborting program` +
`TG: weld frame set, weld status = 2`, the demo's "before" TPWrite but **no
"after"** (abort skips the second demo move and the return home). The robot
aborts from the demo position with the demo's identity frame, so R_E's pose
reads `[[1000.00,0.00,600.00],[0,0,±1,0]]` (q ≡ −q, §10 note).

**Pass criteria:** all three checks match; in checks 2–3 the wire choreography
never desyncs (the Python side serves every request to completion and prints
`robot disconnected (end of cycle)` / `all cycles complete`).

I1's fault-injection validation (5+ consecutive connect failures surviving the
retry limit) needs the `--die-at` harness from the matrix doc §6 — deferred
with I2/I3; program check plus code review covers it until then.

## 12. Stale-module fix I2 + I3 (error matrix F-B, F-E)

*Status 2026-08-28: checks 1 and 3 validated on the VC. Check 3 confirmed the
bounded fallback, variant (b), and settled a fact: **modules loaded via
RobotStudio cannot be `UnLoad`-ed** — the full expected cascade appeared,
the cycle ran clean, and the expected second `could not unload` appeared
after "program finished". Check 2 (attempt 2) **halted the program at the
failing `SocketReceive`** (events 41595/40228/10020/10126, no handler
output) — this exposed matrix finding **F-F**: unhandled errors do not
propagate through the late-binding `%%` call. Fixed the same day by **I7**
(`tgCycleAbort`/`ExitCycle` in the wire helpers). **Check 2 rev 3 PASSED
2026-08-28**: kill mid-.tgs → `TG: cycle error, ERRNO = 1095` + `TG: wire
lost - restarting cycle` + `TG: main started` + accept loop, simulation kept
running and served the next session untouched. The next cycle showed the I2
variant: `module already loaded - reloading from file` once, then a clean
reload — i.e. **ExitCycle does not drop \Dynamic modules** and the I2 path
covered it (I2 thereby validated in its primary stale-module role). Step 4
(mid-prog-sel kill regression) remains optional — same mechanism, shallower
depth. Check 1's transcripts also exercised the §13 cell-macro placeholders
end-to-end.*

An aborted cycle used to leave the .tgs module loaded in the task; the next
cycle's `Load` hit `ERR_LOADED` and silently ran the **old in-memory copy**
even though a fresh file had just been transferred. Fix (all marked with
`! FIX 2026-08-28` comments in `TG_Main.mod`): `ERR_LOADED` → best-effort
unload + `RETRY` the Load; a `tgTryUnload` helper + `tg_module_loaded` flag;
and an explicit `RAISE` tail in `tgRunTgsProgram`'s handler — implementing it
surfaced that a RAPID error handler falling off its end acts as RETURN, so
mid-.tgs errors were being **silently swallowed** (F-E), never reaching
`tgMainCycle`'s log line.

Setup: reload `TG_Main.mod` only. Happy-path transcripts are unchanged.

**Check 1 — program check + clean run.** Program check clean; one normal cycle
identical to §10.

**Check 2 (rev 3, for the I7 build) — mid-.tgs wire loss recovers via
ExitCycle (F-F).**

0. Cleanup first: the manually loaded module from check 3 is still in the
   task (RobotStudio-loaded modules survive PP-to-main — the attempt-2 log's
   warn cascade proves it). Remove it in RobotStudio (RAPID browser →
   T_ROB1 → right-click `TD05Test_Mod` → delete), reload `TG_Comms.sys` +
   `TG_Main.mod` (both changed by I7), then run one clean cycle — expected:
   **no** WARN lines, transcript matches §10.
1. Start `python abb_server.py 127.0.0.1 2000 1 "<...>\HOME"` and press
   **Ctrl+C in the Python console** once a mid-program request is being
   served (e.g. when `serving request 1` appears).
2. **Wait ~10 s watching the Operator Window without touching RobotStudio.**
   Expected: `TG: cycle error, ERRNO = …` + `TG: wire lost - restarting
   cycle`, then — because ExitCycle restarts main — `TG: main started` /
   `TG: socket disconnected` / `TG: waiting for HMI on port 2000`. The
   simulation must KEEP RUNNING (Play stays active). If it stops again,
   record the event log — that would mean ExitCycle is also blocked from an
   error handler at depth (matrix F-F ⚠).
3. **Without touching RobotStudio**, run one normal cycle. Expected: clean
   transcript matching §10. Note whether `TG WARN: module already loaded -
   reloading from file` appears: absent = the PP move dropped the \Dynamic
   module (expected); present once with a clean reload = the I2 path picked
   it up. Both pass; record which.
4. Regression: repeat the kill during `Give me the program ID` (the Phase-1
   path). Expected: same recovery lines as step 2 (the wire helpers now
   catch this before `tgMainCycle`'s handler does).

**Check 3 — fallback path still works (Phase-2 style manual load).** Load
`TD05Test.mod` into T_ROB1 manually via RobotStudio, then run one cycle.
Expected: `TG WARN: module already loaded - reloading from file`, then either
(a) a clean reload (unload succeeded) or (b) `TG WARN: could not unload …` +
`TG WARN: module already loaded - using it` (RobotStudio-loaded modules may
not be UnLoad-able) — both acceptable; the cycle must complete either way.
Record which of (a)/(b) the VC shows.

**Check 4 — freshness proof (optional, strongest evidence).** After the
Ctrl+C abort of check 2, temporarily edit a TPWrite string in the repo's
`TD05Test.mod` (e.g. append `v2` to the `TG DEMO: before R_W_F` text), run one
cycle, and confirm the **new** text prints — the freshly transferred file ran,
not a stale module. Revert the edit afterwards.

**Pass criteria:** checks 1–3 match (check 2's `TG: cycle error` line is the
F-E confirmation ⚠); check 4 optional.

## 13. Cell-macro placeholders (phase 4)

`TG_Cell.sys` gained the remaining FANUC cell macros as **empty placeholder
PROCs**: `TG_WeldPrep`, `TG_CamPrep`, `TG_DryRunOn`, `TG_DryRunOff`.
`TD05Test.mod` now calls them in the FANUC sample's order (DRY_RUN_OFF →
conditional DRY_RUN_ON after the password check; WELD_PREP at program start
and again before the weld transition).

Setup: reload `TG_Cell.sys` (TD05Test recopies itself on request 10).

**Pass criteria:** (1) RAPID program check clean — this confirms the ⚠ that
an empty PROC body is accepted; (2) one normal cycle with wire transcript and
Operator Window identical to §10 (the placeholders are silent no-ops).

*Status 2026-08-28: **validated** — the §12 check-1 run (two full cycles)
executed the placeholder calls end-to-end with program check clean and
transcripts identical to §10. Empty PROC bodies confirmed legal on RW 6.15.*

## 14. Arc readiness check (`TGArcCheck.mod`) — before any weld code

Standalone diagnostic proving the controller can hold our weld data and run our
weld motions. **No TG_* dependency** — `tool0`/`wobj0`, no socket. Design
rationale in
[abb_weld_motion_and_data_design_v1.md](abb_weld_motion_and_data_design_v1.md)
§2.4 / §4.

### 14.0 What the first run already established (2026-08-31 07:10)

**Step 1 PASSED**, which settles the biggest open question: the `welddata` and
`seamdata` literals compile and read back on *this* controller, so the component
set is confirmed first-hand, not just descriptor-derived:

```
welddata := [ weld_speed, org_weld_speed, main_arc, org_arc ]
arcdata  := [ sched, mode, voltage, wirefeed, control, current,
              voltage2, wirefeed2, control2 ]
```

`weld_speed`, `main_arc.wirefeed` and `main_arc.voltage` — the three paths
`TG_ApplyWeldParams` will write — all exist and accept assignment.

**Step 2 executed without an arc error**, which is real evidence: `ArcLStart` /
`ArcLEnd` run on this system with no welder attached. The likely reason is in the
equipment config — `autoinhib_on = TRUE` (see the arc log), i.e. the process
self-inhibits when the equipment is unavailable, so the arc instructions degrade
to pure motion. That is effectively the "blocked weld" the ABB forums allude to,
obtained for free.

Two things the first run did **not** establish, both fixed in v2 of the module:

| Gap | Why v1 could not answer it |
|---|---|
| Does `welddata.weld_speed` actually govern the weld speed? | The log has no timestamps, and the pass criterion was "watch it take ~34 s". Unverified. |
| Which optional components exist? | The step-3 message printed identically whether or not the probes were uncommented, so the log carried no information. |

### 14.1 Which welder is configured — read the arc log, not the option list

The controller writes its own answer to
`<VC>\INTERNAL\arcLog_T_ROB1.log` on every start. Current content:

```
Found ARC1 WelderType: FronTPSInt EquipmentClass: EIP_awEqFrTPS4K5K
Loaded: RELEASE:/options/arc/WeldEquip/Code/EIP_awEqFrTPS4K5K.mod
  GetCfgDataStr: units = SI_UNITS
```

That is **TPS 4000/5000 over EtherNet/IP**, selected by "Fronius TPS **Integrated**"
(`FronTPSInt`) — *not* TPS/i. Confirming evidence: `HOME\Arc\ConfigTemplates\`
holds `Fronius_EIP` and `FroniusTPS4K5K` but **no `FroniusTPSi`**, and the TPS/i
installer would have created that folder.

To get TPS/i: select the TPS/i power source **and clear "Fronius TPS Integrated"**
— that key overrides the selection to `FRON_EIP` (`FronIntegr1` in
`install_PWS.cmd`). Then re-check the log for
`WelderType: FroniusTPS/i` / `EquipmentClass: awEqFrTPSi`.

| Check | Wrong (current) | Right for this cell |
|---|---|---|
| arc log `EquipmentClass` | `EIP_awEqFrTPS4K5K` | `awEqFrTPSi` |
| `ConfigTemplates\` folder | `Fronius_EIP` | `FroniusTPSi` |
| `FeedReference` signal | `aoFr1Power` | `aoFr1WFSpeed` |

### 14.2 Units — a config choice, not a fixed conversion

The log line `units = SI_UNITS` refers to `PROC/ARC_UNITS`, which RobotWare ships
in three flavours (`arcbase\config\proc\pARC_UNITS.cfg`):

| ARC_UNITS | arc_length | arc_velocity (weld_speed) | arc_feed (wirefeed) |
|---|---|---|---|
| `SI_UNITS` *(active now)* | mm | **mm_s** | **mm_s** |
| `US_UNITS` | inch | **ipm** | **ipm** |
| `WELD_UNITS` | mm | mm_s | m_min |

`US_UNITS` matches the HMI exactly — it sends travel speed and wire speed both in
IPM. Selecting it would let both values pass through with **zero conversion**.
That is a design decision, not just a setting; step 2 of the module measures
which system is actually in force.

### 14.3 Re-run with v2 of the module

Reload `abb/rapid/TGArcCheck.mod` (v2) and run all three routines.

**Step 1** — expect the same six values as before (unchanged):

```
  weld_speed        =8.89
  main_arc.wirefeed =520
  main_arc.voltage  =4.9
  seam purge_time   =0.2
  seam postflow_time=0.05
```

**Step 2 (`TGArcMoveCheck`) — this is the one that matters now.** It times two
runs over the *same* 300.0 mm line and prints both:

```
  A REF  MoveL 300mm v100, sec =<x>
  B WELD 300mm ArcL,     sec =<y>
```

Read them in this order:

1. **A validates the method.** Expect **≈3.0 s** (300 mm ÷ 100 mm/s). If A reads
   ≈0, RAPID lookahead outran the stopwatch and **B means nothing** — report that
   rather than interpreting B.
2. **B answers the design question.** B includes a 141 mm approach at v200
   (≈0.7 s), so subtract that:

| B (minus ~0.7 s) | Conclusion |
|---|---|
| **≈34 s** | `weld_speed` governs, interpreted as **mm/s** → SI_UNITS confirmed, conversion ×0.42333 is correct |
| **≈80 s** | `weld_speed` governs but is read as **IPM** → US_UNITS in force; send HMI values verbatim |
| **≈1.5 s** | **the `v200` argument governs and `weld_speed` does not** — this would invalidate the core assumption of the weld design; report it, do not work around it |

**Step 3 (`TGArcProbeOptional`)** — probes are now **pairs** of lines
(assignment + read-back). Uncomment one pair, run a program check, then run the
routine. Distinctive values (1.5 / 7 / 2) so they cannot be confused with step 1:

| Probe | Component | Buys us |
|---|---|---|
| A | `main_arc.control` | Fronius `aoFr1Dynamic` = the HMI's Arc Control, which FANUC never applied |
| B | `main_arc.sched` | Fronius `JobPort` — job-mode operation only |
| C | `main_arc.mode` | Fronius `ModePort` — explicit mode only |
| D | `org_weld_speed` | read-only probe; production must never write it |

A component that does not exist makes the **whole module** fail to compile — that
failure is the answer for that probe. Re-comment it and continue. Report which
A/B/C/D lines printed.

## 15. Phase 4 weld implementation: TG_Weld.sys + TD05Weld.mod (two real welds)

> **VC-VALIDATED 2026-08-31** - weld-demo ran 2 full cycles; all pass criteria
> in 15.4 met (clamp warning + 8.89/220.133/10/0 on weld 1; UDWP=0 + 12.7 +
> library zeros on weld 2; R_E served both cycles). Note: the clamp warning
> printed "was49" without a space - fixed in TG_Weld.sys after this run
> (cosmetic only, no re-run needed).

What ships (2026-08-31, after the §14 measurements came back conclusive):

- `abb/rapid/TG_Weld.sys` — `PERS seamdata sdTG_Weld`, `PERS welddata
  wdTG_Weld` (the SCH[20] analogue), a 10-slot recipe library `wdTG_Lib{n}`
  (the AWE1WPnn analogue, placeholder values), and `TG_ApplyWeldParams` —
  the ONE place wire values become weld data (IPM→mm/s, Fronius-range
  clamping, org_* kept in step for pendant tune-reset).
- `abb/rapid/TGS/TD05Weld.mod` — the two-weld .tgs program mirroring
  Weld2/Weld3 of `TD05tRJYQd.ls` (capture set trimmed; touch-sense and
  R_W_S out of scope). Kept separate from `TD05Test.mod`, which remains the
  non-Arc comms regression program.
- `hmi_prototype/abb_server.py` — new `weld-demo` mode: serves `TD05Weld`
  with **different parameters per weld** (weld 1 UDWP=1, weld 2 UDWP=0) so
  one run covers both branches. Wire format untouched.
- `hmi_prototype/test_phase4_weld.py` — 14 tests (41 total green):
  `FakeWeldRobot` is the executable spec of the TD05Weld choreography, plus
  the conversion arithmetic the RAPID side must reproduce.

### 15.1 Load

Copy/load `TG_Weld.sys` resident into T_ROB1 (like TG_Comms/TG_Cell; needs
633-4 Arc). `TD05Weld.mod` is NOT pre-loaded — the server transfers it into
`HOME:/TGS/` during request 10, exactly like TD05Test in phase 3.

### 15.2 Run

```
python hmi_prototype\abb_server.py 127.0.0.1 2000 2 "D:\ABB\ABB-IRB-4600-20-2-50\Virtual Controllers\Controller1\HOME" weld-demo
```

then start `TG_Main` main (AUTO, PP to main) as usual.

### 15.3 Expected Operator-Window output (per cycle, weld section)

Weld 1 — HMI sends UDWP=1, proc 1, travel 21 IPM, WFS 520 IPM, arc length
49.0, arc control 0.0 (the HMI's own native-.tgs defaults):

```
TG: weld params, UDWP = 1
TG WELD: arc length clamped high, was 49
TG WELD: applied, UDWP=1
  weld_speed mm/s   =8.89
  wirefeed          =220.133
  arc length (volt) =10
  arc control       =0
```

Weld 2 — HMI sends UDWP=0, travel 30 IPM only:

```
TG: weld params, UDWP = 0
TG WELD: applied, UDWP=0
  weld_speed mm/s   =12.7
  wirefeed          =0
  arc length (volt) =0
  arc control       =0
```

### 15.4 Pass criteria

1. Full choreography twice (2 cycles), each cycle serving **two** complete
   R_W_F + R_W_P rounds (`PWeld2`, `PWeld3`), ending in R_E — Python side
   prints both welds' requests in order.
2. The numeric block above, exactly: 8.89 = 21×0.42333, 220.133 = 520×0.42333,
   12.7 = 30×0.42333 (±0.005 for TPWrite rounding).
3. The **clamp warning appears** for weld 1 — this is deliberate: the HMI's
   default arc length (49.0) is out of Fronius range, so the clamp is
   exercised on the very first weld, not hidden until the real cell.
4. Weld 2's wirefeed/arc values are the `wdTG_Lib{2}` placeholders (zeros) —
   proof the predefined branch reads the library, with only `weld_speed`
   overridden (FANUC `$CMD_WSPEED` parity).
5. All four arc segments execute as motion (autoinhib, §14); optional
   visual check: weld 1 runs its 200 mm seams at ~8.9 mm/s (≈22 s), weld 2
   at ~12.7 mm/s (≈16 s).
6. Both welds run in the frame served by R_W_F (targets are in
   `wobjTG_Weld`) — same mechanism TD05Test already demonstrated.

Skip/abort branches (`weld_status` 0 / 2) are covered by the fake-robot
tests; on the VC they behave as in phase 2 (skip: no R_W_P; abort: straight
to R_E).

## 16. Phase 5: touch-up staging & retrieval (touch-up doc T1-T5)

Code under test: `TG_Main.mod` (`tgUnloadKeepEdits` + `tgSaveEditedModule`),
`TG_Comms.sys` (`nTG_ProgEdited`), `hmi_prototype/rws_client.py` /
`tg_retrieve.py`, and the RWS transfer in `abb_server.py`. Python side is
covered by `test_phase5_retrieval.py` (22 tests, incl. a digest-authenticated
fake RWS server); this section is the RAPID/VC half. Design + rationale:
[abb_program_touchup_and_retrieval_v1.md](abb_program_touchup_and_retrieval_v1.md).

### 16.1 Setup

1. Reload **both** `TG_Comms.sys` (new PERS `nTG_ProgEdited`) and
   `TG_Main.mod` into `T_ROB1`. Reload resets the PERS frames to identity
   (known, section 4).
2. `HOME:/TGS/edited/` does **not** need to exist - `tgSaveEditedModule`
   creates it (`MakeDir`, ERR_FILEACC swallowed once it exists).

⚠ **Caution for every test here: after editing, do NOT move PP to main.**
That unloads a `\Dynamic` module instantly (manual 3HAC050917 section 1.138)
and the edit is gone before our code can see it. Stop -> edit -> Play
(resume) is the whole dance; it is also the operating procedure the real
cell's operators need.

### 16.2 No-edit regression (must come first)

Run the phase-3 loop unchanged:

    python hmi_prototype/abb_server.py 127.0.0.1 2000 2 "<VC HOME dir>"

**Expect:** transcript identical to section 7 (phase 3) - the operator
window shows NO new lines. `UnLoad \ErrIfChanged` on an unmodified module
behaves exactly like the old plain `UnLoad`, `HOME:/TGS/edited/` is not
created, `nTG_ProgEdited` stays 0.
**Pass:** two clean cycles, no `TG: touch-up detected` line, no `edited/`
folder.

### 16.3 The touch-up cycle (T1 + T3a)

1. Start one cycle: `python hmi_prototype/abb_server.py 127.0.0.1 2000 1
   "<VC HOME dir>"`, start RAPID from main in auto.
2. While the .tgs moves (after `TG: calling program TD05Test`), stop RAPID
   execution with the virtual FlexPendant's Stop (or RAPID tab -> Stop).
   The Python client just blocks; the socket survives. Do NOT use anything
   that resets the program pointer - a PP reset drops the `\Dynamic` module
   on the spot (finding F-4).
3. **T1**, on the virtual FlexPendant in manual mode: Program Editor ->
   Modules. *Expect:* `TD05Test_Mod` is in the list (a `\Dynamic` module is
   an ordinary program-memory module). Open it, jog the virtual robot
   slightly (RobotStudio Freehand jog works - ModPos records the current
   position however it was reached), select the `jtCap1` argument in a
   `MoveAbsJ` line, Debug -> **Modify Position**. *Expect:* the button is
   enabled and the confirm dialog appears; confirm.

   ⚠ Do NOT edit the module in RobotStudio's RAPID editor + Apply as a
   shortcut - VC-observed 2026-08-31, finding **F-4** in
   [rapid_validation_findings_v1.md](rapid_validation_findings_v1.md):
   Apply on a module with PP inside it forces a PP reset, the `\Dynamic`
   module is dropped instantly, and RobotStudio pops "The module no longer
   exists on the controller. Save to file?" (answer No). The touch-up never
   reaches `tgUnloadKeepEdits` - only FlexPendant edits are representative
   of an operator touch-up.
4. Auto -> Play (resume). The cycle finishes normally.

**Expected operator-window lines, in order, after `TG: program TD05Test
finished`:**

    TG: touch-up detected in TD05Test
    TG: edited module saved for retrieval

**Pass (T3a):** both lines appear; `<VC HOME dir>/TGS/edited/TD05Test.mod`
exists and contains the modified literal (open it - the jointtarget value
differs from `abb/rapid/TGS/TD05Test.mod`); data view shows
`nTG_ProgEdited = 1`; the next cycle loads fresh (no ERR_LOADED warning,
i.e. the unload after staging really happened).
**If ERR_NOTSAVED had unloaded the module anyway** (manual ambiguity, doc
T3), the `Save` would fail and print `TG WARN: could not save edited
module` instead - record that outcome, it flips the design to
Save-before-UnLoad.

### 16.4 T3b (optional, informational): PERS-only change

`TD05Test.mod` has no PERS on purpose (doc section 6.1). To answer T3b, add
a scratch line `LOCAL PERS num nT3b:=0;` to the module and `nT3b:=1;` at
the top of its PROC, transfer, run one cycle **without any manual edit**.
Whichever way it goes, no action is needed - record whether the touch-up
lines appear (= PERS assignment sets the changed flag) or not (= flag is
ModPos/text-edit only). Remove the scratch lines afterwards.

### 16.5 RWS probe, T2 and T4

1. Probe the VC's RWS endpoint (port 80 unless the VC was configured
   otherwise; `curl.exe` ships with Windows):

       curl --digest -u "Default User:robotics" "http://localhost/rw/system?json=1"

   *Expect:* a JSON system description. Connection refused -> check the
   VC's configured listening port (RobotStudio forum: multi-VC setups use
   e.g. 8880/8881) and substitute it everywhere below.
2. **T2** - with the pendant in MANUAL mode and the program stopped.
   ⚠ Run this on its OWN stopped cycle, NOT on the 16.3 touch-up cycle:
   whether a Save clears the module's changed-since-load flag is exactly the
   T3 unknown, so a pre-unload save could suppress the staging that 16.3 is
   trying to observe. (T2/T4 need no edit at all - they save the unmodified
   module.)

       curl --digest -u "Default User:robotics" -X POST -d "" "http://localhost/rw/mastership/rapid?action=request"

   ✔ ANSWERED 2026-08-31: in manual the FlexPendant holds RAPID mastership
   locally (`GET /rw/mastership/rapid` -> `mastership:"local"`, holder
   `FlexPendant`); the request is refused ("held by someone else",
   0xc004841a) and `action=save` fails on the same code with or without an
   explicit request. **Option B is auto-only; trigger A is unaffected.**
   RW6 mastership domain resources are `cfg`/`motion`/`rapid` (no `edit`).
3. **T4** - switch to AUTO (program still stopped inside the .tgs; module
   loaded), then save twice with no edit in between and compare:

       curl --digest -u "Default User:robotics" -X POST -d "fs-newname=edited&fs-action=create" "http://localhost/fileservice/$home/TGS/"
       curl --digest -u "Default User:robotics" -X POST -d 'path=$home/TGS/edited&name=T4a' "http://localhost/rw/rapid/modules/TD05Test_Mod?action=save&task=T_ROB1"
       curl --digest -u "Default User:robotics" -X POST -d 'path=$home/TGS/edited&name=T4b' "http://localhost/rw/rapid/modules/TD05Test_Mod?action=save&task=T_ROB1"
       fc /b "<HOME>\TGS\edited\T4a.mod" "<HOME>\TGS\edited\T4b.mod"

   (Quoting: single-quote the -d bodies in PowerShell so "$home" stays
   literal; in a cmd/conda prompt use DOUBLE quotes - cmd treats ' as data
   and the & splits the command. Always curl.exe/fc.exe in PowerShell.)
   API quirks, VC-observed 2026-08-31: the controller APPENDS `.mod` to
   `name` (pass the base name only), the save does NOT create the target
   directory (org_code -530 without the fileservice create), no explicit
   mastership is needed in auto (taken internally), and `path` accepts both
   `$home/...` and `HOME:/...` spellings.
   ✔ PASSED 2026-08-31: the two saves are byte-identical. Serialization
   facts: the save writes CRLF line endings throughout (repo sources are
   LF, so +1 byte/line vs the master) but is otherwise character-identical
   for unmodified content; a previously adopted controller serialization
   round-trips byte-exact (doc section 6.2).

### 16.6 T5: the full round trip

After a successful 16.3 run (staged file exists, flag = 1):

    python hmi_prototype/tg_retrieve.py TD05Test --rws http://localhost:80 --dest "<scratch dir>"

(or `--vc-home "<VC HOME dir>"` to exercise the kept copy fallback - it
cannot clear the flag, it says so.)

**Expected output** (RWS variant):

    retrieving TD05Test from RWS at http://localhost:80
    backed up current master to <scratch>\retrieved_backups\TD05Test_<ts>.mod
    adopted retrieved program as master: <scratch>\TD05Test.mod

**Pass:**
1. `fc` (or git diff) between the adopted file and `abb/rapid/TGS/
   TD05Test.mod` shows EXACTLY one changed declaration - the touched-up
   target, numerically matching where you jogged (doc T5).
2. `HOME:/TGS/edited/TD05Test.mod` is gone and the data view shows
   `nTG_ProgEdited = 0` (RWS variant).
3. Re-running the same command prints `no edited program staged ... nothing
   to retrieve` and leaves the scratch dir untouched.
4. RWS transfer leg: rerun 16.2 with the URL instead of the HOME dir -
   `python hmi_prototype/abb_server.py 127.0.0.1 2000 2 http://localhost:80`
   - same clean transcript, module delivered by
   `PUT /fileservice/$home/TGS/TD05Test.mod` this time.

### 16.7 VC results (2026-08-31)

| Check | Result |
|---|---|
| 16.2 no-edit regression (copy transfer) | ✔ 2 clean cycles, no touch-up lines, no `edited/` created; cam-frame transform re-checked to 0.006 mm |
| RWS probe | ✔ RWS on `http://localhost:80`, RW 6.15.8029; live option list confirms **no 614-1** (and no 637-1) |
| 16.6 item 4, RWS transfer leg | ✔ 2 clean cycles via `PUT /fileservice`; file on the VC disk byte-identical to the repo master, mtime postdates controller start |
| 16.3 / T1 | ✔ pendant Modify Position enabled and accepted inside the `\Dynamic` module (first attempt via RobotStudio-editor Apply produced finding **F-4** instead — see the warning above) |
| 16.3 / T3a | ✔ `TG: touch-up detected` + `TG: edited module saved for retrieval`; ERR_NOTSAVED **refused** the unload (module still loaded — the staging Save serialized it). ⚠ Scope: the edit was made in manual but the cycle was **resumed in AUTO**, so the RAPID `Save` executed in auto. Manual-mode execution of the save is **T6**, untested |
| 16.6 / T5 | ✔ retrieved over live RWS; diff = exactly the one ModPos'd declaration; staged joints = the mid-move stop point (interpolation fraction 0.576302 identical on all four moved axes, spread 2e-6); staged file deleted; `nTG_ProgEdited` → 0 (symbol write needed no explicit mastership); rerun → "nothing to retrieve" |
| Serialization observation | `Save` re-serialized ONLY the modified declaration (`9E+9`, full precision); unmodified lines kept character-verbatim (`9E9`) — but line endings are normalized to CRLF on every save (both RAPID `Save` and RWS `action=save`; measured 162 CR on 162 lines). After one retrieve the master is a controller serialization and round-trips byte-exact |
| 16.5 / T2 | ✔ ANSWERED (negative in manual, as designed for): with the FP in manual, `/rw/mastership/rapid` reports `mastership:"local"`, holder `FlexPendant`; the RWS request is refused ("held by someone else", 0xc004841a) and `action=save` fails on the same code — **option B cannot save in manual mode**; trigger A is unaffected (validated). RWS module list labels the loaded module `DynMod` — usable by the HMI to detect a still-loaded module |
| 16.5 / T4 | ✔ two RWS saves in AUTO byte-identical (`cmp`); quirks: `name` gets `.mod` appended, target dir not auto-created (-530), no explicit mastership needed in auto, `$home`/`HOME:` both accepted |
| 16.4 / T3b | ⏳ optional, pending |
| 16.8 / T6 (trigger A in manual) | ✔ PASSED 2026-08-31 in **MANR**: full cycle run and resumed in manual after a ModPos of `jtCap2`; both staging lines appeared, staged file carries the edit, no `20025` guard stop, no save warning. Cross-check: the reported C2 capture pose moved by a **pure base rotation** — radius and z preserved to 5 µm, and the TCP azimuth delta (19.72511°) matches the staged joint-1 delta (19.72512°) to 1e-5°. This run also re-exercised the RWS transfer leg (module delivered by PUT) |

## 16.8 T6: does the RAPID-side save work when the cycle RUNS in manual?

Why this is a separate test: §16.3 made the edit in manual but resumed the
cycle in **auto**, so `tgSaveEditedModule`'s `Save` executed in auto. The real
operator habit is to verify a touch-up in manual reduced speed *before* going
to auto — that path has never been exercised. It is a different question from
T2: T2 failed because RWS **mastership** is held by the FlexPendant in manual,
and mastership gates *remote clients*; a `Save` executed by the RAPID program
is the controller itself and is not a mastership client. The `Save`
Limitations (3HAC050917 §1.229) list no operating-mode restriction, so the
expectation is PASS — but expectation is not validation.

**Procedure** (Claude cannot drive this: manual-mode execution needs the
enabling device held on the virtual FlexPendant).

1. `python hmi_prototype/abb_server.py 127.0.0.1 2000 1 "<VC HOME dir>"`.
2. Switch to **manual**, motors on, and run the whole cycle from the virtual
   FlexPendant with the enabling device held (Play). Stop mid-.tgs, ModPos a
   `jointtarget` as in §16.3, then **resume in manual** and let the cycle
   run to its end — do NOT switch to auto.
3. **Keep the enabling device held until the operator window is quiet.**
   The save runs *after* `TG: program TD05Test finished`; releasing the
   enabling device is a program stop, and 3HAC050917 §1.229 warns that "a
   program stop during execution of the `Save` instruction can result in a
   guard stop with motors off" (`20025 Stop order timeout`).

**Expect:** the same two lines as §16.3 (`TG: touch-up detected` /
`TG: edited module saved for retrieval`) and the staged file in
`HOME:/TGS/edited/`.
✔ **PASSED 2026-08-31** (opmode `MANR`): trigger A's `Save` works in manual
exactly as in auto — mastership never enters the picture for a `Save`
executed by the RAPID program itself, which is the whole reason trigger A was
chosen over the HMI-driven option B (refused in manual, T2). No guard stop
occurred: the `fine` point at the end of the .tgs had already brought the
robot to a standstill before the save.
**Pass:** staged file present and carrying the edit; no `TG WARN: could not
save edited module`; no 20025.
**If it fails:** the touch-up is *lost* on that cycle (the handler warns, then
unloads). Fix would be to save before attempting the unload, or to defer the
save to the next auto cycle — record the ERRNO from the warning line, it
names which.

**Related check while you are there — motion must be finished before the
save.** The same Limitations say "avoid ongoing robot movements during the
saving". Our sample ends with `MoveAbsJ jtHome,v100,fine,tTG_Weld` and a
`fine` zone synchronizes the program with the motion, so the robot is
stationary by the time the save runs. This is a property of the *program*,
not of `TG_Main`: **a generated .tgs must end on a fine point** (or
`TG_Main` must add a `WaitRob \ZeroSpeed` before the save). Manual reduced
speed makes moves longer and is the most likely place to expose it — an
exporter rule worth carrying into the Weld Planner.

## 17. Phase 6: R_W_S weld statistics (id 13)

What is being checked: the new `TG_ReqWeldStats` serves the request in the
right place (once per weld, right after the weld instruction), the csv payload
arrives byte-exact, and the HMI's analytics gate behaves — a row is recorded
only when `succ_ae` is 1.

Reminder on what the numbers are: **dummy values** written by the .tgs program
(see [abb_weld_stats_port_v1.md](abb_weld_stats_port_v1.md)). This section
validates the *request*, not the statistics.

### 17.1 Check A — non-Arc VC, `TD05Test` (comms regression)

Nothing new to set up; the module already in `HOME:/TGS/` is rebuilt from the
repo on every transfer.

```
python hmi_prototype/abb_server.py 127.0.0.1 2000 2 <VC-HOME>
```

**Expected**, in each of the two cycles, between request 14 and request 100:

```
  robot -> '13'
serving request 13
  robot -> '+0123.456,+0007.890,+0001.000'
  weld stats: 123.456 mm (4.860 in), arc on 7.890 s, succ_ae=1 -> recorded
```

FlexPendant: `TG: weld stats +0123.456,+0007.890,+0001.000`

**Pass criteria**
1. The payload is **byte-identical** to the string above — 29 characters, three
   signed 9-char fields. A different width means `tgFmtReal` disagrees with the
   Python `fmt_real`, which would eventually break the C++ parse too.
2. Request 13 appears **exactly once per cycle**, and the tail of the request
   log is `… 14, 13, 100` (FANUC order: parameters, weld, stats, end).
3. `4.860 in` = 123.456/25.4 — proves the mm→inch conversion happens HMI-side.
   The per-cycle tally at the end of the cycle reads
   `weld rows recorded: [(4.86, 7.89)]`.
4. Both cycles identical (the PERS survive a cycle; a drift would show here).

✔ **PASSED 2026-08-31**, 2/2 cycles. Payload byte-identical on both cycles
(`+0123.456,+0007.890,+0001.000`, 29 chars, three 9-char fields); request log
tail `14, 13, 100` with exactly one id-13 serving per cycle; FlexPendant
`TG: weld stats +0123.456,+0007.890,+0001.000`; tally
`weld rows recorded: [(4.86, 7.89)]` (4.860 = 123.456/25.4, so the mm→inch
conversion is on the HMI side as intended).
**Run with the RWS transport** (`http://localhost:80`) rather than a VC HOME
path — either delivery mechanism works here, since R_W_S is downstream of the
transfer. The RWS leg served `TD05Weld.mod`/`TD05Test.mod` cleanly in the same
runs, so this doubles as a phase-5 regression.

### 17.2 Check B — dry run (the analytics gate)

Serve a dry run so the module reports "no arc"
(`nTG_SuccArcEnd := 1-nTG_DryRun`):

```
python hmi_prototype/abb_server.py 127.0.0.1 2000 2 <VC-HOME> dry-run
```

**Expected:**

```
  robot -> '+0123.456,+0007.890,+0000.000'
  weld stats: ... succ_ae=0 -> NOT recorded (succ_ae != 1)
```

**Pass:** the request is still served (FANUC calls `R_W_S` unconditionally
inside the weld branch) but the per-cycle tally printed at the end of the
cycle reads `weld rows recorded: []`. This is the behaviour that keeps dry
runs out of the operator's weld analytics.

✔ **PASSED 2026-08-31**, 2/2 cycles. `TG: dry run = 1` on the pendant, payload
`+0123.456,+0007.890,+0000.000`, `NOT recorded (succ_ae != 1)`, tally
`weld rows recorded: []` — and the distance/time fields are unchanged from
check A, so only the flag moved. This is the check that actually proves
`nTG_SuccArcEnd := 1-nTG_DryRun` evaluates on the controller: the third field
went to zero without any other field or any request order changing.

### 17.3 Check C — Arc VC, `weld-demo` (two welds, two servings)

```
python hmi_prototype/abb_server.py 127.0.0.1 2000 2 <VC-HOME> weld-demo
```

**Expected:** request 13 served **twice** per cycle, in weld order, with
different payloads:

```
weld 2 (first):   '+0200.000,+0022.500,+0001.000'
weld 3 (second):  '+0200.000,+0015.750,+0001.000'
```

**Pass criteria**
1. Two servings, in that order, with those exact payloads. Identical payloads
   would mean one serving was echoed rather than two independent ones.
2. The arc-on times match the speeds each weld was actually served:
   200 mm / 8.89 mm/s = 22.5 s and 200 mm / 12.7 mm/s = 15.75 s. If §15's weld
   speeds are re-tuned, these dummies must be re-tuned with them or the
   transcript stops being self-consistent.
3. **The §15.4 weld criteria still hold** (8.89 / 220.133 / clamp on weld 1;
   12.7 + library zeros on weld 2) — that is the regression half of this check:
   inserting `R_W_S` must not disturb the phase-4 choreography.
4. An aborted weld (`hmi.weld_status = 2`) serves **no** request 13 — the stats
   call lives inside the weld branch. (Covered by the automated tests; not part
   of the VC run below, which exercises the welding path.)

✔ **PASSED 2026-08-31**, 2/2 cycles. Two servings per cycle in weld order,
`+0200.000,+0022.500,+0001.000` (PWeld2) then
`+0200.000,+0015.750,+0001.000` (PWeld3) — distinct, so two independent
servings rather than one echoed payload; tally
`weld rows recorded: [(7.874, 22.5), (7.874, 15.75)]` (7.874 = 200/25.4).
The arc-on times match the speeds actually served in the same transcript:
200/8.89 = 22.50 s and 200/12.7 = 15.75 s.
**§15.4 regression intact**: weld 1 logged `arc length clamped high, was 49`
with `weld_speed 8.89 / wirefeed 220.133 / arc length 10 / arc control 0`;
weld 2 `UDWP=0` with `weld_speed 12.7` and library zeros. Inserting R_W_S did
not disturb the phase-4 choreography.
