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

- `wobjTG_Cam.uframe` ≈ `[[850,-120,400],[…]]` and `wobjTG_Weld.uframe` ≈
  `[[900,80,350],[…]]` — the dummy frames served by `abb_server.py`
  (`cam_frame_xyzwpr` / `weld_frame_xyzwpr`), proving the received frames
  landed in the work objects the .tgs program uses.
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
parameter (`WObj.uframe:=...`).

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
