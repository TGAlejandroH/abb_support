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
