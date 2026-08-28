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

## 5. For Phase 3 (reference)

The VC's `HOME:` maps to a plain Windows folder inside the solution:
`<solution>\Virtual Controllers\<controller-name>\HOME\`. The dynamically
loaded .tgs modules will live in `HOME:/TGS/` — creating that folder and
copying `.mod` files into it with Explorer/Python stands in for the FTP
transfer used on the real cell.
