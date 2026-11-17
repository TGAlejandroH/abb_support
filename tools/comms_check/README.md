# comms_check - first contact with the real IRC5

Everything for the on-site communication test lives in this folder. Nothing here
depends on the rest of the repo except `tools/rapid_check.py` one level up, which is
used automatically when present.

| File | What |
|---|---|
| `comms_probe.py` | The whole PC side: ping, Robot Web Services (RWS) checklist, module upload, TCP echo test, bench helpers. Standard library only. |
| `comms_probe.cmd` | Runs the script with the Anaconda interpreter (plain `python` is not on this laptop's PowerShell path). `.\comms_probe.cmd --ip <IP> all` is the same as `python comms_probe.py --ip <IP> all`. |
| `TG_SocketProbe.mod` | The whole robot side: a standalone RAPID echo server. No `main`, no motion, no I/O, no dependency on `TG_Comms`. |
| `tg_deploy.py` / `tg_deploy.cmd` | Replace the resident TG modules on a controller that already has an older build (section 8). |
| `logs/` | One timestamped log per run (git-ignored). Paste these into the trip report. |

**How to run it.** Always from inside this folder (`cd tools\comms_check`):

    Anaconda Prompt:   python comms_probe.py --ip <IP> ping
    Plain PowerShell:  .\comms_probe.cmd   --ip <IP> ping      (nothing in front of it)

`python comms_probe.cmd` is wrong in both shells: it feeds a batch file to Python.

**Status.**
- 2026-09-23, RobotStudio VC `4600-803651_Virtual` (RW 6.15.08): the PC side passed every
  step and every failure path, and the section 6 bench sequence ran end to end, including
  the wrong-bind-IP SETUP ERROR path.
- 2026-09-24, **first real cell** (IRC5 `4600-804589`, RW 6.16.02, MONARC): all four
  steps passed. ping, the RWS checklist with the write round trip, upload and the socket
  echo, first through the service port (RWS) and then through the WAN port (everything).
  616-1 read PRESENT from the live option list and the module loaded first time. Socket
  round trip on the real controller was 250-375 ms, which is RAPID scan time inside the
  probe's loop, not the network (ping was 1-2 ms). Two operational lessons from
  that day are in sections 3 and 7: keep the enabling device pressed, and expect the RWS
  PERS write to be refused in MANUAL.

What a full pass proves: the laptop reaches the controller (ping), RWS answers with
our credentials and lets us read state and write files (the production `.tgs`
delivery path), option 616-1 PC Interface is installed, and a TCP payload goes
PC -> robot -> PC on port 2000 (the production socket path). If all four pass, the
production stack has no communication unknowns left.

Out of scope tomorrow, by design: `TG_Main` under Production Manager, any `.tgs`
program, the positioner. Nothing in this folder touches the customer's program.

---

## 1. The ONLY things you change

| Where | What | Value |
|---|---|---|
| `TG_SocketProbe.mod`, the line `PERS string stTG_ProbeIP:="192.168.125.1";` | The controller's **own** IP on the port the laptop is plugged into. | Service port: leave `192.168.125.1`. WAN port (X6): the address MONARC IT assigned to the controller. **Never `127.0.0.1` on a real IRC5.** |
| Command line of `comms_probe.py` | `--ip` (same address as above), `--user`, `--password` | Whatever MONARC gives you. Defaults are RobotWare's factory account `Default User` / `robotics`. |
| Laptop Ethernet adapter | An address in the controller's subnet | Service port: DHCP (the controller hands one out). WAN port: a static IP in the same subnet as the controller, mask to match, no gateway needed. |

Nothing else. No Python edit, no config file, no repo checkout needed on the laptop
beyond this folder (and `tools/rapid_check.py` if you want the offline RAPID check).

If you cannot edit the `.mod` before loading it, you can still fix the IP afterwards:
FlexPendant -> Program Data -> type `string` -> `stTG_ProbeIP` -> edit. Or, with the
controller in AUTO: `python comms_probe.py --ip <ip> setip <ip>`.

---

## 2. Before leaving the office

1. The tool runs. Two ways, both from INSIDE this folder (`cd tools\comms_check`):
   - Anaconda Prompt (any env): `python comms_probe.py --help`
   - Plain PowerShell, where `python` is not on this laptop's path:
     `.\comms_probe.cmd --help` with nothing in front of it; the wrapper finds
     `%USERPROFILE%\anaconda3\python.exe` itself. Do not type `python comms_probe.cmd`.
   Every `python comms_probe.py ...` below can be typed as `.\comms_probe.cmd ...`.
2. `python ../rapid_check.py TG_SocketProbe.mod` prints `OK` if you edited the module.
3. Optional but worth it: prove the module loads on the virtual controller (section 6).
4. Copy this folder (and `tools/rapid_check.py`) to the site laptop / USB stick.
5. Ask MONARC for: the WAN IP of the controller (if any), the UAS user and password,
   and a controller **backup** before anything is loaded (ABB menu -> Backup and Restore).

---

## 3. On site, in this order

Open an Anaconda Prompt or PowerShell **in this folder** (`cd tools\comms_check`).
Replace `<IP>` everywhere. In PowerShell write `.\comms_probe.cmd` where the lines below
say `python comms_probe.py`.

### Step 1 - ping

    python comms_probe.py --ip <IP> ping

Pass: `[PASS] ping  4/4 replies`. Fail: wrong IP, laptop not in the controller's
subnet, wrong port on the cabinet, or the controller is off. Fix this before anything
else; nothing below works without it.

### Step 2 - RWS checklist (read-only first, then one file round trip)

    python comms_probe.py --ip <IP> --user "<USER>" --password "<PASSWORD>" rws

Expected lines and what each one means:

| Line | Meaning | If it fails |
|---|---|---|
| `[PASS] rws-identity  system '<name>', RobotWare 6.16...` | HTTP + digest credentials OK. RWS is base RobotWare, so this must work regardless of options. | `NETWORK`: port 80 not reachable on this IP (over the WAN port that is open question Q9 in the feasibility report; retry on the service port). `AUTH` (401): wrong user/password. |
| `[PASS] rws-options  616-1 PC Interface PRESENT` | Sockets can be tested. | `[FAIL] ... ABSENT`: the option was never installed. The socket test is impossible today; everything else still counts. Confirm on the pendant: ABB menu -> System Info -> System Properties -> Options. |
| `[PASS] rws-state  opmode MANR, controller motoroff, RAPID stopped ...; tasks: T_ROB1*` | Controller state readable. Note the task name; if it is not `T_ROB1` add `--task <name>` to every command. | |
| `[PASS] rws-modules  N modules loaded in T_ROB1; no TG module loaded` | Module list readable. Expect no TG module on a fresh cell. | |
| `[PASS] rws-home  HOME: listed ...` | File service readable. | |
| `[PASS] rws-write  PUT / GET / DELETE round trip on HOME:/TGS OK` | The account can write files. This is the `.tgs` delivery path. Leaves `HOME:/TGS` created, which production needs anyway. | `GRANT` (403): the account has no file write grant. Create the dedicated `TG` UAS user (Controller -> UAS on the pendant or in RobotStudio) with file read/write grants and retry with its credentials. |

The write test needs no RAPID mastership, so it works with the pendant in MANUAL.
Add `--no-write` if MONARC does not want anything written yet.

### Step 3 - put the probe module on the controller

    python comms_probe.py --ip <IP> --user "<USER>" --password "<PASSWORD>" upload

Runs `rapid_check` on `TG_SocketProbe.mod`, then PUTs it to `HOME:/TGS/TG_SocketProbe.mod`
and reads it back. Pass: `[PASS] upload ... is on the controller at HOME:/TGS/TG_SocketProbe.mod`.
Skip this if 616-1 is absent: the module cannot load anyway.

### Step 4 - load and start the probe on the FlexPendant

Controller in **MANUAL**, motors on. The routine has no motion, but in MANUAL the
enabling device must stay pressed while RAPID runs, so do step 5 promptly. The probe
waits at most 90 s per connection and then stops by itself.

1. Program Editor -> Modules -> File -> **Load Module...** -> `HOME:/TGS/TG_SocketProbe.mod`.
   **This load is the 616-1 test.** If it is refused and the event log names `SocketCreate`,
   `SocketBind` or similar as unknown, the option is missing. A different error is a bug in the module; save the event log text.
2. Program Data -> `string` -> `stTG_ProbeIP` must read `<IP>`. Edit it here if not.
3. Debug -> **PP to Routine...** -> `TG_SocketProbe`.
4. Press Start. Operator Window must show:

       TG PROBE: bind <IP>:2000
       TG PROBE: listening - run comms_probe.py socket on the PC

   `TG PROBE: SETUP ERROR, ERRNO = ...` right after the bind line means the IP is not one of
   the controller's own addresses. Fix `stTG_ProbeIP` (step 2) and start again.

Do NOT touch the program pointer of the customer's `main`; PP to Routine is enough,
and when the probe ends execution simply stops.

**Keep the enabling device pressed until the pendant prints `TG PROBE: done`.** Releasing
it is a guard stop: RAPID stops wherever it is, and if that is inside `SocketAccept` the
listener stays open in the controller's TCP stack. The PC then reports
`connected, but no answer` and keeps retrying; nothing is broken. Restart the routine
(PP to Routine, Start) and the close-before-create at its top clears the old listener.
Seen on the real cell 2026-09-24.

### Step 5 - the socket test

    python comms_probe.py --ip <IP> socket

Pass on the PC: `[PASS] socket  echo OK from <IP>:2000, round trip N ms`, then
`[INFO] socket-quit  QUIT sent, robot answered 'TG_BYE'`.
Pass on the pendant:

       TG PROBE: client connected from <laptop IP>
       TG PROBE: received 'TG_PING hh:mm:ss'
       TG PROBE: client connected from <laptop IP>
       TG PROBE: received 'QUIT'
       TG PROBE: done, connections served = 2

The PC keeps retrying the connect for 180 s (`--wait N` to change), so you can start it
before or after pressing Start on the pendant. Fail after the wait: the routine is not
running, the bind IP is wrong, or a firewall on the customer's network sits between the
two (the laptop only makes an outbound connection, so the Windows firewall on the laptop
is not involved).

### Step 6 - clean up

Program Editor -> Modules -> select `TG_SocketProbe_Mod` -> File -> Delete Module
(do not save it into the program). Optionally delete `HOME:/TGS/TG_SocketProbe.mod`;
leaving `HOME:/TGS` in place is fine and useful.

### One-shot alternative to steps 1, 2, 3 and 5

    python comms_probe.py --ip <IP> --user "<USER>" --password "<PASSWORD>" all

Runs ping -> rws -> upload, prints the pendant steps, then waits up to 180 s for the
probe to appear on port 2000 and runs the echo. It skips the socket step automatically
when the option list says 616-1 is absent (`--force-socket` overrides).

---

## 4. Reading the results

| Outcome | Conclusion | Next |
|---|---|---|
| ping FAIL | Layer 1/2/3 problem, nothing about the robot's software. | Check adapter IP, cable, which cabinet port. Try the service port. |
| ping PASS, rws-identity NETWORK | The controller answers ICMP but not HTTP on that interface. | Retry on the service port. If RWS works there but not over WAN, note it: that answers Q9. |
| rws-identity AUTH | Credentials. | Get the right UAS account. `Default User` may be disabled on a customer cell. |
| rws-write GRANT | Account can read but not write. | Dedicated `TG` UAS user with file write grants. |
| rws-options 616-1 ABSENT | Sockets impossible until ABB installs the option (licence + Installation Manager rebuild). | RWS results are still the deliverable of the day. Feasibility report B1 / Q5 stand. |
| module load refused, log names Socket* instructions | Same as above, confirmed on the controller itself. | |
| bind SETUP ERROR | `stTG_ProbeIP` is not the controller's IP on that port. | Fix the PERS, start again. |
| socket WARN `connected, but no answer` repeating | The routine is stopped (enabling device released, guard stop) with its listener left open, or it is just restarting. | Restart the routine and keep the enabling device pressed; the PC keeps retrying for the whole `--wait`. |
| setip HTTP 403 | RWS PERS writes need RAPID mastership; in MANUAL the FlexPendant holds it. | Edit `stTG_ProbeIP` on the pendant (Program Data), or edit the file and reload the module. |
| socket FAIL with routine listening | Something between the laptop and the controller blocks TCP 2000 (customer switch/firewall), or the laptop connects to a different IP than the one bound. | Compare `--ip` with `stTG_ProbeIP`; try the service port. |
| all PASS | Communications are done. The production stack needs only the 616-1 socket library it was built on. | |

---

## 5. Handing the console to Claude

If Claude Code is driving from the laptop, it needs three things typed once: the IP,
the user and the password. It then runs the commands above in order, reads the logs,
and tells you which pendant step is next. The pendant steps (section 3, step 4 and
step 6) are always yours.

---

## 6. Bench / AUTO automation (virtual controller, or a cell in AUTO)

In AUTO nobody holds RAPID mastership, so the pendant steps can be done over RWS.
On the VC the controller's own IP is `127.0.0.1`. Sequence, from this folder with
RobotStudio's VC running (`--user/--password` default to the VC's factory account):

    python comms_probe.py --ip 127.0.0.1 stop
    python comms_probe.py --ip 127.0.0.1 upload
    python comms_probe.py --ip 127.0.0.1 load
    python comms_probe.py --ip 127.0.0.1 setip 127.0.0.1
    python comms_probe.py --ip 127.0.0.1 run
    python comms_probe.py --ip 127.0.0.1 socket
    python comms_probe.py --ip 127.0.0.1 unload
    python comms_probe.py --ip 127.0.0.1 restore

`load` reads the module list and the event log after the `loadmod` POST, because the
POST itself reports success even for a module that fails to compile. `restore` puts
the program pointer back to `main` and starts with cycle forever, i.e. back to
Production Manager's `ExecEngine` on the MONARC bench. `run` sets the program pointer
over RWS and verifies it by reading the PP back; if no action is accepted by this
RobotWare, it says so and you do PP to Routine in RobotStudio instead.

Validated 2026-09-23 on the VC exactly in this order, plus one extra `run` before the
`setip` to see the wrong-IP path: the routine printed its SETUP ERROR and ended, `socket`
reported no listener, RAPID was `stopped`. Round trip on the VC was about 460 ms, which
is VC scheduling, not the network; a real IRC5 answers in a few ms.

The same sequence works on the real cell in AUTO, but the first real contact should be
done from the pendant in MANUAL as in section 3: it keeps a person at the enabling
device and it does not need mastership.

---

## 7. Facts this folder relies on

- RWS is base RobotWare-OS (no option). Digest auth over HTTP port 80. One session
  (cookie jar) per client, or RW6 locks the account out after a burst (finding V-3).
- The fileservice PUT does not create parent folders; `HOME:/TGS` is created explicitly.
- On RW6 the socket instructions belong to 616-1 PC Interface. The MONARC records
  disagree on whether the cell has it (site survey 2026-08-27 and the 2025 backup: absent;
  socket-start plan finding C-4: present in a later VC). Tomorrow decides.
- `SocketBind` needs one of the controller's own interface addresses. `127.0.0.1` only
  works on a virtual controller.
- RAPID module and routine names share one namespace, so the module is
  `TG_SocketProbe_Mod` and the routine `TG_SocketProbe`. Source is ASCII only.
- RWS fileservice writes need no mastership and work in MANUAL. RWS RAPID symbol writes
  (`setip`) do need it, and in MANUAL the FlexPendant holds it: HTTP 403 on the real cell
  2026-09-24. Hence the pendant edit as the primary way to change the bind IP on site.
- A routine stopped inside `SocketAccept` leaves its listener open; a client connects and
  gets no answer, and when the routine restarts the pending connection is closed with no
  data. `socket` treats both as "not running yet" and retries until `--wait` expires.
- Real IRC5 round trip through this probe: 250-375 ms, dominated by RAPID scan and the
  TPWrite calls in the loop, with a 1-2 ms ping. The production request loop has
  the same shape, so expect that order of magnitude per exchange, as on FANUC.

---

## 8. Replacing the TG modules (`tg_deploy.py`)

For a controller that already runs an older TG build, such as MONARC since 2026-10-21. The
script takes a copy of everything first, deletes only the files it has a copy of, and keeps
the cell's own settings (tool calibration, taught part data, touch I/O names) across the
swap. It never starts RAPID.

    Anaconda Prompt:   python tg_deploy.py --ip 10.8.8.56 status
    Plain PowerShell:  .\tg_deploy.cmd     --ip 10.8.8.56 status

**The set, in load order:** `TG_CellCal.sys` (the cell data: `tTG_Cam`, `tTG_Weld`,
`stTG_ServerIP`, and the touch-sense I/O names `stTG_TouchDI`, `stTG_TouchOnDO`,
`stTG_TouchActiveDI`, `stTG_TeachModeDO`), `TG_Comms.sys`, `TG_Cell.sys`, `TG_Touch.sys`,
`TG_Weld.sys`, `TG_Main.mod`, `TG_Parts.mod` from `abb/rapid` into `HOME:/TGS`, then
`TG_Python.mod`, TG_RoboCal's calibration server, from the TG_RoboCal checkout beside this
repo (`robot_side/abb/TG_Python.mod`; another checkout: `--robocal <dir>`) into `HOME:/`, where
TG_RoboCal installs it and the cell has it. `TG_SocketProbe_Mod` (left from 2026-09-24) is
removed with the old set. On a controller with an older build, the IP still lives in
`TG_Comms` and the touch names in `TG_Cell` (`status` says so); `backup` reads them there.

`TG_Python` is a normal module whose entry is `python_main`; nothing on the MONARC cell calls
it (its 2026-10-22 backup checked). Its address `stTG_PyServerIP` is cell data like
`stTG_ServerIP`. It listens on port 2000 like `TG_Comms`, but only while `python_main` runs
for a calibration, never during production.

**Why a snapshot comes first.** Unloading a module throws away its live `PERS` values.
`backup` records them together with the old files. For each configuration value (list:
`CONFIG_PERS` in the script), `stage` compares three values: the old file's, the live one,
and the new file's.
- Live equals the old file: nobody changed it on the controller, so the new file's value
  goes up. This is how the MONARC torch TCP in `TG_CellCal.sys` replaces the dummy the cell
  still has.
- Live differs from the old file: it was set on the controller, so the live value is
  carried into the new copy.
- Live differs and the repo changed it too: `stage` stops on a CONFLICT. `--prefer-file NAME`
  keeps the repo value, `--prefer-live NAME` the controller's (a VC keeping its own tools).

The cell's addresses (`stTG_ServerIP`, `stTG_PyServerIP`) and touch-sense I/O names skip that comparison: the live value is always
carried. Only the controller knows them, and an old file can hold the cell's own value (the
2026-10-21 build had `10.8.8.56` typed into its `TG_Comms.sys`), which would read as "nobody
changed it" and let the repo's VC default replace it. `--prefer-file NAME` still puts the
file's value up.

Run-time counters and flags are not carried. A fresh load resets them, as intended.
`stage` also refuses to stage the generic placeholder for `tTG_Weld` or `tTG_Cam`.

### On site, in this order

| # | Where | Do | Pass |
|---|---|---|---|
| 0 | Pendant | Take a full system backup (Backup and Restore). | Backup folder written. |
| 1 | PC | `python comms_probe.py --ip 10.8.8.56 ping` | Reply. If the IP changed, use the new one everywhere below and in step 3 (`--server-ip`). |
| 2 | PC | `python tg_deploy.py --ip 10.8.8.56 backup` | `OVERALL: PASS`. `logs\deploy_<stamp>\backup\` holds the old files. |
| 3 | PC | `python tg_deploy.py stage` | `OVERALL: PASS`. Read the `carry` lines: each is a value taken from the cell. A CONFLICT on `tTG_Weld` / `tTG_Cam` (expected on MONARC): `python tg_deploy.py stage --prefer-file tTG_Weld,tTG_Cam` puts the MONARC TCPs up. |
| 4 | Pendant | Stop the program. Program Editor -> Modules: unload `TG_Python`, `TG_Parts`, `TG_Main`, `TG_Weld`, `TG_Touch`, `TG_Cell`, `TG_Comms`, `TG_CellCal` (when loaded), `TG_SocketProbe_Mod`. (In AUTO with RAPID stopped, `tg_deploy.py ... unload` does this.) | None of them listed. |
| 5 | PC | `python tg_deploy.py --ip 10.8.8.56 delete` | Lists the deleted files. Other files in `HOME:/TGS` and `HOME:/` stay. |
| 6 | PC | `python tg_deploy.py --ip 10.8.8.56 upload` | 8 x `read-back identical`. |
| 7 | Pendant | Load Module, in the order above: `TG_CellCal.sys` ... `TG_Parts.mod` from `HOME:/TGS/`, then `TG_Python.mod` from `HOME:/`. (In AUTO with RAPID stopped, `tg_deploy.py ... load` does this.) | No error after the last load. A transient 40160 between loads is expected. |
| 8 | PC | `python tg_deploy.py --ip 10.8.8.56 status` | `program` PASS (the controller links the program), 8 files `identical to reference`, configuration differing: `none`. |
| 9 | Pendant | PP to Main, then run as usual. Then take a second backup. | Production Manager idle as before. |

`status` is read-only and can be run at any time. Run it before step 2 as well. If it
reports `program FAIL` BEFORE anything was touched, the program already had errors and
step 8 cannot judge our files. Every command writes `logs\comms_check_<stamp>.log`.

**Going back:** the old files are in `logs\deploy_<stamp>\backup\`. Unload the new set,
put the old files back (`TG_Python.mod` in `HOME:/`, the rest in `HOME:/TGS`), and load them
in their old order (`TG_CellCal` first when the old set has one; the 2026-10-21 set has none,
so `TG_Comms` first). The tag `v1.0.0` is the abb_support checkpoint before the cell-data move.

**Checks `stage` makes:**
- `stTG_ServerIP` in `TG_CellCal.sys` and `stTG_PyServerIP` in `TG_Python.mod` equal
  `--server-ip` (default `10.8.8.56`).
- `TG_Main.mod` declares `tgs_main()` and no `main()`, because Production Manager owns `main`.
- The `SetDO doTG_Camera` lines are commented out: the cell has no such signal. Use `--camera-signal` once it does.
- `rapid_check` passes on every file, and no global name is declared twice across the set.

`status` also checks that no global name in the new set is already declared by a module
outside the set (the cell's own programs).

**Rehearsed 2026-11-16 on VC `4600-803651_Virtual`** (616-1, PM, the `TG_Python` module).
The VC was first put in the cell's state: the 2026-10-21 site build loaded, plus
`TG_SocketProbe_Mod`, an edited `tTG_Cam`, a taught `padvTG_9011` and a changed
`stTG_TouchDI`, with `tTG_Weld` left on the old dummy. The steps status, backup, stage,
unload, delete, upload, load and status then all passed:
- the program linked;
- the 7 files read back identical;
- `tTG_Weld` became the MONARC torch TCP, and the 3 values edited on the controller were
  carried;
- the name-clash check passed with `TG_Python` loaded.

The CONFLICT path and `--prefer-file` were tested offline. The VC was then restored
exactly: same modules, same files, all 72 PERS values.

**The cell-data layout, deployed 2026-11-16 on the same VC** over the v1.0.0 set (IP in
`TG_Comms`, touch names in `TG_Cell`), with
`stage --server-ip 127.0.0.1 --expect-system 4600-803651_Virtual --prefer-live tTG_Weld,tTG_Cam`
(the VC keeps its own tools, not the MONARC TCPs): every step passed. `backup` read the IP and
the four touch names from their old modules; after `load` they read the same from
`TG_CellCal`, the program links, configuration differing: none. Set on the controller, all
five then survived a reload of `TG_Comms`, `TG_Cell` and `TG_Touch`.

**The full MONARC sequence with `TG_Python`, rehearsed 2026-11-16 on the same VC.** The VC
was put in cell 2's state from its 2026-10-22 backup: the cell's files in `HOME:/TGS` and
`HOME:/TG_Python.mod`, its modules loaded with their live values (IP `10.8.8.56` in
`TG_Comms`, the tools the cell's programs had written, the pre-011 `TG_Python`), no
`TG_CellCal`. Then status, backup, stage, unload, delete, upload, load and status ran from
this tree, as on site:
- `stage` stopped on a CONFLICT for `tTG_Weld` and `tTG_Cam`: the cell's live tools are not
  the old file's dummies, and the repo has the MONARC TCPs. `stage --prefer-file
  tTG_Weld,tTG_Cam` put the MONARC TCPs up (owner, load 1 kg); `stTG_PyServerIP` was carried.
- 8 files read back identical, the program links, configuration differing: none.

Expect the same CONFLICT on the cell.
