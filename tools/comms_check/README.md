# comms_check - first contact with the real IRC5

Everything for the on-site communication test lives in this folder. Nothing here
depends on the rest of the repo except `tools/rapid_check.py` one level up, which is
used automatically when present.

| File | What |
|---|---|
| `comms_probe.py` | The whole PC side: ping, Robot Web Services (RWS) checklist, module upload, TCP echo test, bench helpers. Standard library only. |
| `comms_probe.cmd` | Runs the script with the Anaconda interpreter (plain `python` is not on this laptop's PowerShell path). `.\comms_probe.cmd --ip <IP> all` is the same as `python comms_probe.py --ip <IP> all`. |
| `TG_SocketProbe.mod` | The whole robot side: a standalone RAPID echo server. No `main`, no motion, no I/O, no dependency on `TG_Comms`. |
| `logs/` | One timestamped log per run (git-ignored). Paste these into the trip report. |

Status 2026-09-23: the whole PC side was run against the RobotStudio VC (RWS checklist
with the write round trip, upload, the socket client against a stand-in echo server, and
every failure path). `TG_SocketProbe.mod` passes `rapid_check` but has **not yet been
loaded on a controller**; section 6 does that in eight commands whenever the VC is free.

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

1. `.\comms_probe.cmd --help` runs. Plain `python` is NOT on this laptop's PowerShell
   path; the wrapper uses `%USERPROFILE%\anaconda3\python.exe`. An Anaconda Prompt works
   too. Every `python comms_probe.py ...` below can be typed as `.\comms_probe.cmd ...`.
2. `python ../rapid_check.py TG_SocketProbe.mod` prints `OK` if you edited the module.
3. Optional but worth it: prove the module loads on the virtual controller (section 6).
4. Copy this folder (and `tools/rapid_check.py`) to the site laptop / USB stick.
5. Ask MONARC for: the WAN IP of the controller (if any), the UAS user and password,
   and a controller **backup** before anything is loaded (ABB menu -> Backup and Restore).

---

## 3. On site, in this order

Open PowerShell in this folder. Replace `<IP>` everywhere.

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
over RWS; if that action is not accepted by this RobotWare, it says so and you do
PP to Routine in RobotStudio instead.

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
