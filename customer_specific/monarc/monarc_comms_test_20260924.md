# MONARC comms test, 2026-09-24: PC <-> IRC5 `4600-804589`

**Result: all communications pass.** Ping, Robot Web Services (identity, option list,
controller state, file service read and write), module upload, and a TCP socket echo on
port 2000, all over the **WAN port**. RWS was also confirmed through the service port
earlier the same morning. Nothing about the cell's network or software blocks the
TetraGen ABB port any more; what remains for the real cell is mechanical and organisational
(positioner, Production Manager part registration, SafeMove, PLC bits, a dedicated UAS user).

Tool used: [`tools/comms_check/`](../../tools/comms_check/README.md) (`comms_probe.py`,
`TG_SocketProbe.mod`). One log per run is on the site laptop under
`tools/comms_check/logs/comms_check_20260924_*.log` (git-ignored).
Backup taken after the tests: `C:\Users\TG_Laptop08\Documents\4600-804589_Backup_20260924`
(controller time 10:56:52; **the controller clock runs one hour behind the laptop**, so
event-log times below are +1 h in laptop time).

## 1. The cell as found

| Item | Value | Source |
|---|---|---|
| Controller | IRC5 `4600-804589`, RobotWare **6.16.02.00** (build 2027, dated 2025-11-05) | `BACKINFO/version.xml`, RWS `/rw/system` |
| Welder | **Fronius TPS/i** add-in 1.09 (`RW Add-In loaded Welder`). Not the Miller of the older `4600-803651` backup. | `BACKINFO/backinfo.txt` |
| Options | **616-1 PC Interface present.** Also 812-1 Production Manager, 633-4 Arc, 657-1 SmarTac, 652-1 BullsEye, 1125-2 SafeMove Pro, 996-1 Safety Module, 997-1 PROFIsafe, 888-3 PROFINET Device, 841-1 EtherNet/IP, 613-1 Collision Detection, 608-1 World Zones, 1582-1 IoT Data Gateway, 735-8 Keyless Mode Switch. Absent: 623-1 Multitasking, 614-1 FTP/SFTP client (neither is needed). | `backinfo.txt`; RWS `/rw/system/options` (33 entries, live) |
| Tasks | `T_ROB1` (motion), `SC_CBC`, `tAwSys_1` | RWS `/rw/rapid/tasks` |
| Main program | Production Manager: `gapMain.main()` | `RAPID/TASK1/PROGMOD/gapMain.mod` |
| **WAN port, vision-PC side** | **`10.8.8.56`**, configured by MONARC IT on the day. It does **not** appear in `SYSPAR/SIO.cfg` (unchanged from the 09:51 backup): on an IRC5 the WAN address is a **Boot Application setting** and is not carried by backup/restore. Static address or DHCP lease: **unconfirmed**. | `SIO.cfg`; ping and RWS from the laptop |
| LAN3 / X5 | `192.168.0.14`, PROFINET to the PLC. Never connect the PC here. | `SIO.cfg` |
| Service port | `192.168.125.1`, hands the laptop a DHCP address. RWS confirmed there too. | run 10:59 |
| RWS account | `Default User` / `robotics` (RobotWare factory default) has read access and file-write grants. | runs 10:59, 11:19, 11:41 |

## 2. What passed, in order (laptop time)

| Time | Step | Result |
|---|---|---|
| 10:59 | `ping` + `rws` through the service port `192.168.125.1` | PASS. `HOME:/TGS` did not exist and was created by the write test. |
| 11:02 | `upload` through the service port | PASS |
| 11:19 | `ping` + `rws` + `upload` through the WAN port `10.8.8.56` | PASS. Ping 1-2 ms. |
| 11:41 | `rws` on the WAN with `TG_SocketProbe_Mod` loaded from the pendant | PASS |
| 11:46 | `socket` echo on the WAN | PASS. Round trip 374.6 ms, `QUIT` answered `TG_BYE`. |
| 11:50, 11:51 | `socket` echo twice more, routine restarted each time | PASS. 260.9 ms and 248.3 ms. |

The round trip is RAPID scan time plus the `TPWrite` calls inside the probe's loop, not
the network. The production request loop has the same shape, so a few hundred
milliseconds per exchange is the expected order of magnitude, as on FANUC.

## 3. Lessons, and things to tell MONARC

1. **Enabling device.** In MANUAL, releasing it is a guard stop. The first socket attempt
   at 11:42 connected to a listener the stopped routine had left open in `SocketAccept`
   and got no answer. `comms_probe.py socket` now retries through that state; the fix on
   the pendant is simply to restart the routine and keep the device pressed.
2. **RWS RAPID-symbol writes are refused in MANUAL** (HTTP 403 on `setip` at 11:41): the
   FlexPendant holds RAPID mastership. File-service writes are unaffected. On site the bind
   IP was set by editing the module before upload.
3. **Customer event routine error.** Every program start logs event 10051, "could not start
   the specified system event routine `rR1CrashBoxMonitor`: unknown to the system or the
   program is unlinkable". Pre-existing cell condition, not ours; MONARC should know.
4. **WAN address persistence.** `stTG_ServerIP` binds to `10.8.8.56`. Ask MONARC IT to confirm
   it is static (or a DHCP reservation) and to record it, because a system restore does not
   bring the Boot Application network settings back.
5. **Leftovers on the controller.** `TG_SocketProbe_Mod` was still loaded in `T_ROB1` when the
   10:56 backup was taken (it is saved in `RAPID/TASK1/PROGMOD/TG_SocketProbe_Mod.mod`), and
   `HOME:/TGS/TG_SocketProbe.mod` is on disk. Both are harmless: no `main`, no event hooks,
   no `partdata`. Delete the module from the program on the next visit; `HOME:/TGS` stays,
   it is the production delivery folder.
6. **Dedicated UAS user.** The factory account was used for the test. The feasibility
   report's recommendation of a `TG` user with file-write and RAPID-modify grants stands.

## 4. What this closes in the feasibility report

[monarc_tg_integration_feasibility_v1.md](monarc_tg_integration_feasibility_v1.md):
B1 (616-1) closed; B5 identity resolved to `4600-804589` with Fronius TPS/i; open questions
1, 2, 3 and 9 answered; risks R1, R2 and R9 closed; R10 (restore hazard from an option
install) is moot. Still open for the real cell: B4 coordinated frames and the positioner,
section 3's PM registration, R6 SafeMove tool model, R11 PLC bits, and the UAS user.
