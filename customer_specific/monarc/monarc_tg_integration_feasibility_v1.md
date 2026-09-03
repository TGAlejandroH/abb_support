# MONARC retrofit: inserting the TetraGen ABB system into the customer's cell (feasibility v1)

**Question answered:** can the system in `abb/rapid/` (TG_Comms / TG_Main / TG_Cell / TG_Weld,
Phase 6 state) be deployed on the MONARC IRC5 without breaking the customer's existing programs,
how should the PLC "mode toggle" work, and how can their programs call our vision system?

**Sources:** the RobotStudio Pack & Go at `D:\ABB\Monarc_RS\Project` (controller backup
`Controller Data\4600-803651_Virtual`, RobotWare 6.16.0025), read-only. Cell mechanics, I/O map and
the customer-code risk ledger are already in [monarc_cell_teardown.html](monarc_cell_teardown.html)
and are **not repeated here**; this document is only about the insertion. RAPID vocabulary used by the
customer is explained in [monarc_rapid_primer.html](monarc_rapid_primer.html).

**Scope note (generalist product):** everything below separates *what stays generic* (the TG core) from
*what is a per-cell adapter*. MONARC is the first adapter, not the design. Nothing in this folder feeds
the Weld Selector / HMI without authorization (folder README).

---

## 0. Verdict in one page

**Feasible, with one hard blocker and one design gap to close before anything runs on the cell.**

| # | Finding | Kind | What it means |
|---|---|---|---|
| **B1** | **Option `616-1 PC Interface` is not installed** on this controller (`system.xml`, `BACKINFO/backinfo.txt`; the 2026-08-27 site survey found the same on the surveyed controller). On RobotWare 6 the RAPID socket instructions (`SocketCreate/Bind/Listen/Accept/Send/Receive`) belong to that option. Our own prototype VC needed 616-1 to compile `TG_Comms.sys`. | **Blocker** | TG_Comms cannot load, let alone run, until the option is bought and installed (ABB license key + system re-installation via Installation Manager, then restore). Fallback transports exist (section 2.1) but each is a redesign. |
| **B2** | The backup's main program is **Production Manager (option 812-1)**: `gapMain.main()` is the single line `ExecEngine;`. Every customer program is a *part* or a *service menu* that the engine dispatches on a PLC job number. | Architecture | This is good news: PM already has a **PLC-commanded routine mechanism** (`giJobSel` + `diR1MenuOrder`, codes 101-110 used today) and discovers `partdata` / `menudata` declared in *any* loaded module. We can register our entry routine **without editing a single customer file** (section 3). |
| **B3** | Our `TG_Main.mod` declares `PROC main()`. Their task already has `main()` in `gapMain`. Two globals of the same name will not load. | Must fix (ours) | Split TG_Main into a host-agnostic entry `TG_VisionMode()` plus a tiny standalone wrapper that owns `main()` only on greenfield cells. Generic improvement, not MONARC-specific. |
| **B4** | The cell welds in **positioner-coordinated work objects** (`wobj_Stn1`: `ufprog=FALSE, ufmec="STN1"`). Our request PROCs write the served frame into `WObj.uframe`, which is ignored for a coordinated wobj, and report poses in whatever frame the wobj resolves to. | Design gap | Frames must land in **`oframe`** when the wobj is coordinated, and capture poses must be reported **relative to the station plate** (which a coordinated wobj gives for free). We declare **our own** coordinated work objects, one per station, and never write into the customer's. Needs a write rule plus an exporter assertion in TG_Comms (section 2.4). The Weld Planner built and controller-verified the `no_hmi` half of this on 2026-09-01/02; the HMI half is contract open item **O-1**, still open and waiting on us. |
| **B5** | The backup is controller **4600-803651 with a Miller Auto-Axcess E (Miller_EIP)**. The Weld Planner tracker records the *target* as MONARC's **new cell with a Fronius**, and the surveyed controller as **4600-804589**. Three identities, one target. | Open | Every welder-specific mapping in TG_Weld (Fronius arc-length correction, +/-10 clamp) and every calibrated number in this backup is provisional until MONARC confirms which controller and which power source we are integrating with. |

**Recommended insertion (section 3):** a **TG overlay** of six files under `HOME:/TG/` plus two config
fragments, auto-loaded by `SYS.cfg`; the entry routine is exposed to Production Manager as a **service
menu with a PLC command code** (proposed 9001). The PLC selects "TetraGen vision mode" exactly the way it
selects torch cleaning today; the robot serves HMI sessions until the PLC clears a mode input; their
parts keep running unchanged whenever the mode is off. No customer RAPID is edited; removal is
"delete the TG files and the TG config rows".

---

## 1. What we are inserting into (facts that drive the design)

| Item | Value on the backup | Where | Consequence for us |
|---|---|---|---|
| RobotWare | 6.16.00.00 (build 6.16.0025), IRC5 | `BACKINFO/version.xml` | Same RW6 generation as our 6.15.08 validation. Socket API identical across RW6 (our RW5/6 vs RW7 doc). The local VC in the Pack & Go runs **6.15.8029** because 6.16 media is not installed here (teardown, "Blocks work"). |
| Options present | 812-1 Production Manager, 633-4 Arc, 652-1 BullsEye, 657-1 SmarTac, 841-1 EtherNet/IP, 888-3 PROFINET Device, 997-1 PROFIsafe, 1125-2 SafeMove Pro, 608-1 World Zones, 613-1 Collision Detection, 1582-1 IoT Data Gateway, Miller EIP Welder add-in, Positioner package ("Same task for IRB/IRBP") | `system.xml` | Arc is there (TG_Weld loads). Single motion task, no Multitasking: matches our single-task design. |
| Options absent | **616-1 PC Interface**, 623-1 Multitasking, 614-1 FTP/SFTP client | `system.xml` | **B1.** |
| Main program | `gapMain.main()` = `ExecEngine;` | `RAPID/TASK1/PROGMOD/gapMain.mod` | Never edit; hook into PM instead. |
| How the PLC starts work | `GAP_API_COMMANDS`: run part = `siGap_Run_Part_R1` (= `diCycleStart` OR pendant), run menu = `diR1MenuOrder`, number = `giJobSel` (16-bit GI, PROFINET bits 16-31), ack = `goJobSelAck`, feedback = `goMenuCallFbk` | `SYSPAR/PROC.cfg`, `EIO.cfg` | Our "mode toggle" can be a **new command code**, no new I/O required for the trigger itself. |
| Existing PLC menu codes | 101 torch clean ... 110 tool setup (`menudata`, last field) | `SYSMOD/Utility.sys` | We must not collide; propose the **9001-9099 block for TetraGen**. |
| Existing part codes | 111, 112, 131 (templates), 1001-1036, 1500, 1502-1503 | `PartR1S1/2.mod`, `mDeclarations.sys`, `Testing.mod` | Same. |
| Resident modules | Auto-loaded from `HOME:/ApplSys/...` and `FUNCPACK:` by `SYS.cfg -> CAB_TASK_MODULES` | `SYSPAR/SYS.cfg` | Our modules become resident the same way: **add rows, touch nothing else**. |
| Encrypted (unreadable) | `GAP_USER.sys`, `GAP_PARTADV*.sys`, `ProdScr.sys` | `RAPID/TASK1/SYSMOD` | We cannot add hooks inside GAP_USER. Not needed: PM finds `partdata`/`menudata`/`ee_event` declared in ordinary modules (the customer does exactly that in `Utility.sys` and `mDeclarations.sys`). |
| Networks | PROFINET to PLC on LAN3/X5 (`192.168.0.14`, robot is the device, 16 B in / 16 B out); EtherNet/IP private network (`192.168.125.x`) for the Miller welder and the DSQC1030 local I/O; WAN port X6 **not configured in the backup**; COM1 serial channel declared | `SYSPAR/SIO.cfg`, `EIO.cfg` | The vision PC's connection point is the **WAN port**; MONARC IT must assign the IP; `stTG_ServerIP` binds to it. |
| Torch tool | `tWeldGun` (TASK PERS, BullsEye-maintained, live and correct per teardown) | `SYSMOD/BE_User.sys` | .tgs programs pass **`tWeldGun` by name** as the `\Tool` PERS parameter; never copy it. |
| Frames | `wobj_Stn1/2` coordinated (`ufprog FALSE`), `wobj_Stn1/2_NoCoord` fixed | `SYSMOD/wobj_Database.sys` | **B4.** |
| Arc units | `ARC_SYSTEM_PROP -units "US_UNITS"` (pendant shows ipm) but every stored `welddata` value is an exact ipm-to-mm/s conversion (169.333 = 400 ipm, 8.89 = 21 ipm) | `PROC.cfg`, `mDeclarations.sys` | **RAPID stores mm/s regardless of the UI unit setting.** Our `nTG_IpmToMmS` conversion in TG_Weld stays correct on this cell; the header comment in TG_Weld.sys that US_UNITS "would make the conversion the identity" is contradicted by this evidence and must be re-verified before anyone relies on it. |
| Welder | Miller Auto-Axcess E over EtherNet/IP, 8 weld lists, list 4 synergic; UI exposes voltage + wirefeed | `PROC.cfg` | **B5**; TG_Weld's Fronius semantics do not transfer 1:1 (section 2.5). |
| Safety | SafeMove Pro: keep-in `SafetyPerimeter` (0.25 m/s tool speed), `ProductionZone` (4 m/s), `TipChange` keep-out; torch modelled as 3 capsules | `HOME/Safe Move/SafeMove_configuration.xml` | A camera on the torch changes the supervised tool; capture poses must respect the zones (section 2.4). |
| Symbol collisions | No customer symbol carries `TG_`/`tg`/`doTG_` (grep of RAPID + EIO). Only collision: **`main`** | this analysis | **B3.** |

---

## 2. Feasibility by layer

### 2.1 Transport: the socket server needs 616-1 (blocker B1)

Evidence and consequences:

- `system.xml` and `backinfo.txt` list no `616-1 PC Interface`; the curobo tracker's site-survey entry
  (E23, 2026-08-27) says the same for the surveyed controller. Our prototype VC
  (`D:\ABB\ABB-IRB-4600-20-2-50`) carries 616-1 and is where TG_Comms was validated.
- On RW6 the option is a **license key**: adding it means ABB issues a new key, the system is
  re-created with Installation Manager (or "Modify system" on an existing one) and the backup restored.
  That is a **downtime item** and interacts with the teardown's restore hazard (the `HOME:/ApplSys`
  templates auto-loaded by `SYS.cfg` are *not* the production parts). Plan it with ABB service and take
  a fresh full backup first.
- 623-1 Multitasking is also absent. Our design is single-task by decision, so nothing is lost.
- **Robot Web Services** (used for `.tgs` delivery and touch-up retrieval) is base RobotWare-OS per our
  retrieval doc. What still needs checking with ABB for RW 6.16: whether RWS is reachable over the
  **WAN port** without 616-1, or only over the service port. If only the service port, the vision PC
  would have to sit on the `192.168.125.x` private network next to the welder and the local I/O, which
  ABB discourages for non-I/O traffic.

Fallback transports if 616-1 cannot be purchased, assessed against our request protocol (about 10 tiny
exchanges per request, robot-initiated):

| Transport | Needs option? | Fit | Verdict |
|---|---|---|---|
| **Buy 616-1** and keep TG_Comms as is | yes (the missing one) | Exact | **Recommended.** Cheapest engineering path; also unlocks RobotStudio-online and PC SDK for commissioning. |
| RWS "mailbox": PC polls/writes PERS symbols (`/rw/rapid/symbol/data/...`), robot `WaitUntil` on PERS | no (subject to the WAN-port question above) | Inverts the direction (PC-initiated), ~100-300 ms per exchange, needs a transport seam in TG_Comms and a PC-side poller | Viable second transport; worth designing the seam for, since it would also serve cells where sockets are unavailable. Not for v1. |
| Serial `COM1` (`Open "com1:"` + `Write`/`ReadStr`) | no | Byte-compatible with the FANUC wire, RS-232 cable to the PC, 60-byte messages fit | Works on paper; commissioning-grade only (cable length, no isolation, no remote access). Keep as an emergency option. |
| PLC relay (PROFINET group I/O) | no | Frames do not fit in 16 bytes; PLC program becomes our protocol stack | Not viable. |
| 1582-1 IoT Data Gateway (installed) | - | Outbound MQTT/OPC UA publisher, no command path | Not a transport. |

### 2.2 RAPID namespace, residency, files

- **No collisions** except `main` (B3). Fix on our side, generically:
  `TG_Main.mod` exposes `TG_VisionMode()` (loop) and `TG_VisionOnce()` (one HMI session) and drops
  `main()`; a separate `TG_Standalone.mod` (`PROC main() TG_VisionMode; ENDPROC`) is loaded **only** on
  cells where TG owns the program. `tgCycleAbort`'s `ExitCycle` moves the PP to the *task's* `main`, which
  on MONARC is `gapMain.main` (see 2.3).
- **Residency:** add `CAB_TASK_MODULES` rows for `HOME:/TG/TG_Comms.sys`, `TG_Cell.sys`, `TG_Weld.sys`,
  `TG_Main.mod`, `TG_ProdMgr.sys` (section 3). This is the mechanism the customer already uses; it
  survives warm restarts and "load program".
- **Dynamic programs:** `HOME:/TGS/` for `.tgs` modules and `HOME:/TGS/edited/` for touch-up staging,
  unchanged. Production Manager itself loads parts dynamically from `HOME:/DynPart/...@Proc`
  (`DynPartR1S1.mod`), so `Load`/`UnLoad` in this task is proven ground.
- **UAS:** RWS `PUT /fileservice` and `action=save` need a user with write and RAPID-modify grants.
  Create a dedicated `TG` user rather than relying on Default User.

### 2.3 Program flow: how to enter and leave vision mode under Production Manager

PM's `ExecEngine` owns the cycle: it waits for an order (PLC or pendant), fires `ee_event` hooks
(`EE_PRE_PROD`, `EE_POST_PART`, `EE_SERVICE`, `EE_CLOSE_JIG`, `EE_ABORT` ...), indexes the positioner
according to the part's `partadv`, late-binds the part routine by name, and reports state to the PLC
(`soGap_Ready_R1`, `soGap_Running_R1`, `doR1RdyForOrder`, `doJobComplete`). Menu routines are the same
late-bound dispatch without the part choreography; the customer's menus move the robot, activate and
deactivate units and even `Stop;` for the operator (`ServicePos`), so **long-running, motion-capable
routines are normal in that context**.

Four ways to call our system, evaluated:

| Option | Mechanism | Customer edits | Toggle | Assessment |
|---|---|---|---|---|
| **A. TG as a PM part** | `TASK PERS partdata pdTG_Vision:=["TG_VisionOnce","TetraGen vision job","",1,9011,"","padvTG_Stn1"]` in our module. PLC (or Production Screen) orders part 9011; PM indexes station 1, calls `TG_VisionOnce`, which serves **one** HMI session, returns; PM runs `EE_POST_PART` (GoSafe to safe position), counts the part, torch-clean counters advance. | none | one order per job | Cleanest PM semantics (every vision job is a PM cycle, PLC handshake stays coherent). Requires an order per job, which is not the "robot waits for the HMI" behaviour asked for. Keep as the per-job variant. |
| **B. TG as a PM service menu with a PLC command code** | `TASK PERS menudata mdTG_Vision:=["TetraGen vision mode","","TG_VisionMode",255,"T_ROB1",255,TRUE,2,0,FALSE,9001]`. PLC writes 9001 to `giJobSel` and pulses `diR1MenuOrder` (or the operator picks it in the Service menu). `TG_VisionMode` serves HMI sessions **while** a PLC input `diTG_ModeSel` stays high, then returns to PM. | none | PLC input latched | **Recommended.** Identical to how the PLC already commands torch cleaning (code 101). No customer RAPID touched; PM stays the owner of the cycle; their parts run unchanged when the mode is off. |
| C. Branch in `gapMain.main` | `IF diTG_ModeSel=1 THEN TG_VisionMode ELSE ExecEngine ENDIF`; switch by PP-to-main (`diPPToMain` exists) | one line in `gapMain.mod` | PLC input + PP to main | Simple but bypasses PM: the GAP status signals the PLC relies on are not maintained while we run, and the customer's main is now shared code. Fallback only. |
| D. Direct calls from their part routines | e.g. replace the wire-search block of `RS461117_Stn1` by a TG capture set and weld the taught path in the served frame | their part programs | n/a | Possible later for Cartesian-offset parts (mRS461117 style), not for the twist-model parts (mRS462509 rotates the table, not the path). Needs a "frame-only" HMI session type that does not exist today. Not for v1. |

**Details that make B work:**

- **Leaving the mode.** Today `TG_SocketCom` blocks in `SocketAccept ... \Time:=WAIT_MAX`. To notice
  that the PLC dropped `diTG_ModeSel`, accept with a bounded timeout (a PERS, e.g. 5 s) and re-check a
  cell-adapter predicate `tgKeepServing()` on each `ERR_SOCK_TIMEOUT`; between sessions the loop checks
  it too. Generic change: the standalone wrapper's predicate is simply `TRUE`.
- **Status back to the PLC.** One DO `doTG_VisionActive` (free PROFINET bit) set inside
  `TG_VisionMode`; PM's own `goMenuCallFbk` also echoes the running menu.
- **Station handling.** PM does *not* index a station for a menu routine. The `.tgs` program does it
  itself, as the template [mRS999999_Stn1.mod](mRS999999_Stn1.mod) already shows (`IndexToStn1` or
  `IndexStn1Direct`, then `ActStn1`). Two corrections the template still needs: (i) **level both tilt
  axes before any index** (the customer's PLC menu 103 does `ArmsToZero_DPos; IndexToStn1;` and the
  `EE_CLOSE_JIG` hook does the same for parts; the hook does not fire in a menu routine, and
  `ArmsToZero_DPos` ends in `Stop;` so call your own leveling sequence, not theirs); (ii) `IndexToStn1`
  pre-positions the chucks from the PM part queue and warns when it is empty; prefer
  `IndexStn1Direct`.
- **Wire-loss recovery.** `tgCycleAbort` does `ExitCycle`, validated on our VC. Under PM the PP lands in
  `gapMain.main`, `ExecEngine` restarts, and `EE_ABORT` may fire `GoSafeEEv:MoveAbort` (robot moves to
  the safe position). Acceptable ("HMI lost: back to PM idle") but **must be observed on the MONARC VC**
  before we accept it. Alternative to test: have the exporter emit `ERROR RAISE;` in every `.tgs` PROC
  so the error propagates through the late-bound call to `TG_VisionMode`, which can then unload and
  return to PM with `doTG_Error` set (finding F-F only covered *unhandled* errors).
- **PLC work required:** one new command code (9001) on the existing menu-order path, one input bit and
  one output bit (section 3.3). No PROFINET hardware change: the 16-byte frames have spare bits.

### 2.4 Motion and frames on a positioner cell (design gap B4)

#### 2.4a A work object is two stacked frames

`world` then `uframe` then `oframe` then the robtarget's own numbers. `uframe` says where the fixture
or table sits; `oframe` says where the workpiece sits on that fixture; a taught point is measured in
`oframe`. The switch that matters is `ufprog`:

| `ufprog` | Who owns `uframe` | Who owns `oframe` |
|---|---|---|
| `TRUE` (static) | the programmer. The stored value positions the points | the programmer |
| `FALSE` (coordinated) | **RobotWare**, recomputed continuously from the unit named in `ufmec`. The stored value does not position anything | the programmer |

The customer's own data is the proof: `wobj_Stn1` and `wobj_Stn1_NoCoord` carry **identical** `uframe`
and `oframe` and differ only in `ufprog`. The static twin uses the stored `uframe`; the coordinated one
ignores it. FANUC parallel: `UFRAME[n]` is `uframe`, and there is no `oframe` counterpart, which is why
the port writes `uframe` today.

Consequence, and it is the dangerous kind: on a coordinated work object our served frame is accepted,
stored, and **silently ignored**. The robot welds the uncorrected path with no error.

#### 2.4b We declare our own coordinated work objects, one per station

Do **not** pass the customer's `wobj_Stn1` into a request PROC: the PROC writes into it, and a
customer-owned record is exactly what the `TG` naming convention exists to keep us out of. Declare our
own with the same mechanical unit instead. `ufmec` is baked into the record, so a single work object
cannot serve both stations, and decision **D4** already gives one `.tgs` template per station:

```rapid
! TG_Frames.sys - CELL ADAPTER. Station names are cell configuration, so these
! live in the adapter, not in the generic core. uframe stays identity: with
! ufprog:=FALSE the positioner defines it and the stored value is never used.
PERS wobjdata wobjTG_Stn1:=[FALSE,FALSE,"STN1",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
PERS wobjdata wobjTG_Stn2:=[FALSE,FALSE,"STN2",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
```

`wobjTG_Weld` / `wobjTG_Cam` in `TG_Comms.sys` stay static and unchanged; they remain the default for
cells with no positioner. The exporter names the record per station through the output profile, which
`AbbTranslator.configure_controller_data(wobj_name=...)` already supports.

**Who writes what, and when.** The resident record states the *kind*: `robhold`, `ufprog` and `ufmec`
are cell configuration and never change. The generated module states the *frame*, assigning the
nominal part frame at program entry ahead of any motion, and the runtime vision write then overwrites
the same slot. Two writers, one slot, in that order. The module must assign unconditionally on every
run, because a resident record keeps whatever the previous program left in it. That is finding F-2's
stale-frame hazard in a new place.

**Because we start from identity, no composition with their taught fixture frame is needed.** Our
`oframe` is the whole plate-to-part transform, which is exactly what vision measures. This corrects a
looser statement made earlier: composition would only be required if we adopted their record, whose
`oframe` is their fixture position on the plate.

**The verification that replaces it** is frame coincidence: our bundle's positioner end link, where the
workpiece attaches in the `.tgs` URDF, must sit on the same plate face that RobotWare uses as the
station user frame. The kinematics extraction found the plate axis origin **is** the station flange, so
they should agree, but any residual is a fixed offset that must then live in exactly one place, either
the URDF or a constant pre-multiplied into `oframe`. Measure it: command a known table angle, compare
the reported pose against the predicted one.

#### 2.4c The write rule, and an assertion the exporter supplies

The rule belongs in TG_Comms and applies to `TG_ReqCamFrame` and `TG_ReqWeldFrame` alike. **The data
decides**, because the record is the only thing that can be right. An exporter argument that *overrode*
the data could write `uframe` on a coordinated work object, which is the silent failure above. The slot
is chosen once per work object and then used twice, by the exporter at entry and by the request at
runtime, so both sides have to reach the same answer from the same fact.

An optional argument is still worth adding, as an **assertion** rather than a decision. Recommended
form is the mechanical-unit name rather than a fixed/coordinated switch, because it also catches a
station-1 program running against station 2 - a live hazard on a cell where both stations share
`extax` slots:

```rapid
! \Mec is the exporter's stated expectation, never the decision:
!   "STN1"  expect coordinated on that unit    ""  expect static    omitted  no check
bKind:=TRUE;
IF Present(Mec) bKind:=tgWobjMatches(WObj,Mec);
...                                   ! the wire exchange ALWAYS runs to the end -
                                      ! cutting it short desyncs the HMI
IF WObj.ufprog THEN
    WObj.uframe:=pFrame;              ! static: FANUC UFRAME[n] parity
ELSE
    WObj.oframe:=pFrame;              ! coordinated: uframe belongs to the positioner
ENDIF
IF NOT (frame_ok AND bKind) nTG_WeldStatus:=2;    ! existing abort path, matrix I4
```

A mismatch takes the same route as a malformed frame payload: complete the exchange, then abort the
program rather than skip the weld. What it buys is worth the argument: since points are emitted
divided by that work object, a record of the wrong kind means every point in the program is wrong by
the station transform, and approach and retract motion still look perfectly normal.

#### 2.4d Which frame the HMI sees

With a coordinated work object, `CRobT` returns the TCP **relative to the plate** at any tilt or
rotation. That is the frame captures must be reported in here, because a base-frame pose is meaningless
if the table moves between captures, and it is the frame the HMI must serve back. The HMI's
registration assumes a fixed base; on a positioner cell that base is the plate frame. **Cross-team
item** for HMI and Weld Planner: confirm the registration math holds with the plate as base.

#### 2.4e Status in the Weld Planner repo (re-read 2026-09-02, `develop` at `aea9efa`)

This moved a long way while the first draft of this report was being written. **Half the carrier is
built and controller-verified.** The half MONARC needs is still open and still waiting on our side.

| Item | Where | State |
|---|---|---|
| Corrections go to `oframe` on a coordinated work object | evidence **E28** | recorded |
| Points emitted relative to a declared work object | **D26** | **built, default on** for `no_hmi`, landed 2026-09-01 |
| `LOCAL PERS wobjdata` accepted as a `\WObj` argument | evidence **E50** | **controller-verified** 2026-09-02: module loaded and program-checked clean on an IRB 4600 virtual controller. Not simulated, so motion is not re-validated |
| `oframe` left identity as the landing site for the vision write | translator source | already deliberate there, and matches 2.4b |
| HMI-mode carrier: which record, which slot, who assigns it at entry | contract **O-1**, Phase 7 | **open**, owner joint, marked "Needs Alejandro", **blocks Phase 8** |
| `ufprog:=FALSE` plus `ufmec`, the `oframe` correction, `ActUnit`, station-2 template | **Phase 8** | all unchecked |
| Our MONARC bundle adopted as the ABB dev and test input | **D27**, 2026-09-02 | authorized by Bhavin, with a note to coordinate over this folder's README |
| Our `joint_limits.md` confirms the station slots and moves the index axis to `eax_d` | evidence **E51** | confirmed, no output change |

What the translator emits today in `no_hmi` mode: one `LOCAL PERS wobjdata wobjTG00n` per distinct
frame, shaped `[FALSE,TRUE,"",<the frame the points were divided by>,<identity>]`, with `\WObj:=` on
every motion and `wobj0` kept for home and transition moves, which have no part to be relative to. So
a **static** cell is now correctable end to end. A **coordinated** one is not, because nothing flips
`ufprog`/`ufmec` yet and nothing assigns the frame into `oframe` instead of `uframe`. That is O-1,
and 2.4b and 2.4c above are our half of the answer to it.

⚠ **Read evidence E49 before the first MONARC frame test.** The program frame was never auto-applied
on the ABB export path at all, so in an inlined weld family the frame of the first weld stayed active
for every later move. Measured on one project: of 66 moves, 61 were bound to the part's work object
where 30 should have been, the return home and the fixture-clearing transitions among them. Correcting
that work object at runtime, which is the entire reason it exists, would have moved the home position
along with the part. Fixed on 2026-09-01 and guarded at compiler level, and neither of their
validation projects could have shown it. On this cell the same defect would drive the torch toward a
positioner that has just been re-indexed.
- **One station at a time, drives shared.** `STN1`/`STN2`/`INTERCH` are separate mechanical units with
  `-activate_at_start_up FALSE` on one shared drive group (teardown). Every `.tgs` program must
  `ActUnit` the station it welds, `DeactUnit` it before any service motion (torch clean, wire cut) and
  before returning to PM, and never have two stations active. `9E9` in the unused `extax` slots is
  legal only while that unit is deactivated (tracker D24/D25 already encode this).
- **Configuration data.** Customer targets carry valid `confdata` with `ConfL\On`. Our demo programs
  relax `ConfJ/ConfL` for the VC only; exporter output must carry proper `confdata`.
- **SafeMove and world zones.** Capture poses must stay inside `SafetyPerimeter` and outside
  `TipChange`; outside `ProductionZone` the tool is limited to 0.25 m/s, so approach moves toward the
  reamer/service end are slow by design. **Mounting a camera on the torch changes the supervised tool
  geometry** (three capsules today) and the TCP the safety controller supervises (already 25 mm off
  the RAPID TCP per teardown): the safety configuration must be updated and re-sealed by a safety
  engineer, and `tWeldGun`'s load data updated for Collision Detection. This is a plant-level change,
  not a RAPID one.
- **Cuboid world zones** (`WZDOSet \Stat`) only set DOs; they do not block motion.

### 2.5 Welding data (B5)

- **Equipment class differs from our design target.** This backup: `Miller_EIP` (`awEqMillerEIP`), 8
  weld lists (`MILLER_WELD_SCHED`), list 4 synergic. Customer `welddata` reads
  `[weld_speed, 0, [sched, mode, voltage, wirefeed, ...], ...]` with `sched` = Miller weld list number,
  `voltage` = 40 on the synergic list (an arc-length/trim value) and 24.5 on list 1 (real volts). So the
  meaning of `main_arc.voltage` depends on the selected list. TG_Weld's mapping (HMI "Arc Length" ->
  Fronius arc-length correction, clamped to +/-10) is **Fronius-specific**; on Miller the same HMI field
  maps to the list's trim/voltage with different limits. Make the clamp limits and the field-to-component
  map **cell-adapter data** (PERS), not constants. The Weld Planner has since locked **D29**: the arc
  runs the Fronius in Job mode, the exporter drives only the job selection, and the correction
  components stay neutral. That is the right shape for a Fronius and the wrong one for the Miller in
  this backup, where the same field is a Miller weld list number. Another reason B5 has to be settled
  before either side commits.
- **Recipe library.** `wdTG_Lib{n}` must be seeded from the customer's own `welddata`/`seamdata`
  (`wd3_16_ft_tube`, `wd3_16_ft_block_095`, ...) by procedure number, agreed with MONARC. Never re-type
  the numbers (teardown: values are exact ipm conversions; a rounded copy silently changes the weld).
- **Units, and a live contradiction with the Weld Planner.** The customer's own weld data says RAPID
  holds **mm/s** on this cell even though the arc system is configured `US_UNITS`. Twelve stored
  values are exact conversions of round pendant entries: `8.89` for a 21 ipm travel speed, `169.333`
  for a 400 ipm wire feed. The reading that follows is that `ARC_UNITS` governs entry and display,
  not the stored field. **Decision D28 in the Weld Planner concluded the opposite on 2026-09-02** and
  now converts `welddata.weld_speed` to ipm by default, reasoning from the same `PROC.cfg` line
  without checking what the cell actually stores. Both cannot be true, and the gap is a factor of
  2.36 on weld travel speed. Nobody has measured it under `US_UNITS`, `TGArcCheck` step 2 is exactly
  that measurement, and D28 moves the validation VC to `US_UNITS` anyway, so the test is one run
  away. Set `weld_speed:=8.89`, weld the 300 mm reference line, time it:

  | Measured | The field is | Consequence |
  |---|---|---|
  | about 34 s | mm/s | D28 over-converts and exported welds run 2.36x fast |
  | about 80 s | ipm | D28 is right and TG_Weld must stop converting on a `US_UNITS` cell |

  Until that is run, do not seed the recipe library from converted numbers.
- **Dry run.** `TG_DryRunOn/Off` are placeholders. On this cell the Miller add-in has `autoinhib_on` and
  the Production Screen exposes manual arc operations; the RobotWare Arc "blocking" mechanism to use
  programmatically is still to be identified.
- **Arc error dialogue is on the PLC/HMI.** `ARC_ERR_HNDL_IO` routes weld faults to the cell HMI via
  PROFINET group signals. A weld fault inside a `.tgs` weld will wait for the operator's answer on the
  *cell* HMI while our HMI is blocked waiting for the next request. That is safe (our side just waits)
  but the two HMIs must not confuse the operator; document it in the operating procedure.
- **Welder identity.** If the target really is a Fronius cell (tracker), the Fronius mapping in TG_Weld
  is the right starting point and the Miller notes above are for the test asset only. Resolve B5 first.

### 2.6 Cell I/O choreography (per-cell adapter: TG_Cell.sys)

Every MONARC part routine performs the same cell handshake (teardown, "One part, end to end"):
clear outputs, wire feed 0.3 s + cut + `do5_WireBrake`, ask the PLC to clamp
(`doOK_Unclamp` -> wait `diReadyToResumeProg`), weld, `doAllWeldsComplete`, deactivate, safe position.
A `.tgs` program that wants to be a drop-in on this cell must do the same, but the `.tgs` must stay
brand- and cell-neutral. Resolution, consistent with how TG_Cell already ports the FANUC utilities:

- `.tgs` programs call neutral hooks that already exist or are added once: `TG_WeldPrep` (before the
  first weld), a new `TG_PartReady` (after captures, before welding) and `TG_PartDone` (after the last
  weld), `TG_CamOpen/Close`.
- **MONARC's** `TG_Cell.sys` implements them with the customer's signals: `TG_PartReady` =
  `SetDO doOK_Unclamp,1; WaitDI diReadyToResumeProg,1; SetDO doOK_Unclamp,0`;
  `TG_PartDone` = `SetDO doAllWeldsComplete,1`; `TG_WeldPrep` = wire trim if MONARC wants it before
  vision-guided welds (not needed for touch sensing any more, still useful for stick-out). Exact sequence
  to be agreed with MONARC and their PLC integrator.
- `doTG_Camera` (camera flap) maps to a **spare DSQC1030 output**: bits 5-15 are unused placeholders
  (`Local_IO_0_DO6`...). Overlapping two signal names on one bit is legal on this controller (their own
  config does it on PROFINET bits 32-36) but confusing; ask MONARC to retire the placeholder name.

### 2.7 File transfer and touch-up retrieval

- RWS `PUT /fileservice/$HOME/TGS/<prog>.mod` and the staged-edit retrieval work as designed, subject
  to the WAN-port question in 2.1 and a `TG` UAS user.
- `HOTEDIT_MODPOS -tuning_in_auto` is enabled on this cell (teardown): operators tune positions in
  AUTO, which our `UnLoad \ErrIfChanged` staging catches. A PLC `diPPToMain` while a `.tgs` module is
  loaded drops the dynamic module and any unsaved touch-up (finding F-4 class); state this in the
  operating procedure.

### 2.8 Vision replaces touch sensing

The customer locates parts with the welding wire (`SearchC`/`SearchL`, SmarTac through the Miller) and
either offsets the path (`Offs`) or **re-indexes the turntable** to the measured angle, with a hand-coded
twist model (teardown, "Finding the part"). Our system replaces the measurement with captures and a
served frame. Two consequences: the touch-sense request family stays out of scope for MONARC, and the
served frame is a rigid transform, so parts that today rely on the twist model need the Weld Planner to
plan per-weld corrections rather than one frame per part. Flag to the Weld Planner track.

---

## 3. Recommended insertion design

### 3.1 The TG overlay (what gets installed)

```
HOME:/TG/TG_Comms.sys      generic core (request protocol, PERS state)      resident
HOME:/TG/TG_Main.mod       generic core: TG_VisionMode / TG_VisionOnce      resident
HOME:/TG/TG_Weld.sys       generic core + cell-adapter PERS (units, clamps,
                           recipe library seeded from MONARC welddata)      resident
HOME:/TG/TG_Cell.sys       CELL ADAPTER: MONARC I/O macros (2.6)           resident
HOME:/TG/TG_Frames.sys     CELL ADAPTER: coordinated work objects (2.4b)   resident
HOME:/TG/TG_ProdMgr.sys    CELL ADAPTER: Production Manager binding        resident
HOME:/TG/TG_SYS.cfg        config fragment: CAB_TASK_MODULES rows above
HOME:/TG/TG_EIO.cfg        config fragment: doTG_Camera, diTG_ModeSel, doTG_VisionActive
HOME:/TGS/                 dynamic .tgs modules (+ edited/ for staging)
```

Not installed on MONARC: `TG_Standalone.mod` (owns `main()` on greenfield cells only),
`TGArcCheck.mod`, the `TGS/TD05*.mod` demos.

`TG_ProdMgr.sys` (sketch, field order per the `menudata` descriptor: description, image, procName,
validStn, taskList, validPos, allowAfterError, type 2 = service, minUserLevel, blockOtherTasks, plcCode):

```rapid
MODULE TG_ProdMgr(SYSMODULE)
    ! Production Manager binding for the TetraGen vision system (MONARC cell adapter).
    ! PLC: giJobSel := 9001, pulse diR1MenuOrder  ->  ExecEngine calls TG_VisionMode.
    ! Operator: Production Screen -> Service -> "TetraGen vision mode".
    TASK PERS menudata mdTG_Vision:=["TetraGen vision mode","","TG_VisionMode",255,"T_ROB1",255,TRUE,2,0,FALSE,9001];
    ! Optional per-job variant (option A): one PM part order = one HMI session.
    ! TASK PERS partdata pdTG_VisionS1:=["TG_VisionOnce","TetraGen vision job stn 1","",1,9011,"","padvTG_Stn1"];
ENDMODULE
```

`TG_Main.mod` entry (sketch of the generic change; not yet written or validated):

```rapid
PROC TG_VisionMode()
    ! Host-agnostic entry. Serves HMI sessions while the cell adapter says so.
    tg_module_loaded:=FALSE;
    TG_SocketDisc;
    SetDO doTG_VisionActive,1;
    WHILE tgKeepServing() DO          ! MONARC adapter: diTG_ModeSel=1 ; standalone: TRUE
        tgMainCycle;                  ! accept (bounded timeout) -> prog sel -> transfer -> run -> disc
    ENDWHILE
    TG_SocketDisc;
    SetDO doTG_VisionActive,0;
ENDPROC
```

`TG_SYS.cfg` fragment (load with "Load parameters if no duplicates"; adds rows, replaces nothing):

```
SYS:CFG_1.0:6:0::
#
CAB_TASK_MODULES:

      -File "HOME:/TG/TG_Comms.sys" -ModName "TG_Comms" -Task "T_ROB1"
      -File "HOME:/TG/TG_Cell.sys" -ModName "TG_Cell" -Task "T_ROB1"
      -File "HOME:/TG/TG_Frames.sys" -ModName "TG_Frames" -Task "T_ROB1"
      -File "HOME:/TG/TG_Weld.sys" -ModName "TG_Weld" -Task "T_ROB1"
      -File "HOME:/TG/TG_Main.mod" -ModName "TG_Main" -Task "T_ROB1"
      -File "HOME:/TG/TG_ProdMgr.sys" -ModName "TG_ProdMgr" -Task "T_ROB1"
```

`TG_EIO.cfg` fragment (bit numbers from the free ranges in Appendix B; final numbers with the PLC
integrator):

```
EIO:CFG_1.0:6:1::
#
EIO_SIGNAL:

      -Name "doTG_Camera" -SignalType "DO" -Device "Local_IO" -DeviceMap "5" -Label "TG camera flap" -Category "TetraGen"
      -Name "diTG_ModeSel" -SignalType "DI" -Device "PN_Internal_Device" -DeviceMap "66" -Label "TG vision mode select (PLC)" -Category "TetraGen"
      -Name "doTG_VisionActive" -SignalType "DO" -Device "PN_Internal_Device" -DeviceMap "77" -Label "TG vision mode active" -Category "TetraGen"
```

### 3.2 Install / verify / remove

1. Resolve B1 and B5 (option purchase, target identity). Full controller backup.
2. Copy `HOME:/TG/` and create `HOME:/TGS/`. Load `TG_EIO.cfg` then `TG_SYS.cfg` (add-only). Warm start.
3. Program check: zero errors; `TG_*` modules listed as resident; `mdTG_Vision` appears in the
   Production Screen Service menu.
4. Set `stTG_ServerIP` to the WAN IP, `nTG_Port` (2000), unit/clamp PERS, recipe seeds. Create the
   `TG` UAS user.
5. Dry test without the vision PC: PLC issues 9001 with `diTG_ModeSel=1`; operator window shows
   `TG: waiting for HMI on port 2000`; drop `diTG_ModeSel`; robot returns to PM within the accept
   timeout and `doTG_VisionActive` falls. Their part programs still run when ordered.
6. Full test with the HMI/Python stand-in, per the validation matrix in section 6.

Removal: delete the six `TG_*` rows from `CAB_TASK_MODULES`, the three `TG` signals, and
`HOME:/TG`, `HOME:/TGS`. Nothing of the customer's changes, which is exactly what the `TG` naming
convention promises ([tg_naming_convention.md](../../docs/tg_naming_convention.md)).

### 3.3 What the PLC integrator has to add

- Command **9001** on the existing menu-order path (`giJobSel` + `diR1MenuOrder` pulse), same as 101-110.
- One output to the robot, `diTG_ModeSel` (latched "stay in vision mode"), one input from the robot,
  `doTG_VisionActive`. Both inside the existing 16-byte PROFINET frames (Appendix B), so no GSDML or
  hardware change; PLC program mapping only.
- Optional: a "TetraGen fault" input if we add `doTG_Error`.
- Interlock rule the PLC should keep: do not issue a part order while `doTG_VisionActive` is high
  (PM will refuse anyway, since a menu routine is running, but the PLC should not queue one).

### 3.4 Generic core vs cell adapter (the boundary that keeps the product generalist)

| Generic core (ships identical to every ABB cell) | Cell adapter (authored per cell, all `TG`-prefixed) |
|---|---|
| `TG_Comms.sys`: protocol, PERS register map, the `uframe`/`oframe` write rule and the `\Mec` assertion (2.4c) | `TG_Cell.sys`: I/O macros mapping the neutral hooks to the cell's signals and handshakes |
| `wobjTG_Weld` / `wobjTG_Cam`: static work objects, the default for cells with no positioner | `TG_Frames.sys`: the coordinated work objects, one per station, naming that cell's mechanical units |
| `TG_Main.mod`: `TG_VisionMode/Once`, load/unload, staging; `tgKeepServing()` as an overridable predicate | `TG_ProdMgr.sys` (PM cells) or `TG_Standalone.mod` (TG owns `main`) or a one-line branch in the host main (option C) |
| `TG_Weld.sys`: `TG_ApplyWeldParams`, library mechanics | Unit scale, clamp limits, field-to-component map, recipe seeds; the `.tgs` template's tool/wobj names (`tWeldGun`, coordinated station wobj) |
| Python/C++ HMI side: codec, RWS client, retrieval | `TG_SYS.cfg`, `TG_EIO.cfg`, PLC command code block 9001-9099, network parameters |

---

## 4. Risk register

| ID | Risk | Severity | Evidence | Mitigation / owner |
|---|---|---|---|---|
| R1 | 616-1 absent: no sockets | **Critical** | `system.xml`; tracker E23; our VC needs it | Purchase + ABB install (2.1). Design a transport seam so an RWS mailbox can follow if procurement fails. |
| R2 | Wrong target: backup is Miller 4600-803651; tracker says new Fronius cell; survey saw 4600-804589 | **High** | tracker D1/D16, 2026-09-01 log; `PROC.cfg` | Get MONARC to name the target controller and send *its* backup; treat every number here as a proxy until then. |
| R3 | `main` collision blocks loading our modules | Medium (easy) | `gapMain.mod` vs `TG_Main.mod` | Restructure TG_Main (B3). Generic. |
| R4 | Frames written to `uframe` are ignored on coordinated wobjs; base-frame poses meaningless when the table moves | **High** | `wobj_Database.sys`; TG_Comms request PROCs | Write rule plus `\Mec` assertion in TG_Comms; our own coordinated work objects; plate-frame reporting; HMI/planner confirmation (2.4). |
| R4b | No HMI-mode work-object carrier and no coordinated declaration yet, so a MONARC frame test can prove the wire but not the geometry | **High** | contract **O-1** open; Phase 8 unchecked; the `no_hmi` carrier is built and VC-verified (**E50**) | Close O-1 with 2.4b and 2.4c, then Phase 8 flips `ufprog`/`ufmec` and moves the entry assignment to `oframe` (2.4e). |
| R4c | Our URDF plate frame and the controller's station user frame may not coincide, leaving a fixed offset in every corrected weld | Medium | kinematics extraction vs `MOC.cfg`; unverified | Measure at a known table angle; put any residual in one place only (2.4b). |
| R5 | `ExitCycle` recovery restarts PM and may auto-move the robot to safe (EE_ABORT) | Medium | `GoSafeEEv.sys`, `tgCycleAbort` | Observe on the MONARC VC; test `ERROR RAISE;` propagation alternative. |
| R6 | Camera on the torch invalidates the SafeMove tool model/TCP and Collision Detection load; capture poses vs zones | **High** (safety) | `SafeMove_configuration.xml`; teardown TCP mismatch | Safety engineer updates and re-seals the configuration; planner world model carries the zones; `tWeldGun` load data updated by MONARC. |
| R7 | Station activation errors: two stations active, `9E9` on an active unit, tilted arms during index | **High** (motion) | `MOC.cfg`; `Utility.sys` menus 103/104 | Template rules in 2.4; add arm leveling before index to the `.tgs` template; VC test. |
| R8 | Weld data mapping wrong for the actual welder; clamp +/-10 is Fronius-only; unit assumption in TG_Weld header contradicted | Medium | `PROC.cfg`, `mDeclarations.sys` | Cell-adapter PERS for map/limits; seed library from customer data; re-measure units on the MONARC VC. |
| R9 | Vision PC network path unknown (WAN IP unconfigured; RWS-over-WAN vs 616-1 unclear) | Medium | `SIO.cfg` | MONARC IT assigns the WAN IP; ask ABB about RWS reachability on RW 6.16 without 616-1. |
| R10 | Restore hazard when the system is re-installed for the option: `HOME:/ApplSys` templates vs the saved program | Medium | teardown "Restore hazard" | Establish the authoritative program before the option install; backup before/after; our rows are add-only. |
| R11 | PLC changes depend on MONARC's integrator (code 9001, two bits, interlock) | Medium | `EIO.cfg`, `PROC.cfg` | Section 3.3 is the spec; agree early. |
| R12 | VC fidelity: local VC is 6.15.8029 not 6.16.0025; no SafeMove config in the VC; no welder | Medium | teardown risk ledger; `option_registry.xml` | Install 6.16.0025 media; import the safety config for visualization; use the Simulated Welder or `autoinhib`. |
| R13 | Encrypted GAP modules: no hooks in `GAP_USER.sys` | Low | `RAPID/TASK1/SYSMOD` | Not needed: declarations in our own module are discovered (customer precedent). |
| R14 | PLC `diPPToMain` during a `.tgs` run drops the dynamic module and any unsaved touch-up | Low | F-4; `EIO.cfg` SYSSIG_IN | Operating procedure; PLC should not PP-to-main while `doTG_VisionActive`. |
| R15 | Two HMIs during a weld fault (cell HMI arc dialogue vs our HMI) | Low | `ARC_ERR_HNDL_IO` | Our side simply waits; document for operators. |
| R16 | Torch-clean/service counters (`EE_SERVICE`) never fire in menu mode; torch maintenance during long vision sessions | Low | `Utility.sys ServiceFlag` | Option A variant counts jobs as parts; or the `.tgs` calls a cell hook for torch cleaning on a counter. |

---

## 5. Open questions

For MONARC:
1. Which controller is the target (4600-803651, 4600-804589, other), old or new cell, and which power
   source? Please send that controller's backup.
2. Will MONARC purchase 616-1 PC Interface, and when can ABB install it (downtime window)?
3. WAN port IP and network policy for the vision PC.
4. Who owns the PLC program; can they add command 9001 and two bits (3.3)?
5. Camera mounting: who updates and re-seals the SafeMove configuration and the torch load data?
6. Mode semantics: latched mode (option B) or one order per job (option A)?
7. Exact clamp/PLC handshake we must reproduce in `TG_Cell` (2.6), and whether wire trim is wanted
   before vision-guided welds.
8. Procedure numbers to weld recipes: which existing `welddata`/`seamdata` pairs, and the meaning of
   `voltage` on each Miller weld list (if Miller is the target).

For ABB:
9. Is RWS reachable over the WAN port on RW 6.16 without 616-1?
10. Programmatic weld inhibit ("dry run") mechanism available with the installed Arc equipment class.

Internal (HMI / Weld Planner):
11. Plate-frame poses and `oframe` write-through on positioner cells (2.4): does the HMI's
    registration hold with the station plate as base, and does the planner serve frames in that frame?
11b. Does the bundle's positioner end link coincide with the controller's station user frame, and if
    not, where does the residual offset live (2.4b)?
11c. Weld Planner sequencing: the HMI-mode carrier and Phase 8's coordinated `wobjdata` are the gate
    for every frame item here. Does the profile name the work object per station (2.4b), does the
    module assign the nominal frame into `oframe` at entry, and is `\Mec` accepted as the
    exporter-side assertion (2.4c)?
12. Twist-model parts (2.8): per-weld corrections vs one frame per part.

---

## 6. Next steps (ordered)

1. **Gate:** answers to questions 1-2. Nothing below depends on them except the final on-cell test, so
   work proceeds in parallel.
2. **MONARC VC testbed:** install RobotWare 6.16.0025 + Positioner 6.16 media; modify the Pack & Go VC to
   add 616-1 (free on a VC) and, if the target is Fronius, the Fronius add-in; import the SafeMove
   configuration for visualization. This is where every RAPID item below is validated.
3. **Generic core changes** (repo `abb/rapid/`, with tests in `hmi_prototype/`): TG_Main split
   (`TG_VisionMode/Once`, no `main`), bounded accept + `tgKeepServing()`, the `uframe`/`oframe` write
   rule and the `\Mec` assertion on both frame requests (2.4c), cell-adapter PERS in TG_Weld, neutral
   hooks `TG_PartReady/TG_PartDone` in TG_Cell, standalone wrapper.
4. **MONARC adapter** (this folder): `TG_Cell.sys` (MONARC I/O), `TG_Frames.sys` (coordinated station
   work objects), `TG_ProdMgr.sys`, `TG_SYS.cfg`, `TG_EIO.cfg`, recipe seeds; update the `.tgs`
   template (arm leveling before index, `tWeldGun` by name, `wobjTG_Stn1` in every motion).
4b. **Weld Planner side, in parallel and on the critical path** (`curobo_suite`): close contract
   **O-1** with 2.4b and 2.4c, then Phase 8 flips the declaration to `ufprog:=FALSE` with the station
   as `ufmec` and moves the entry assignment from `uframe` to `oframe`. The `no_hmi` carrier and the
   `LOCAL PERS` question are already done (**D26**, **E50**), so what remains is the HMI-mode record
   and the per-station name in the output profile. Until that lands, a frame test on the MONARC
   virtual controller can only prove the wire, not the geometry.
5. **VC validation matrix** (each with expected operator-window output and a pass criterion, in the
   usual delivery style):
   - T1 overlay loads on the MONARC VC with zero program-check errors; `mdTG_Vision` visible in the
     Production Screen.
   - T2 PLC command 9001 (I/O simulator: `giJobSel`, `diR1MenuOrder`) enters `TG_VisionMode`; dropping
     `diTG_ModeSel` returns to PM within the accept timeout; a customer part order runs afterwards.
   - T3 full HMI cycle from `TG_VisionMode` with `TD05Test`-style program adapted to `tWeldGun` and a
     coordinated station wobj; served frame lands in `oframe`; reported capture pose equals the
     hand-computed plate-relative pose (numerical check, as in F-2).
   - T4 Arc program on station 1 with `IndexStn1Direct` after arm leveling; `ActStn1`/`DeactStn1`
     bracket; welds run at the library `weld_speed` (TGArcCheck-style timing) under `US_UNITS`.
   - T5 wire-loss during a `.tgs` run: observe PM behaviour after `ExitCycle`; decide R5.
   - T6 touch-up staging and RWS retrieval unchanged under PM.
6. **PLC and safety coordination** with MONARC (3.3, R6) in parallel with 2-5.

---

## Appendix A. Evidence index (all read-only, under `D:\ABB\Monarc_RS\Project\`)

- `Controller Data\4600-803651_Virtual\system.xml`, `BACKINFO\backinfo.txt`, `BACKINFO\version.xml`:
  RobotWare 6.16.0025, option list (no 616-1, no 623-1).
- `SYSPAR\SYS.cfg`: `CAB_TASK_MODULES` auto-load rows, `CAB_EXEC_HOOKS`, `NoOfRetry 50`, single task.
- `SYSPAR\EIO.cfg`: PROFINET device map (16 B each way), `SYSSIG_IN/OUT`, cross-connections,
  `Local_IO` spares, `B_GAP_SIM`.
- `SYSPAR\PROC.cfg`: `GAP_API_COMMANDS`/`GAP_API_STATE`, `ARC_SYSTEM_PROP US_UNITS`, `Miller_EIP`,
  `MILLER_WELD_SCHED`, `ARC_ERR_HNDL_IO`, SmarTac profiles.
- `SYSPAR\SIO.cfg`: PROFINET IP on LAN3, COM1, no WAN IP.
- `SYSPAR\MOC.cfg`: `STN1/STN2/INTERCH` units, `-activate_at_start_up FALSE`, activation relays.
- `RAPID\TASK1\PROGMOD\gapMain.mod` (`main` = `ExecEngine`), `PartR1S1/2.mod`, `DynPartR1S1/2.mod`,
  `mRS461117.mod`, `mRS462509.mod`, `Testing.mod`, `WeldTest.mod`, `mLogging.mod`.
- `RAPID\TASK1\SYSMOD\Utility.sys` (menudata with PLC codes 101-110, ee_events, world zones,
  `ArmsToZero_DPos`), `mDeclarations.sys` (partdata 1001-1036, weld data), `BE_User.sys` (`tWeldGun`),
  `wobj_Database.sys`, `Irbp1Data.sys`, `IrbpSetup.sys`, `TorchServices.sys`, `m2ndLoadPos.sys`,
  `GoSafe*.sys` (EE_ABORT -> move to safe), `user.sys`; `GAP_USER.sys` confirmed encrypted.
- `HOME\ProdScr\*.xml`: Production Screen apps; `HOME\Safe Move\SafeMove_configuration.xml`: tool model
  and zones.
- `Virtual Controllers\4600-803651_Virtual\INTERNAL\option_registry.xml`: the local VC runs
  RobotWare 6.15.8029 media.
- Our side: `abb/rapid/*.sys|.mod`, `docs/abb_port_plan_v1.md`, `docs/rapid_validation_findings_v1.md`,
  `docs/abb_program_touchup_and_retrieval_v1.md`, `docs/abb_weld_motion_and_data_design_v1.md`;
  the Weld Planner tracker `curobo_suite/docs/abb_integration_plan_v1.md` and
  `abb_hmi_request_contract_v1.md`, re-read 2026-09-02 at `develop` / `aea9efa` (D1, D3, D4, D16,
  D24 to D30, E23, E28, E46, E48 to E51, contract open item O-1).

## Appendix B. Spare bits on the PLC PROFINET device (from `EIO.cfg`, `PN_Internal_Device`)

| Direction | Free DeviceMap bits (0-127) |
|---|---|
| PLC -> robot (DI/GI) | 13-15, 52-57, **66-107**, 112-127 |
| Robot -> PLC (DO/GO) | 49, 65-69, 71-72, **77-99**, 101-106 |

Pre-existing overlaps in the customer map (for awareness, not ours to fix): inputs 35, 36, 38, 39 carry
both `giTableAngle` and `diR2/R3 WireFeed/GasPurge`; outputs 32-36 carry both `goMenuCallFbk` and
`doBE_Calib`, `doOK_Unclamp`, `doOK_Clamp`, `soAtSTN1/2_2ndPos`.

Spare local I/O (DSQC1030 `Local_IO`): outputs 5-15 and inputs 4-15 are unused placeholders
(`Local_IO_0_DO6`..`DO16`, `Local_IO_0_DI5`..`DI16`).
