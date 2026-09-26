# Touch Sensing (Auto Touch-Ups): FANUC to ABB Port, Research and Plan (v1)

Status:
- **P0 DONE** (VC experiments, 27/27 PASS, 2026-09-25).
- **P1 DONE** (ids 16/18/19 in `TG_Comms.sys` plus the prototype HMI; 13/13 VC checks
  PASS, 2026-09-26).
- **P2 DONE** (`TG_TouchSearch` in the new `TG_Touch.sys`, plus the cell macros; 36/36
  VC checks PASS, 2026-09-26).
- **P3 DONE** (`TD05Touch.mod`, real searches through the real TG cycle; 19/19 VC checks
  PASS, 2026-09-26).
- **Next:**
  - P4 and P5, the cross-repo work (HMI ABB overrides, Weld Planner ABB emission);
  - P6, the real cell.

Written 2026-09-25 from the four new KAREL sources, the HMI and Weld Planner code, the ABB
manuals and a live read-only look at the running VC.

P0 established on the VC (§6, P0 status):
- plain `SearchL \Stop` stops on its DI, so curobo E17 is wrong for it;
- `SearchPoint` is the detection point, in the work object's frame (the §3.3 contract);
- the D3 pause-and-retry works through a late-bound call.

- **Decided 2026-09-25 (§8.1):**
  - `SearchL` is the v1 primitive;
  - no request 17 on ABB;
  - a missed touch pauses, as on FANUC;
  - P0 uses World Zones plus VC-only I/O on the MONARC VC;
  - `TEACHMODE_ON` ports as a cell macro;
  - v1 covers indexed welds;
  - I draft the HMI and Weld Planner changes too;
  - searches run at 15 mm/s over a 150 mm stroke;
  - no wire trim (the stickout is assumed fine);
  - Teach mode turns off by itself during welding;
  - no automatic return after a touch: the program's own move back is the return (D12,
    2026-09-26).
- **Still open:** none (§8.2).
- Everything still marked **[unverified]** concerns SmarTac (X4/X5, not run under D1) or
  the real welder signals (P6).

Scope: HMI requests **16-19** (`R_TS_D`, `R_TS_F`, `R_TS_P`, `R_TS_END`), the "classic"
auto touch-up offset, on **indexed** welds (D6).

The planner's **endpoint search** mode (`ETSP<weld>`, FANUC Simple TouchSense writing
PR[90]/PR[91]) does not use these requests. It is out of scope here: a separate task
with its own investigation (owner, 2026-09-25).

Related docs:
- [fanuc_hmi_request_program_calls_v1.md](fanuc_hmi_request_program_calls_v1.md): the request-number table (ids 16-19).
- [abb_port_plan_v1.md](abb_port_plan_v1.md): the wire conventions this port inherits (§1.2, §4.5 pose literal).
- [weld_frame_update_strategy_v1.md](weld_frame_update_strategy_v1.md): THE RULE (served frames go to `.oframe`), which `R_TS_END` also follows.
- [abb_error_recovery_matrix_v1.md](abb_error_recovery_matrix_v1.md): the recovery attitude a missed touch has to fit into.
- [rapid_validation_findings_v1.md](rapid_validation_findings_v1.md): where the P0 findings get recorded.
- `customer_specific/monarc/monarc_cell_teardown.html`, "Finding the part": a real ABB cell doing wire touch sensing with plain `SearchL`/`SearchC`.

---

## 0. Summary

1. **On FANUC the controller only measures. The HMI computes.** The TP program searches;
   `R_TS_P` hands the HMI the TCP at contact, in the weld UFRAME. At `R_TS_END` the HMI
   computes a translation-only correction and sends back a corrected weld frame. The
   FANUC schedule's own offset (`Search Start [n] PR[10]`) is never used. §1.
2. **The same split carries over to ABB unchanged.** The robot needs to do three things:
   search along a line until the wire touches, report where the TCP was at contact in the
   weld work object, and accept a new `.oframe`. None of ABB's own correction tools
   (`PDispSet`, `PrePDisp`, `OFrameChange`) are needed. §2.4, §3.
3. **ABB has no "search frame" object and does not need one.** An ABB search is defined by
   two robtargets, so its direction is just the vector between them. The robtargets live
   in the weld work object, and that object's `.oframe` is the localized frame the HMI
   served. So the search direction follows the part automatically, which is exactly what
   FANUC's dynamic `$TH_WRKFRAME` update (`R_TS_F`) exists to do. For `.tgs` projects the
   HMI's search frame *is* the served weld frame (`cad_T_searchFrame` is always identity).
   **Decided (D2): ABB does not send request 17.** §3.3.
4. **Two ABB primitives can do the touch.**
   - `SearchL \Stop` is base RAPID. Its manual pins down exactly what it returns: *"the
     position of the TCP … when the search signal has been triggered … taking the
     specified tool, work object and active ProgDisp/ExtOffs … into consideration"*.
   - SmarTac `Search_1D \SearchStop` (option 657-1) is ABB's touch-sense product. It is
     configured per cell in PROC, and both MONARC controllers already have a welder
     profile (`smtMiller1` on the VC we run, `smtFronius1` on the real 4600-804589).
   - Both are wrapped behind one TG routine so the choice never reaches `.tgs` programs.
   - **Decided (D1): `SearchL` for v1.** SmarTac can replace it later, if P0 shows it
     searches on a VC and runs headless.
   - A forum report says SmarTac runs searches as plain moves on a VC. The Weld Planner
     tracker's **E17** says the same of `SearchL`. Neither claim had a primary source.
     **P0 disproved E17 for `SearchL`:** it stops on its DI on the VC. SmarTac was not
     tested. §2.6, §3.4.
5. **The wire.**
   - 16: unchanged.
   - 18: sends the contact TCP as a pose literal (the ABB convention, plan §4.5). The HMI
     keeps only xyz, as today.
   - 19: receives a pose literal into `WObj.oframe`, exactly like `TG_ReqWeldFrame`.
   §3.2.
6. **The HMI and the Weld Planner both refuse ABB touch sensing today.**
   - The HMI aborts the run on id 16 for ABB, and `ABBRobot` implements none of the four
     handlers.
   - The planner's touch generation is FANUC-only (Phase 15, "cannot be validated offline").
   - The port is therefore three repos, deployed together. §5.
7. **Delivery follows the usual phases: experiments first, then code.** P0 confirms that
   `SearchL` searches on the VC and pins its frame semantics before any production RAPID
   is written. §6.

---

## 1. What FANUC does today (the source of truth)

### 1.1 The four requests, from the KAREL sources

Files: `Resources/FANUC/KAREL/R_TS_D.kl`, `R_TS_F.kl`, `R_TS_P.kl`, `R_TS_END.kl`
(added 2026-09-25). `>` = robot sends, then reads a 1-byte ack. `<` = robot sends a
prompt, then reads N bytes.

| KAREL | ID | Sequence | Controller side effect |
|---|---|---|---|
| `R_TS_D` "Touch Sense Do Per Weld" | 16 | `> "16"` · `< "Give me TS status"` → 1 char | `R[195:doTouchSense]` := flag |
| `R_TS_F` "Touch Sense Search Frame" | 17 | `> "17"` · `< "Give me the frame x"` … `r` → 6 × 9 chars | `$TH_WRKFRAME[1].$WORK_FRAME[1]` := frame (touch frame 1). `$THSCHEDULE[1].$MASTER_FLAG` := TRUE, `[2]` := FALSE, `[3]` := TRUE |
| `R_TS_P` "Touch Sense Point Record" | 18 | `> "18"` · `> "x,y,z,w,p,r"` of **PR[32]** (6 × `CNV_REAL_STR(v,8,3)`) | none |
| `R_TS_END` "Touch Sense End" | 19 | `> "19"` · `< "Give me the frame x"` … `r` → 6 × 9 chars | `PR[6]` and `$MNUFRAME[1,6]` (UFRAME 6, the weld frame) := frame |

Notes:
- The 1-char reply after each request id is the HMI's generic `"0"` ack
  (`FANUCRobot.cpp:72-131`).
- Frames use the usual 9-char `%+09.3f` real codec. Rotation goes on the wire in w=Rx,
  p=Ry, r=Rz order (`FANUCRobot.cpp:636-647`).
- The `"x,y,z,w,p,r"` string that `R_TS_P` sends is built with `CNV_REAL_STR(v,8,3)`, not
  with the zero-filled `%+09.3f` codec. The HMI splits on commas and `stof`s each field
  (`FANUCRobot.cpp:650-676`), so the field width does not matter.

### 1.2 The TP choreography (sample `TD05tRJYQd.ls`, lines 76-196 = Weld2)

```
! TouchSense for Weld2 ;
CALL SET_SUB_ROUTINE_SR('TSPWeld2_full') ;
CALL R_W_F ;                  <- first pass: touch-up cleared, serves the LOCALIZATION-ONLY frame
CALL R_TS_D ;
CALL CAM_CLOSE ;
IF (R[195:doTouchSense]=1) THEN ;
  CALL R_TS_F ;
  CALL TEACHMODE_ON ;
  Search Start [2] PR[10] ;   <- schedule n = number of touches; PR[10] never used
  UFRAME[6]=PR[6] ;
  J P[20..51] ... CNT100 ;    <- cuRobo bridge to the first touch
  L P[52] 775mm/sec FINE ;    <- approach
  L P[53] 775mm/sec FINE Search[+X] ;   <- search from the approach, +X of the touch frame
  CALL R_TS_P ;               <- PR[32] (contact record) -> HMI
  WAIT 0.50(sec) ;
  L P[54] 775mm/sec FINE ;    <- retreat to the approach
  ... bridge, second touch with Search[-Z] ...
  Search End ;
  CALL R_TS_END ;             <- corrected weld frame -> PR[6]/UFRAME[6]
ENDIF ;
! Start of Weld2: SET_SUB_ROUTINE_SR('PWeld2'), R_W_F (re-serves the corrected frame) ...
```

How the Weld Planner generates this (curobo_suite, read 2026-09-25):
- Emitted by `build_touchsense_program_blocks`, `fanuc_program_blocks.py:150-185`. The
  per-touch program comes from `weld_touch_generation_native.py:1973-2055`.
- **P[52] = P[53] = P[54] = the same approach pose.** The search is anchored on the
  approach, as `program_instruction_reference_v1.md:65-67` states. The solved contact
  pose is used only for simulation and validation.
- The approach is placed at `touch + snapped_axis · d` (`weld_planner_touch.py:366-397`).
  `d` is clamped to 10-100 mm; the shipped config uses 80 mm for manual points and 30 mm
  for auto points (`config.json:268-269`).
- The search token is `-snapped_axis`, snapped to the dominant **CAD** axis
  (`weld_touch_generation_native.py:895-900`). In other words, it is the approach-to-touch
  direction.
- `Search Start [n]` has n = the number of touch points, at most 3
  (`weld_touch_generation_native.py:2345`, `weld_planner_touch.py:192-203`).
- A parallel-plane guard stops two touches from probing the same axis
  (`WeldWorkflowController.py:2901-2909`).
- `TEACHMODE_ON` is emitted only when `WELDER_ENABLE_FRONIUS_TEACH_MODE` is true, which it
  is in the shipped config (`weld_touch_generation_native.py:952-954`).
  - The HMI repo ships `TEACHMODE_ON.ls` / `TEACHMODE_OFF.ls` as **empty stubs**.
  - The planner never emits `TEACHMODE_OFF`.
  - It switches the Fronius TPS/i **Teach mode** on, which retracts the wire as the torch
    nears the part so the stickout is kept (Fronius, §2.5).
  - **Owner, 2026-09-25:** while touch sensing is ON, the TPS/i disables Teach mode
    automatically, so Teach mode never disturbs a measurement. It protects the wire only
    during the moves in between.
- Search speed and search distance are **not** in the planner. They live in the
  controller's touch schedule: **15 mm/s and 150 mm** (owner, D9).
- No wire trim precedes the offset touch block. The planner's wire-snip call belongs to
  the endpoint touch-sense groups, so the stickout is assumed fine (owner, D10).
- Teach mode turns off automatically during welding, which is why no `TEACHMODE_OFF` is
  needed (owner, D11).

### 1.3 The FANUC touch-sensing concepts involved

From the FANUC touch-sensing setup manual (excerpt; source list in §9):
- **Touch frame.** It "determines the x, y, and z directions for the search motion".
  Touch frames are relative to the robot's UFRAME, or to a leader group's coordinated
  frame. `R_TS_F` rewrites touch frame 1 on every weld.
- **Contact Record PR.** "By default … position register 32": "a temporary buffer to hold
  the last search contact position … a real position, not an offset". This is what
  `R_TS_P` sends.
- **Master Flag.** When ON, "the touched positions are recorded as the reference positions
  to be used by future searches". The offset is computed only against a mastered
  reference.
  - `R_TS_F` forces schedules 1 and 3 to master mode on every weld.
  - **[inferred]** The purpose is to stop FANUC computing, or failing on, its own offsets,
    which TG discards anyway.
- **Search patterns:**
  - *Simple*: two searches; stores the found position.
  - *Fillet/Lap*: 1-D, 2-D or 3-D offset "plus rotation about an axis of which no
    searching is performed".
  - *V-Groove*.
  - *OD/ID*.
- **Search speed.** The manual default is 50 mm/s.
- **No contact within the search distance** raises alarm **THSR-017** "No contact with
  part", severity *Pause*.

### 1.4 What the HMI does with it (TGuideWeldingHMI, read 2026-09-25)

References are to `TGuideWeldingHMI/TGuideWeldingHMI/`.

- **Dispatch.** `RobotCell.cpp:1964-2041`. The handlers are virtuals on `Robot`
  (`Robot.h:137-140`), implemented only in `FANUCRobot.cpp:795-851`.
- **16, the flag.** The reply is `1` only if both of these hold (`RobotCell.cpp:679-691`):
  - the operator enabled auto touch-ups for this run (Tools menu: Disabled /
    "Touchups + Weld" / "Only Touchups"), and
  - the current weld (named by the preceding `TSP<weld>_full` R_W_F) has touch points in
    the project.

  A `1` also resets the touch counter.
- **17, the search frame.** The HMI sends `local_bTpart * cad_T_searchFrame`
  (`RobotCell.cpp:2003-2005`), in **robot base**. For `.tgs` projects `cad_T_searchFrame`
  is hard-set to identity (`WeldLibrary.cpp:861`), so it equals the weld frame that R_W_F
  just served.
- **18, the point.** The Nth report fills stored touch point N (`WeldLibrary.cpp:1004-1013`).
  - Only xyz is kept; w/p/r are discarded (`FANUCRobot.cpp:831-832`).
  - The frame assumption is a single comment: `// POINT RECORDED IN THE CAD FRAME SENT TO
    THE ROBOT` (`WeldLibrary.cpp:696`).
  - The message carries no weld id and there is no bounds check.
- **19, the offset math.** `WeldLibrary.cpp:942-969, 1015-1086`.
  - For each touch, only the coordinate **along its snapped axis** in the search frame is
    used; the other two coordinates are ignored.
  - With nominal N and actual M built that way: `δ_cad = M - N`, `δ_base = R(base_T_cad) · δ_cad`.
  - The new frame is `T(δ_base) · base_T_part`, a **translation-only** shift. It replaces
    any previous touch-up offset, is saved to the project, and is sent back
    (`RobotCell.cpp:2026-2036`).
  - 1, 2 or 3 touches on distinct axes give a 1-D, 2-D or 3-D shift. A repeated axis means
    the last touch wins.
  - **There is no Fillet/Lap/Simple distinction anywhere in the HMI.**
- **The next `R_W_F('PWeld<n>')`** re-serves `touchup · localization`. It also pushes the
  touch-up in inches, which ABB already receives into `posTG_Touchup`.
- **Known defects, both brands:**
  - Id 16 with `current_weld == nullptr` sends no reply, so the robot blocks forever.
    Recorded in the HMI's `docs/endpoint_touch_sense_hmi_plan_v1.md:563-564`.
  - "Only Touchups" mode puts weld status 3 on the wire as `"0"`. The weld is skipped,
    which is harmless but not what the mode intends.
- **ABB today:**
  - `supports_touch_sense` is FANUC-only (`RobotBrand.h:105-107`).
  - On ABB, id 16 aborts the run (`RobotCell.cpp:1966-1980`).
  - `ABBRobot` overrides none of the four handlers; the base defaults log "not supported"
    and send nothing (`Robot.cpp:29-48`).
  - `ABBRobot.cpp:200-206` already strips the `TSP…_full` token.

### 1.5 What this means for the port

The FANUC schedule and pattern settings (Fillet/Lap 1-D/3-D, Simple, the master flags)
feed FANUC's **own** offset computation. TG never uses that computation: PR[10] is never
consumed, and the HMI recomputes everything from raw contact points.

What does carry over to ABB is the physical behaviour of each search:
- its line and direction,
- its speed and maximum distance,
- sensing with the **wire**,
- stopping on first contact,
- recording the TCP **at contact**, not after the stop.

Confirmed by the owner 2026-09-25 (D7): "Fillet/Lap or Simple" for two touches only
describes how schedule 2 is set up on the controller, and sensing is with the wire. The
values that do carry over are 15 mm/s and 150 mm (D9).

---

## 2. How touch sensing works on ABB (research)

### 2.1 Three layers

| Layer | What it is | Option |
|---|---|---|
| `SearchL` / `SearchC` / `SearchExtJ` | Base RAPID. Moves linearly or circularly and records the TCP when a DI (or a PERS bool) changes. | none (RobotWare-OS) |
| **SmarTac** | ABB's touch-sense package. System module `SmarTac.sys` (read-only, encrypted) provides `Search_1D`, `Search_Groove`, `Search_Part`, `PDispAdd` and the functions `PoseAdd`, `OFrameChange`. I/O is bound in PROC (`SMARTAC_SETTINGS` / `_SIGNALS` / `_SPEEDS`). | **657-1** ("IO version" = no SmarTac board; the welder does the sensing) |
| Welder-integrated sensing | The power source puts the sense voltage on the wire and reports contact on a fieldbus DI: Miller `doWld1TouchOn` / `diWld1TouchActive` / `diWld1Touched`; Fronius TPS/i `TouchSensing` in, "Arc stable / Touch signal" out. | welder add-in |

SmarTac with the "IO version" is layers 2 and 3 together. The SmarTac manual documents
this configuration: "only the software package for SmarTac is used, no SmarTac hardware
… instead the touch sensing capability of the Fronius welder is used" (3HAC024845-001 p. 20).

### 2.2 `SearchL`: facts from the RAPID reference (3HAC050917-001 rev H, §1.234)

- **SearchPoint** is *"The position of the TCP and external axes when the search signal
  has been triggered. The position is specified in the outermost coordinate system taking
  the specified tool, work object, and active ProgDisp/ExtOffs coordinate system into
  consideration."* That is the contact TCP in the given work object, which is exactly
  FANUC's PR[32] semantics.
- **Stop modes:**
  - `\Stop`: stiff stop. "Only allowed if the TCP-speed is lower than 100 mm/s."
  - `\PStop`, `\SStop`: path stops.
  - no switch: a flying search to `ToPoint`.
  - Typical stop distance at 50 mm/s: `\Stop` 1-3 mm, `\SStop` 4-8 mm, `\PStop` 15-25 mm.
  - The robot "is not moved back to the searched position".
- **Repeatability** is 0.1-0.3 mm at 20-1000 mm/s. The I/O must be interrupt-driven
  ("use I/O device with interrupt control, not poll control"), so fieldbus latency matters.
- **The previous move must end in a `fine` point.** Otherwise the search starts on the
  real path and can hit on the wrong side (figures xx0500002244-46).
- **Errors, all recoverable:**
  - `ERR_WHLSEARCH`: no detection; the robot has continued to `ToPoint`.
  - `ERR_SIGSUPSEARCH`: the signal was already high at the start, or the signal was lost;
    the robot stops at the start.
  - `ERR_PERSSUPSEARCH`: the PERS-bool equivalent.
  - The manual's recovery example is `StorePath` → move back → `RestoPath` → `ClearPath` →
    `StartMove` → `RETRY`.
- **It runs only in the main motion task** (`T_ROB1`), and not while `StorePath` is active.

### 2.3 SmarTac: facts from the manual and from RobotWare 6.15.08 on disk

Sources:
- Application manual *SmarTac* 3HAC024845-001 rev A (© 2004-2016; full text extracted,
  source in §9).
- The RW 6.15.8029 install under
  `%LOCALAPPDATA%\ABB\RobotWare\RobotWare_6.15.8029\options\arc`:
  - `RS\MoveInstructionDescriptions\Search_1D.xml`
  - `RS\SearchTemplates\*.xml`
  - `options\smartac\config\*.cfg`
  - `install.cmd`, which echoes "SmarTac 8.0" and installs the North-America style.

**`Search_1D`** (manual §6.1.1):
- Syntax: `Search_1D [\NotOff] [\Wire] Result [\SearchStop] StartPoint SearchPoint Speed
  Tool [\WObj] [\PrePDisp] [\Limit] [\SearchName] [\TLoad]`.
- The RW 6.15 descriptor also lists **`\SchSpeed`** (num, optional), which the manual does
  not document. **[unverified]**
- Motion: "a linear movement to the start point … The SmarTac board is activated and motion
  starts towards the search point … The robot will continue past the search point for a
  **total search distance described by twice the distance between StartPoint and
  SearchPoint**."
- `Speed` is "used when moving to the StartPoint. The velocity of the search motion is
  unaffected." The search velocity comes from PROC `SMARTAC_SPEEDS -main_search_speed`
  (default 20 mm/s, range 1-80).
- `SearchPoint` is programmed "so that the torch is touching the surface of the part
  feature", i.e. the **nominal contact**.
- **`\SearchStop`**: "this robtarget will be updated as the point where the robot detects
  the part feature". The manual does not say which frame it is in. **[unverified]**
- **`Result`** (pose) is "the difference between the programmed SearchPoint, and the actual
  SearchStop". `\WObj` "determines what frame Result will be related to".
- `\Wire` sets `doWIRE_SEL` (board systems only).
- `\NotOff` keeps the sensor active and the break box open. It must not precede a weld,
  or the arc fails to ignite.
- **Errors:** a FlexPendant menu offers RETRY (start point moved 50 % further out),
  RETURN (continue with the *default* result) or RAISE (to the caller). There are three
  faults: activation failed, search failed, and touching at start.
  - With `\Limit`, an over-limit result opens an OK/RAISE menu.
  - PROC `SMARTAC_SETTINGS -errorhandler` ("Disable Errorhandler") and the
    `SMT_ERR_HNDL_IO` group (dialog over I/O to a PLC or HMI) exist.
  - What `RAISE` puts in `ERRNO`, and what disabling the handler changes, is not
    documented. **[unverified]**

**The ABB-native correction model** (manual ch. 4). This is what ABB offers instead of
FANUC patterns:

| FANUC pattern | ABB equivalent |
|---|---|
| Fillet/Lap 1-D | `Search_1D pe, …; PDispSet pe;` |
| Fillet/Lap 2-D / 3-D (translation) | chained `Search_1D … \PrePDisp:=pe` (translations add; same-direction searches are averaged) |
| "…plus rotation" | separate displacement frames per weld end (exercise 4), or `OFrameChange(obREF, p1,p2,p3, pe1,pe2,pe3)`, which builds a new object frame from 3 reference points |
| Simple (found position) | `\SearchStop` (Search_1D) / `SearchPoint` (SearchL) |
| V-Groove | `Search_Groove` (wire only) |
| OD/ID | no instruction (`SearchC` is a circular *path*, not a circle-centre pattern) |

TG uses none of the correction column (§0.2). Only the "found position" row matters.

### 2.4 Why the HMI-computed model still fits ABB

The HMI needs raw contact xyz in the served part frame. On ABB that is `SearchPoint` or
`\SearchStop` in the weld work object. As long as no program displacement is active,
those coordinates are relative to `uframe · oframe`, i.e. the served frame. The HMI then
sends back one frame, and `R_TS_END` writes it where `R_W_F` writes it. Nothing on the
controller has to know how many touches there were or which axes they probed.

### 2.5 What our controllers actually have (read 2026-09-25)

| Controller | Touch-relevant facts | Source |
|---|---|---|
| **Running VC** `4600-803651_Virtual` (RW 6.15.08.00, AUTO) | Options include 633-4 Arc, **657-1 SmarTac IO version**, **Miller EIP Welder**, 608-1 World Zones, 616-1. PROC `SMARTAC_SETTINGS T_ROB1 -uses_signals "smtMiller1" -uses_speeds "smtspeedstd" -errorhandler false`. `smtMiller1`: detect `diWld1Touched`, sensor_on `doWld1TouchOn`, **sensor_active `diWld1TouchActive`** (all on `EtherNetIP/ioMillerWld1`, device running). `smtspeedstd`: main 20 mm/s, groove 15 mm/s. | live RWS GETs |
| **Real MONARC cell** `4600-804589` (RW 6.16.02, Fronius TPS/i add-in 1.09) | `SMARTAC_SETTINGS T_ROB1 -uses_signals "smtFronius1"`. `smtFronius1`: detect `siFr1PartDetect`, reference_set `soFr1SensorRef`, sensor_on `soFr1TouchSense`. The `si`/`so` signals come from the add-in, not `EIO.cfg`. | `Documents\4600-804589_Backup_20260924\SYSPAR\PROC.cfg:79-104` |
| RW 6.15 Fronius TPS/i equipment template | `doFr1TouchSense` (DO map 12), `doFr1TeachMode` (DO map 25), `diFr1TouchSense` (DI map 7), PROC `-TouchSenseDO "doFr1TouchSense"` | `…\weldequip\pws\FroniusTPSi\Cfg\eio\eARC1_EQUIP_ROB1.cfg`, `…\proc\pARC1_EQUIP_ROB1.cfg` |
| MONARC's own programs (older Miller backup) | 13 × `SearchL\Stop` / `SearchC\Stop` on `diWld1Touched` (v5 in the quoted examples), preceded by `SetDO doWld1TouchOn,1; WaitDI diWld1TouchActive,1`. **No ERROR handler anywhere**, so a missed touch aborts the part. They configured SmarTac but search with plain `SearchL`. | teardown, "Finding the part" |

Fronius TPS/i semantics, from Fronius's interface signal descriptions:
- `TouchSensing` puts ~70 V (limited to 3 A) on the wire or nozzle.
- The "Arc stable / Touch" output "lasts **0.3 s longer** than the duration of the short
  circuit". This is a plausible reason for the FANUC `WAIT 0.50` after each touch, and it
  would trip `ERR_SIGSUPSEARCH` on a search started too soon.
- `Teach mode` also applies 70 V, and **retracts the wire when the nozzle nears the part**
  to keep the stickout.
  - Owner (D5): TouchSensing ON disables Teach mode automatically.
  - The public interface description (checked 2026-09-25) states only that welding start
    blocks TouchSensing ("As long as the Welding start signal is set, the TouchSensing
    signal cannot be activated"). It says nothing about how the two interact.
  - P6 observes it on the cell: the wire must not move during a search.

### 2.6 The VC question: can any of this be tested offline?

- **curobo E17** (`abb_integration_plan_v1.md:999-1000`): "`SearchL` does not function in a
  virtual controller … travelling the full stroke". No source is cited.
- **ABB forum thread 4433** (2010):
  - One user with SmarTac saw the search run the full stroke.
  - Another reply: "SmarTac doesn't support searching in the VC as only a Move instruction
    is executed".
  - A third user: "I don't use smarttac. I just use the SearchL command directly. This
    still works", provided the I/O signal's access level is *All*.
- **ABB forum thread 10749**: an ABB staff answer on making `SearchL\Stop` stop in
  simulation, using a LineSensor that is activated by a robot signal.

Working hypothesis:
- **plain `SearchL` works on the VC if the DI really changes during the move;**
- **`Search_1D` may be a plain move on the VC.**

Both were **[unverified]** and 16 years old, and they decided the test strategy, so P0
tested them first.

**Result (P0, 2026-09-25): the `SearchL` half is confirmed, and E17 is wrong for plain
`SearchL`.**
- On the RW 6.15.08 VC, `SearchL \Stop` stopped on a DI that really changed during the
  move. It stopped 99 mm short of its ToPoint, three times out of three.
- It returned the detection point in the work object's frame (§6, P0 status).
- E17 describes what happens when nothing drives the DI.
- The SmarTac half (X4) was not run: D1 chose `SearchL`, and X4 is optional.

---

## 3. Design: mapping FANUC to ABB

### 3.1 Line-by-line mapping

| FANUC (TP / KAREL) | ABB | Notes |
|---|---|---|
| `SET_SUB_ROUTINE_SR('TSPWeld2_full')` + `R_W_F` | `stTG_SubName:="TSPWeld2_full"; TG_ReqWeldFrame \Tool:=tTG_Weld \WObj:=<weld wobj>;` | exists; the HMI already handles the token on ABB |
| `R_TS_D` → R[195] | `TG_ReqTouchSenseDo;` → `nTG_DoTouchSense` | new, wire identical |
| `IF R[195]=1` | `IF nTG_DoTouchSense=1 THEN` | |
| `R_TS_F` → `$TH_WRKFRAME[1]` | **not sent** (D2) | §3.3 |
| `TEACHMODE_ON` | `TG_TeachModeOn;` (cell macro in `TG_Cell.sys`, placeholder like `TG_WeldPrep`; TPS/i: `doFr1TeachMode`, §2.5) | D5; same place in the block as FANUC |
| (ArcTool drives the touch output during `Search[]`) | `TG_TouchSearch` calls the cell macros `TG_TouchSenseOn` / `TG_TouchSenseOff` around **each** search | sensing is live only while searching, and Teach mode protects the wire during the bridges (D5) |
| `Search Start [n] PR[m]` / `Search End` | nothing | ABB searches are self-contained |
| `UFRAME[6]=PR[6]` | nothing | PERS references are live (plan §7.6 style b) |
| bridge `J P[..] CNT100` | `MoveJ … z<n>` | exporter, as for welds |
| `L P[52] FINE` (approach) | `MoveL rtApproach, vApproach, fine, tTG_Weld \WObj:=…;` | `fine` is mandatory before a search (§2.2) |
| `L P[53] FINE Search[+X]` | `TG_TouchSearch rtApproach, rtContact, tTG_Weld \WObj:=…;` → `rtTG_TouchHit` | new primitive, §3.4 |
| `R_TS_P` (PR[32]) | `TG_ReqTouchSensePoint;` | new; sends `rtTG_TouchHit` as a pose literal |
| `WAIT 0.50(sec)` | `WaitTime 0.5;` (parity), and see §3.6 | |
| `L P[54] FINE` (retreat) | `MoveL rtApproach, …, fine, …;` | the ONLY return: FANUC schedules run with "auto return" off, and `TG_TouchSearch` does not return either (D12) |
| `R_TS_END` → PR[6]/UFRAME[6] | `TG_ReqTouchSenseEnd \Tool:=tTG_Weld \WObj:=<weld wobj>;` → `WObj.oframe` | new; THE RULE |

### 3.2 The ABB wire, per request

The conventions are those of `abb_port_plan_v1.md` §1.2 and §4.5: FANUC choreography,
prompts and acks, with poses and frames as one RAPID pose literal
`[[x,y,z],[q1,q2,q3,q4]]`.

| ID | ABB sequence | HMI ABB side |
|---|---|---|
| 16 | `> "16"` · `< "Give me TS status"` → 1 char (`0`/`1`) | same as FANUC. **Must always reply**, including the `nullptr` case. |
| 17 | **not sent** (D2) | nothing; the id-16 guard covers `cad_T_searchFrame ≠ I` |
| 18 | `> "18"` · `> <pose literal of the contact TCP in the weld wobj>` | parse the literal, keep xyz (as FANUC keeps xyz) |
| 19 | `> "19"` · `< "Give me the frame"` → pose literal → `WObj.oframe` | same codec and the same `StationRebase::ToLiveWorld` path as `sendWeldFrameReply` |

- A malformed frame on 19 must never be welded against. As built in P1, it **abandons the
  cycle** (`tgTouchAbort`). An abort sentinel would not survive: the weld's own R_W_F
  overwrites `nTG_WeldStatus` right after.
- Id 18 carries no "hit valid" marker on the wire (D3). The robot simply never sends an
  invalid hit (§3.6).

### 3.3 Frames: why ABB needs no search frame

FANUC needs `R_TS_F` because a `Search[+X]` direction lives in a *touch frame* that is
separate from UFRAME 6. When vision moves the part, the touch frame has to be rotated to
match, and that is what request 17 does.

On ABB the direction is the vector `StartPoint → SearchPoint`. It is expressed in the same
work object as the targets, whose `.oframe` the HMI serves in the preceding
`TG_ReqWeldFrame`, so it rotates with the part for free. For `.tgs` projects the frame the
HMI would send in 17 **is** that served frame (`cad_T_searchFrame = I`, §1.4).

Omitting 17 on ABB is safe:
- the robot drives the protocol, so a request it never sends has no HMI-side state to
  desync;
- the HMI's 17 handler only transmits.

Recommended guard: the HMI's ABB path refuses (at id 16) a weld whose
`cad_T_searchFrame ≠ I`. Only legacy folder projects can have that, and ABB never loads
them.

The rules the contact report depends on:
1. **No program displacement is active during a search.** Both `SearchL`'s `SearchPoint`
   and (presumably) SmarTac's `\SearchStop` include the active ProgDisp. TG programs never
   use `PDispSet`, and TG must not use `\PrePDisp`.
2. **`uframe` is identity** for base-referenced work objects (`TG_Comms.sys` THE RULE). The
   `.tgs` already resets it at entry.
3. **The search tool is `tTG_Weld`**, with its TCP at the wire tip at nominal stickout, the
   same TCP the FANUC UT8 represents.
4. **Indexed welds, the v1 scope (D6).** The HMI serves the frame at the authored
   positioner angle, and the searches run at that angle, in `wobjTG_Weld`.
   - `TG_TouchSearch` must derive its end point by offsetting **only** `trans`, so the
     positioner stays still during the search.
   - That means the `extax` of the start, contact and end targets must all equal the
     index.
   - The index must be reached before the `TSP…` `TG_ReqWeldFrame`, as TD05Weld already
     requires for welds.
5. **Coordinated welds are later (P7, outside v1 per D6).** In a coordinated work object the contact is still
   reported in part coordinates, so the HMI's `δ_cad` is shape-independent. Only the
   composition changes, to `stn_T_cad · T(δ_cad)` instead of `T(R·δ_cad) · base_T_cad`.
   Today's FANUC coordinated path multiplies a positioner-frame delta by the search frame
   and is meaningless (HMI note), so there is no parity target yet.

### 3.4 The touch primitive: `SearchL` or SmarTac `Search_1D`

Exported programs call **one** TG routine and never the primitive itself:

```
TG_TouchSearch StartPoint, ContactPoint, Tool \WObj   ! illustrative signature only
```

It fills `rtTG_TouchHit` and a valid flag, and owns all error handling. It has to: an
error raised inside a late-bound `.tgs` does not propagate to `TG_Main`
(`TG_Comms.sys:700-715`, finding F-F).

| | A: SmarTac `Search_1D \SearchStop` | B: `SearchL \Stop` + cell signals |
|---|---|---|
| Returned point | `\SearchStop`: frame and detect-vs-stop point **[unverified]** | `SearchPoint`: documented (contact TCP in `\WObj`, §2.2) |
| Welder specifics | in PROC, already configured on both MONARC controllers (`smtMiller1`, `smtFronius1`), including Fronius's `SensorRef` gating | in `TG_Cell` (sensor on / wait active / detect DI), per welder. `AliasIO` can bind the detect DI, and `ReadCfgData` could even read it from the SmarTac PROC profile. |
| Search geometry | total stroke = 2 × \|Start − Search\| (fixed); FANUC's 150 mm from the approach (D9) forces `SearchPoint` off the contact | `ToPoint` = approach + 150 mm along the direction: exact FANUC parity |
| Search speed | PROC `main_search_speed` (cell-wide); `\SchSpeed` **[unverified]** | per call |
| Missed touch | pendant menu (RETRY / RETURN / RAISE). RETURN continues with a **default** result, which must never reach the HMI as a measurement. Headless behaviour **[unverified]**. | `ERR_WHLSEARCH` in our ERROR handler: fully headless |
| Touching at start | built-in RETRY with the start moved out 50 % | `ERR_SIGSUPSEARCH`, handled by us |
| Option dependency | 657-1 at **link time**: the calling module will not load without it | none |
| VC testability | forum says a plain move **[unverified]** (X4) | forum says it works if the DI changes **[unverified]** (X1) |
| Precedent | ABB's product for this job | what MONARC's own programmer used on this very cell |

**Decided (D1): build `TG_TouchSearch` on B (`SearchL`) for v1.** Adopt A behind the
same signature only if X4/X5 show that `Search_1D` searches on the VC and can run
headless.

Reasons:
- `SearchL` is the only primitive whose returned point is documented.
- It is the only one whose failure we can handle without a pendant dialog.
- It has no option dependency.
- It is probably the only one we can validate offline.

Its cost is one welder-specific cell macro, `TG_TouchSenseOn`/`Off`, which is exactly
what `TG_Cell.sys` exists for.

Confirmed by the owner 2026-09-25 (D1).

### 3.5 What the exporter must emit per touch

The planner already has all of this in the solved touch: approach pose, contact pose and
snapped axis.

- `rtApproach`, the StartPoint: the FANUC P[52]/P[53]/P[54] pose.
- `rtContact`: the solved contact TCP pose (planner `contact_tcp_target`), same
  orientation. On FANUC this pose exists only in the simulator.
- **B (chosen, D1).** The primitive derives
  `ToPoint = rtApproach + u · nTG_TouchStroke`, with
  `u = (rtContact - rtApproach)/|rtContact - rtApproach|`.
  - That is FANUC's geometry: the search starts at the approach and gives up 150 mm later
    (D9).
  - At the shipped standoffs, the search can go 120 mm past the nominal contact for an
    auto point (30 mm standoff) and 70 mm for a manual point (80 mm).
  - Only `trans` is offset; orientation, `robconf` and `extax` are copied from
    `rtApproach` (§3.3 item 4).
  - `rtContact` supplies the direction, and the nominal against which the pendant logs
    the measured distance.
- **A (SmarTac, not used in v1).** The stroke is fixed at 2 × |Start − Search|. A 150 mm
  stroke would need `SearchPoint` 75 mm from the start, not at the contact, which would
  make its `Result` meaningless. That is one more reason D1 went to B.
- **Speeds.** The approach runs at the existing 775 mm/s parity speed. The search runs at
  `nTG_TouchSpeed` = 15 mm/s (D9), well inside `\Stop`'s < 100 mm/s limit.
- **Keep the direction axis-snapped in the CAD frame.**
  - The HMI's math uses only the snapped-axis coordinate of each contact (§1.4), so an
    oblique ABB search would feed it a coordinate it does not model.
  - curobo **E30** ("relax rather than transfer" the `+X..-Z` validation for ABB) therefore
    needs an HMI math change first. Flag it on the planner side.

### 3.6 Failure policy (decided: D3 = option (a))

| Event | FANUC today | ABB options |
|---|---|---|
| No contact within the stroke | THSR-017, **Pause**; the operator decides | (a) **parity**: retreat to the StartPoint, TPWrite, `Stop`; Start retries. The wire stays in lockstep because 18 has not been sent. (b) Skip auto touch-up for this weld and weld on the vision frame. The HMI must be told, so a new wire value on 18 is needed. (c) Abort the program via the existing `nTG_WeldStatus=2` path. (d) Skip the weld. |
| Wire already touching at start | alarm | wait for the DI to drop (bounded, ≥ 0.3 s for Fronius, §2.5), retry once, then as above |
| Signal lost / welder not in touch mode | alarm | `TG_TouchSenseOn` waits for the welder's "active" DI with `\MaxTime`; a timeout is treated as no-contact |

Decided (D3): **(a)**. It is FANUC-parity and needs no HMI change.

- A stale `rtTG_TouchHit` is structurally impossible: the primitive clears the valid flag
  before every search, and `TG_ReqTouchSensePoint` refuses to send an invalid hit.
- Log every search on the pendant: contact xyz, and distance from the nominal contact.
  Transcripts then carry the numbers needed for numeric checks.

### 3.7 Where the code will live

| Item | Module | Why |
|---|---|---|
| `TG_ReqTouchSenseDo` / `…Point` / `…End`, `nTG_DoTouchSense`, `rtTG_TouchHit`, hit-valid flag | `TG_Comms.sys` | pure protocol, like every other request |
| `TG_TouchSearch` | `TG_Touch.sys` (new, resident) | owns motion and error recovery. Under option A it references `Search_1D` and so must load only on 657-1 systems, the same reason `TG_Weld.sys` is separate (Arc). |
| `TG_TouchSenseOn` / `TG_TouchSenseOff` / `TG_TeachModeOn` | `TG_Cell.sys` | welder I/O = cell hardware. Placeholder bodies, as for `TG_WeldPrep`. Filled in with `doWld1TouchOn`/`diWld1TouchActive` for the Miller VC, and the TPS/i touch-sense and `doFr1TeachMode` signals for the real cell. |
| sample program `TD05Touch.mod` | `abb/rapid/TGS/` | the executable spec of what the exporter emits: the `TSPWeld2_full` block line for line (2 touches, +X then -Z) |

Names follow `tg_naming_convention.md`: `TG_` public, `tg` local, and type prefix before
`TG_`.

---

## 4. Accuracy notes (why numbers from the VC are not accuracy numbers)

- **Detection latency.** `SearchPoint` is where the TCP was when the controller *saw* the
  DI. A fieldbus welder DI adds its update period: at the chosen 15 mm/s (D9), 10 ms is
  0.15 mm, as a **systematic** lag along the search direction. FANUC has the same effect
  through its own I/O. So keep the speed identical to FANUC's, and compare ABB with FANUC
  offsets on the same part (P6).
- **Stop overshoot.** With `\Stop` the wire is pushed past the contact by the stop
  distance. The manual gives 1-3 mm at 50 mm/s; at 15 mm/s it should be smaller, and X6
  measures it. The report is unaffected (it is the detection point), but the wire can
  bend. Teach mode does not help here, because it is off while touch sensing is on (D5).
- **Stickout.** The TCP is the wire tip at nominal stickout. Any stickout error is a 1:1
  error along the torch axis. Offset touch sensing runs no trim (D10), so the nominal
  stickout is an accepted assumption, as it is on FANUC today.
- **VC runs validate geometry and choreography** (frames, wire, error paths), not the
  sensing physics.

---

## 5. Cross-repo work

The three repos deploy together, as they did for R_W_P. I draft all three (D8), with the
HMI and planner changes starting once P1 has frozen the wire.

### 5.1 TGuideWeldingHMI

1. `RobotBrand.h`: enable `supports_touch_sense` for ABB, behind a config key until P6
   passes. The id-16 gate then admits ABB.
2. `ABBRobot`: implement three of the four virtuals.
   - 16: as FANUC.
   - 18: parse the pose literal and return xyz.
   - 19: pose literal through the same path as `sendWeldFrameReply`, including
     `StationRebase::ToLiveWorld`.
   - 17 stays unimplemented (D2). The robot never sends it.
3. Fix the id-16 `nullptr` no-reply deadlock (both brands).
4. Guard: ABB plus `cad_T_searchFrame ≠ I` → reply `0` at id 16, and log it.
5. Unit tests for `AutoTouchUpsOffset`. None exist today; they would pin the per-axis math
   that §3.5 depends on.

### 5.2 curobo_suite (Weld Planner)

1. The ABB touch generation path (tracker Phase 15). Per touch it emits:
   - approach `MoveL … fine`;
   - `TG_TouchSearch rtApproach, rtContact, …`;
   - `TG_ReqTouchSensePoint`;
   - `WaitTime 0.5`;
   - retreat.

   The wrapper emits `TG_ReqTouchSenseDo` / `IF nTG_DoTouchSense=1` /
   `TG_TeachModeOn` (gated on `WELDER_ENABLE_FRONIUS_TEACH_MODE`, as for FANUC) / … /
   `TG_ReqTouchSenseEnd`. Touch sensing on/off lives inside `TG_TouchSearch`, not in the
   emitted program.
2. `AbbTranslator.MoveLSearch` stops being "MoveL + warning".
3. `abb_hmi_request_contract_v1.md`: move 16/18/19 out of "Not ported".
4. `robot_brands/abb.py`: `supports_touch_generation`.
5. **E17** needs correcting with P0's evidence. Plain `SearchL` does stop on its DI on a
   RW 6.15 VC (§6 P0 status, X1). That makes touch sensing VC-testable for geometry and
   choreography, while the sensing physics stays P6. The tracker is edited when D8's
   planner work starts.
6. **F-5**: exported `.tgs` data must stay `LOCAL`. A global `PERS` that clashes with a
   resident module locks the task
   ([rapid_validation_findings_v1.md](rapid_validation_findings_v1.md)).
7. **E30** gets the §3.5 caveat.

### 5.3 abb_support (this repo)

- RAPID per §3.7.
- `hmi_prototype/abb_server.py`: handlers 16/18/19 plus a Python mirror of
  the HMI's per-axis offset math, so VC transcripts can be checked numerically.
- `hmi_prototype/test_phase8_touchsense.py`: wire and choreography tests, with a fake-robot
  executable spec of the TSP block.
- `docs/robotstudio_setup.md` §19.
- Findings go into `rapid_validation_findings_v1.md`.

---

## 6. Phased plan

Each phase ships, per the standing delivery style:
- code;
- unittest, including a fake-robot spec;
- a numbered `robotstudio_setup.md` section with the exact modules, command, expected
  transcripts and a pass criterion;
- a status note here.

### P0: VC experiments (probe module only, no production code)

A throw-away `abb/rapid/TGTouchProbe.mod` (like `TGFsProbe.mod`) run on the MONARC VC.

**How to fire a touch on the VC:**
- **M1, manual.** Toggle the detect DI in RobotStudio's I/O Simulator during a slow
  (5 mm/s) search. Good enough for a smoke test.
- **M2, pure RAPID + World Zones (chosen, D4).** The VC has 608-1.
  - A `WZBoxDef` box plays the part surface.
  - `WZDOSet … \Inside … doTG_SimTouch` drives a VC-only DO when the TCP enters it.
  - A VC-only `EIO_CROSS` copies it to a VC-only DI `diTG_SimTouched`. Cross connections
    to a DI are legal; the SmarTac manual's own Fronius example does it.
  - The same trick turns `doTG_SimSensorOn` into `diTG_SimSensorActive`, and a VC-only
    SmarTac profile `smtTG_Sim` uses the pair.
  - The real Miller signals are not touched, and the "contact" happens at a known plane,
    which gives numeric pass criteria.
- **M3, RobotStudio Station Logic.** A LineSensor on the torch drives the DI (the ABB
  answer in forum thread 10749). Most realistic, but needs a station edit.

**Experiments** (each result is recorded with its transcript):

| # | Question | Decides |
|---|---|---|
| X1 | Does `SearchL \Stop` stop on the VC when the DI rises (M2)? Is `SearchPoint` the *detection* point, measured as the difference from `CRobT` after the stop? | E17; whether B is VC-testable |
| X2 | With a **non-identity** `oframe` (served frame) and identity `uframe`, is `SearchPoint.trans = inverse(oframe) · TCP_base` at detection? Is it unaffected by the robtargets' orientation? | §3.3 frame contract (the F-2-class risk) |
| X3 | `ERR_WHLSEARCH` (no box) and `ERR_SIGSUPSEARCH` (DI high at start): where is the robot, and does the §2.2 recovery choreography work from inside a routine called by a late-bound module? | §3.6 implementation |
| X4 | Does `Search_1D` link on this VC (657-1)? With `smtTG_Sim`, does it *search* (stop at the box) or move the full 2d stroke? Is `\SearchStop` filled, and in which frame? `Result` = SearchPoint − SearchStop? `\SchSpeed` accepted? | option A viability; forum claim |
| X5 | SmarTac missed touch: default menu behaviour; with `-errorhandler TRUE`, what `ERRNO` reaches the caller; what RETURN leaves in `\SearchStop` | whether A can run headless |
| X6 | Timing: `TG_TouchSenseOn` → active DI; overshoot after `\Stop` at 15 mm/s (D9); a full 150 mm miss, to time the pause path | parameters for P2 |

X1-X3 alone are enough to start P1/P2 on option B. X4/X5 run in parallel; they only
decide whether option A replaces B.

**P0 status: DONE for X1, X2, X3 and X6. 27/27 verdict rows PASS, 2026-09-25.**

- **Files.** The probe is `hmi_prototype/vc_probes/TG_TouchProbe.mod`, not
  `abb/rapid/`: it follows the other VC probes, and it is never for a cell. The VC-only I/O
  is `TG_TouchSimEIO.cfg`, loaded as D4. The runner and judge are `touch_probe.py` and
  `touch_probe_math.py`, and the judge has 22 unit tests.
- **Run.** Recipe and expected output are in [robotstudio_setup.md](robotstudio_setup.md)
  §19. Raw data: `touch_probe_20260925_235715.json`.

| # | Result | What it settles |
|---|---|---|
| X1 | Three searches at 15 mm/s. Each **stopped 99.15 mm short of the ToPoint**, so the search stopped on the DI. SearchPoint was **0.10 mm above the face** and the robot came to rest **0.95 mm past it**. Spread 0.000 mm. | E17 is wrong for plain `SearchL`. SearchPoint is the detection point, not the stop point: the FANUC PR[32] semantics. |
| X6 | At 50 mm/s: SearchPoint 0.43 mm above the face, stop 3.14 mm past it. The TRM gives 1-3 mm at 50 mm/s, which matches. | The ~0.1/0.43 mm lead is the simulated trigger. **Refined by X7 in P3:** the rig sees a touch only every 24 ms, up to one period early, so the lead is a sawtooth between 0 and 0.36 mm at 15 mm/s; 0.1 mm was one phase of it. Timing, not a frame error. Real latency is measured in P6. |
| X2 | Searched in a work object with identity `uframe` and `oframe` = S + [80,-60,-120], `OrientZYX(35,10,-20)`. `oframe × SearchPoint` equals X1's world hit to **0.004 mm**. SearchPoint is 0.0001 mm off the part-frame search line, and carries the part-frame tool orientation. `CRobT` agrees between the two work objects to 0.004 mm. | **§3.3 frame contract confirmed.** SearchPoint comes back in the work object's coordinates, which is exactly what the HMI assumes for id 18. |
| X3a | No part: `ERR_WHLSEARCH` (**ERRNO 1072**) after 9.95 s, i.e. the full 150 mm stroke. The robot was at the ToPoint (0.67 mm, read unsettled). The manual's `StorePath` / `MoveL` S / `RestoPath` / `ClearPath` / `StartMove` brought it back to S (0.11 mm). Event `40574` "Search Warning … move back to the start position". | The D3 retreat works. |
| X3b | Touching at the start: `ERR_SIGSUPSEARCH` (**ERRNO 1073**), robot at S (0.000 mm), recovered. Event `40661`. | The "wire already touching" row of §3.6. |
| X3c | The full D3 path through a **late-bound** call: miss → retreat → `Stop` in the ERROR handler → Start → part now present → `RETRY` hit at the face (0.10 mm) → the late-bound caller resumed. | **D3 works as designed inside a `.tgs`-style late-bound call.** |

What P0 found that was not asked:
- **F-5** ([rapid_validation_findings_v1.md](rapid_validation_findings_v1.md)). A global
  `PERS` declared in two loaded modules is a semantic error that locks PP-to-main for the
  whole task. The first load collided with `TG_UfmecProbe`'s `stPrbStep`.
  - Rule for the exporter: `.tgs` data stays `LOCAL`.
  - Rule for P1/P2: every new TG global name must be checked against the resident modules.
- **Settle before `CRobT`.** The first run read S 1.4 mm off because the robot had just
  arrived. This is the known effect `tgSendPose` already guards against. The primitive
  reports `SearchPoint`, so it is unaffected, but any `CRobT` in P2 needs the same settle.
- **RWS warm restart** on RW 6.15 is `POST /ctrl` with `restart-mode=restart`;
  `/ctrl?action=restart` answers 400.
- **SmarTac's START hook** logs `80003` "SmarTac Initialized" with the active profile's
  signals at every program start. It is harmless, and it shows the VC profile is
  `smtMiller1`.

Not run: X4 and X5 (SmarTac `Search_1D`). They are optional under D1 and matter only if
`Search_1D` is reconsidered.

### P1: request PROCs and the prototype HMI

- **RAPID.** `TG_ReqTouchSenseDo` / `TG_ReqTouchSensePoint` / `TG_ReqTouchSenseEnd`, and the
  PERS, in `TG_Comms.sys`.
- **Python.** `abb_server.py` handlers, a `touch` scenario with scripted nominal points,
  snapped axes and served frame, and the HMI offset-math mirror.
- **Tests.** Wire byte-level tests for 16/18/19 (bad-payload abort included). The HMI-math
  mirror is checked against hand-computed 1-D/2-D/3-D cases, and against the HMI C++
  formulas quoted in §1.4.
- **VC (non-Arc path).** A `TD05Test`-style run in which the robot *pretends* a hit (no
  motion). It proves the wire and the `.oframe` write, with the pass criterion that the
  served-back frame equals `T(R·δ) · frame` to 0.01 mm.

**P1 status: DONE 2026-09-26. 22 new unit tests pass (150 in the suite), and 13/13 VC checks
PASS.**

**RAPID, in `TG_Comms.sys`.** Additive: nothing that existed changed.
- New data: `nTG_DoTouchSense` (R[195]), `rtTG_TouchHit` (PR[32]) and `bTG_TouchHitOK`.
- New routines: `TG_ReqTouchSenseDo` (16), `TG_ReqTouchSensePoint` (18) and
  `TG_ReqTouchSenseEnd \WObj` (19), plus the local `tgTouchAbort`.
- **The wire:**
  - 16 is FANUC's exchange. The first character is kept, anything but `1` means no block,
    and a payload that is neither `0` nor `1` is reported.
  - 18 sends one pose literal and consumes the hit flag.
  - 19 receives one pose literal into `WObj.oframe`, or `wobjTG_Weld` if `\WObj` is
    omitted.
  - No 17 (D2).
- **Two refusals abandon the cycle** (`tgTouchAbort`: an ErrWrite, the sockets dropped,
  `ExitCycle`):
  - an id-18 report without a valid hit;
  - a malformed id-19 frame.
  - This replaces §3.2's first idea of an abort sentinel for 19. The weld's R_W_F would
    overwrite any sentinel, so there is no graceful path back from 19, and the weld must
    never run against a frame the HMI did not mean.
- New names checked against every resident module (F-5).

**Sample program.** `abb/rapid/TGS/TD05TsWire.mod` is the FANUC `TSPWeld2_full` block,
line for line, with both searches pretended: it writes `rtTG_TouchHit` itself.
- Its "measured" points differ from the prototype's nominal ones by +3.000 mm X and
  -2.000 mm Z.
- Each point carries junk in the coordinates its search does not measure.

**Python.**
- `abb_server.py` has handlers for 16/18/19 and `auto_touchup_offset`, the HMI's per-axis
  math. It refuses a report count other than the weld's, where the HMI has no bounds
  check. The weld's served frame composes the stored touch-up, the TSP R_W_F clears it,
  and 16 always answers, avoiding the HMI's `nullptr` deadlock.
- The modes are `touch-wire`, `touch-off` and `touch-corrupt`.
- `hmi_prototype/test_phase8_touchsense.py` holds the math cases and
  `FakeTouchWireRobot`, the executable spec of `TD05TsWire` under `TG_Main`.

**The old tests are fixed too.**
- `test_phase1`-`6` had been red since the two-port handshake landed: the fake robots never
  answered it, and the HMI waited 30 s on **the real port 2001**.
- They now share `hmi_prototype/fake_rapid.FakeHandshake`, on an ephemeral port.

**VC results.** `hmi_prototype/vc_probes/touch_wire_vc.py`; recipe in
[robotstudio_setup.md](robotstudio_setup.md) §20. The runner deploys `TG_Comms.sys`, then
runs `TG_Main.tgs_main` against the prototype HMI.

| Scenario | Result |
|---|---|
| `touch-wire` | request order `10 5 4 16 18 18 19 4 100`. The HMI received the pretended points exactly. The controller's own `wobjTG_Weld.oframe` = localization + R·(3,0,-2) to **0.003 mm**, with the rotation unchanged (\|Δq\| 4e-7). `nTG_DoTouchSense` = 1 and the hit flag was consumed. |
| `touch-off` | `10 5 4 16 4 100`, the block skipped. `oframe` = the localization exactly. |
| `touch-corrupt` | the cycle ends at 19, with no weld frame and no end request. Event `80001` "TG: touch sensing aborted". `oframe` still holds the TSP frame. |

**Two VC harness lessons,** recorded in §20:
- A program stopped inside `TG_HandshakeCom` keeps its listener. An HMI that reconnects at
  once lands on that stale listener and times out, so the runner waits out the restarted
  `TG_SocketDisc`.
- After bursts of TG `TPWrite` lines the VC logged **41617** "Too intense frequency of
  Write Instructions", and a later `TPWrite` blocked for more than 75 s. Only a warm
  restart cleared it (see findings, related observation 2).

### P2: `TG_TouchSearch` and the cell macros

- **RAPID.** `TG_Touch.sys` (option B, D1), and `TG_TouchSenseOn`/`Off` and
  `TG_TeachModeOn` placeholders in `TG_Cell.sys`.
- **Error policy** per D3 (pause, retreat, Start retries). The hit-valid flag and the
  on-pendant logging are in scope.
- **VC.** With the M2 box at a known offset, the reported contact must lie on the search
  line (≤ 0.1 mm off-line), with its snapped-axis coordinate within v·(one I/O cycle) of
  the box face. X3 scenarios reproduce the chosen failure behaviour.

**P2 status: DONE 2026-09-26. 36/36 VC checks PASS, 159 unit tests.**

**`abb/rapid/TG_Touch.sys`** (new, resident, after `TG_Cell`):
- **The call:** `TG_TouchSearch StartPoint, ContactPoint, Tool, WObj`.
  - It is `SearchL \Stop` along StartPoint → ContactPoint, for `nTG_TouchStroke` = 150 mm
    at `nTG_TouchSpeed` = 15 mm/s (D9). The speed is clamped below 100 mm/s, the `\Stop`
    limit.
  - Only the position is extended, so orientation, confdata and external axes come from
    StartPoint (D6).
  - It moves to StartPoint on a fine point first; normally that is a zero-length move.
- **The hit** goes to `rtTG_TouchHit` / `bTG_TouchHitOK` (TG_Comms) for id 18. After it,
  **no return (D12)**; the robot stays just past the contact.
- **Every failure pauses (D3):** an event-log warning, `Stop`, and Start searches again. A
  loop does this, not `RETRY`, so the sense voltage is re-armed before each attempt. Four
  reasons, counted in `nTG_TouchPauses` / `nTG_TouchLastPause`:
  1. the signal is not configured (`ERR_ALIASIO_DEF`);
  2. the welder is not live (no confirmation within `nTG_TouchActiveWait`);
  3. no contact (`ERR_WHLSEARCH`; recovery back to the start first);
  4. touching at the start (`ERR_SIGSUPSEARCH`, after waiting up to `nTG_TouchClearWait`
     = 1 s for the contact DI to drop; the Fronius holds it 0.3 s).
- A StartPoint/ContactPoint pair less than 1 mm apart is an exporter bug: `ErrWrite` +
  `EXIT`.

**`TG_Cell.sys`:**
- `TG_TeachModeOn`, `TG_TouchSenseOn` and `TG_TouchSenseOff` drive the welder by NAME,
  through `AliasIO` on the new cell data: `stTG_TouchDI`, `stTG_TouchOnDO`,
  `stTG_TouchActiveDI` + `nTG_TouchActiveWait`, and `stTG_TeachModeDO`.
- The defaults are the MONARC VC's Miller signals, the same ones SmarTac's `smtMiller1`
  uses. The real cell's Fronius values come at P6.
- A new welder, or a test rig, therefore changes data, not code.
- Each macro re-raises its I/O errors, so they reach `TG_TouchSearch`.
- `TG_TouchSearch` calls sense-on and sense-off around **each** search (D5).

**F-5 is now checked offline.** `tools/rapid_check.py` flags a global name declared in two
of the checked modules. `--all` and the test suite run it over every shipped module.

**`hmi_prototype/_deploy_vc.py`** now deploys `TG_Touch.sys` too.

**VC results.** `hmi_prototype/vc_probes/touch_search_vc.py` + `TG_TsProbe.mod`; recipe in
[robotstudio_setup.md](robotstudio_setup.md) §21.
- The **production** `TG_TouchSearch` ran on the P0 World Zone rig, in a work object with a
  tilted `oframe`.
- The rig's names stand in for the welder, so `TG_TouchSenseOn`'s handshake really runs.
- The runner plays the operator in each paused scenario: it writes one PERS over RWS, then
  presses Start.

| Scenario | Result |
|---|---|
| S1 part present | hit 0.10 mm above the face (the known trigger lead), 0.0001 mm off the search line. **The robot was left 0.94 mm past the hit and 50.85 mm from the start: no return (D12).** Sense output off afterwards (D5). |
| S2 no part → "part placed" → Start | pause 3 after the full stroke, events `40574` + "TG: touch search found nothing". Then the same hit. |
| S3 welder never confirms → fixed → Start | pause 2 "touch sensing not live" after 2.0 s. Then the hit. |
| S4 unknown signal name → fixed → Start | pause 1 "touch signal not configured". Then the hit. |
| S5 wire touching at the start → cleared → Start | pause 4, events `40661` + "wire touching at search start". Then the hit. |
| after the run | TG_Cell's welder names were restored exactly. |

What is still open: the real-welder timing and sensing, which is P6.

### P3: sample program `TD05Touch.mod`

- The `TSPWeld2_full` block ported line for line (2 touches: +X, -Z), then `PWeld2`, run
  against the prototype HMI.
- **VC pass criterion.** Move the M2 box by a known (dx, dz), e.g. +3.000 / -2.000 mm in
  part axes. The prototype computes δ; the frame served at 19 **and** re-served at the next
  R_W_F must be the nominal shifted by R·(3, 0, -2) (±0.3 mm, sensing-limited). The weld
  then runs in the shifted frame.

**P3 status: DONE 2026-09-26. 19/19 VC checks PASS, 168 tests in the suite.**

**`abb/rapid/TGS/TD05Touch.mod`** is the executable spec of what the exporter emits for a
touch-sensed weld. It is the FANUC `TSPWeld2_full` block line for line:
1. `TG_TeachModeOn`, then a bridge.
2. Per touch: approach (fine) → `TG_TouchSearch` → `TG_ReqTouchSensePoint` →
   `WaitTime 0.5` → `MoveL` back to the approach. That move is **the** return (D12).
3. `TG_ReqTouchSenseEnd`.
4. The PWeld2 weld frame, then the weld. It is a dry pass here; `TD05Weld.mod` shows the
   arc instructions.

The part is a 100×100×200 mm block in the served frame `[1600, 0, 1450 | Rz 90°]`, so part X
is world +Y and the HMI's rotation into the base is exercised. Touch 1 searches the -X face
in +X, and touch 2 the top face in -Z.

**Contract tests** in `test_phase8_touchsense.py` check that three descriptions of that one
part agree: `TD05Touch.mod`, the rig, and `abb_server.TOUCH_REAL_DEMO`.
- contacts are on their faces, and approaches are 30 mm off along the snapped axis;
- every air move is outside the block;
- the orientation equals the home tool orientation in world;
- the rig box equals the block mapped to world.

**The rig: `hmi_prototype/vc_probes/TG_TrRig.mod`** (VC only). `TG_TrRun`:
- points TG_Cell's welder names at the World Zone rig;
- builds the block as a World Zone box, moved by `nTrShiftY/Z`;
- calls the bench `tgs_main`. The TG cycle itself is untouched: handshake, file transfer,
  `Load \Dynamic`, late-bound `TD05Touch`.

A 0.1 s TRAP would have re-made the box had the `.tgs` Load erased it. It never had to: 1
build per run.

**Runner:** `hmi_prototype/vc_probes/touch_real_vc.py`. Recipe in
[robotstudio_setup.md](robotstudio_setup.md) §22.

| Run (block shift, part frame) | HMI measured δ_cad | Controller `oframe` after the weld's R_W_F |
|---|---|---|
| baseline (0, 0, 0) | (-0.22, 0, +0.12) | `[1600.000, -0.220, 1450.120]` = the HMI's frame to 0.0000 mm |
| shifted (+3, 0, -2) | (+2.68, 0, -1.70) | `[1600.000, 2.680, 1448.300]`: part X landed on world **Y** |
| aligned (+3.24, 0, -2.16) | (+3.04, 0, -2.06) | `[1600.000, 3.040, 1447.940]` |

- aligned - baseline = (3.26, 0, -2.18) against (3.24, 0, -2.16): **0.02 mm**.
- shifted - baseline = (2.90, 0, -1.82): 0.18 mm, inside the rig's resolution.

**The rig's resolution: X7, a new P0 probe run** (`TG_TouchProbe.TG_TpX7`).
- The first shifted run missed the planned ±0.05 mm "difference" criterion, so the rig was
  measured before any tolerance was touched.
- Twelve searches, the face moved 0.03 mm further each time.
- **The hit stayed put for nine steps, then jumped 0.36 mm.** On this VC a touch is seen
  only every **24 ms** (0.36 mm at 15 mm/s), and always up to one period BEFORE the face.
  That is presumably the World Zone evaluation cycle.
- P0's constant 0.10 mm lead was one phase of that sawtooth: every P0 search started exactly
  50 mm from the face.
- The P3 criteria are therefore:
  - one quantum (+0.03 mm) for any touch and for the arbitrary shift;
  - 0.03 mm for the **aligned** shift (9 and 6 quanta). There every touch crosses its face at
    the baseline's phase, so the quantization cancels.
- The judge's lead allowance (`touch_probe_math.LEAD_S`) now covers the full quantum.
- This limits the rig, not SearchL: on the real cell the DI is an interrupt-driven welder
  signal (P6).

**Two defects found and fixed on the way:**
- **F-6** ([rapid_validation_findings_v1.md](rapid_validation_findings_v1.md)).
  `CJointT().extax`, a component of a function result, is a RAPID **syntax** error. The
  first `TD05Touch.mod` failed to load with 40322, which the cycle absorbed: requests
  `['10']` only. `tools/rapid_check.py` now flags the pattern offline.
- The runner harness had lessons of its own; they are in §22.

### P4: HMI (TGuideWeldingHMI), §5.1

P4 is gated on P1's wire, so the formats are frozen first. It is validated with the real
HMI against the VC: same scenario as P3, and the HMI log must show the same δ as the
prototype.

### P5: Weld Planner emission, §5.2

The emitted module is diffed against `TD05Touch.mod` shape. VC run as in P3.

### P6: real cell (MONARC 4600-804589, Fronius TPS/i, `smtFronius1`)

1. Fill in `TG_TouchSenseOn`/`Off` and `TG_TeachModeOn` for TPS/i.
   - Read the add-in's touch signal names over RWS first; they are not in `EIO.cfg`.
2. Measure the signal timing: touch signal hold (the 0.3 s claim), activation delay.
3. Check D5 and D11 on the cell:
   - with Teach mode on, the wire must not move during a search (watch the TPS/i "Wire
     position" output or the stickout);
   - the weld after the touch block must strike and run normally with Teach mode still
     requested.
4. Check repeatability: 10 touches on a fixed plate, spread ≤ 0.3 mm.
5. Check accuracy: shim the part by a gauge block; δ must match the shim.
6. If a FANUC cell is available, compare offsets on the same part.

### P7 (later, outside v1 per D6): coordinated welds

The composition in the station frame (§3.3 item 5), plus HMI and planner changes.

---

## 7. Risks

| Risk | Mitigation |
|---|---|
| ~~Neither primitive searches on the VC (E17 true)~~ | **Retired by P0 (2026-09-25):** `SearchL \Stop` stops on its DI on the VC (X1) |
| `\SearchStop` / `SearchPoint` frame differs from the assumption, which would silently offset every touch | X2 is a numeric check on a non-identity frame, the same lesson as F-2 |
| A stale or default point reaches the HMI | hit-valid flag, cleared per search; SmarTac RETURN is never used |
| Touch signal still high from the previous touch | wait for the DI to drop before each search; keep `WaitTime 0.5` |
| A missed touch travels the full 150 mm into whatever lies beyond the part (fixture, clamp) | same exposure as FANUC today. The planner can check that the end point is reachable and collision-free when it solves the touch; add it to §5.2 if wanted |
| HMI counts points by order with no bounds check | the exporter emits exactly one `TG_ReqTouchSensePoint` per solved touch; the fake-robot spec pins it |
| Three repos out of lockstep | same deploy discipline as R_W_P: one change set per repo, versioned together |

---

## 8. Decisions and open questions

### 8.1 Decided (owner, 2026-09-25)

- **D1 (was Q1): the primitive is `SearchL \Stop` (option B) for v1.** SmarTac
  `Search_1D` may replace it behind the same `TG_TouchSearch` signature later, but only if
  X4/X5 show that it searches on the VC and runs headless. X4/X5 stay in P0 at lower
  priority. (§3.4)
- **D2 (was Q2): ABB does not send request 17.** The search direction is implicit in the
  weld work object. The HMI's ABB path adds the `cad_T_searchFrame ≠ I` guard at id 16
  instead. (§3.3, §5.1 item 4)
- **D3 (was Q3): a missed touch pauses, as on FANUC (policy (a)).**
  1. Retreat to the StartPoint.
  2. Write the reason to the pendant.
  3. `Stop`. Start retries the search.

  No wire change. The hit-valid flag still guards id 18. (§3.6)
- **D4 (was Q6): P0 uses method M2 on the running MONARC VC (`4600-803651_Virtual`).**
  - A World Zone box stands in for the part surface.
  - VC-only I/O (`doTG_SimTouch` → `diTG_SimTouched`, plus a simulated sensor on/active
    pair) and a VC-only `smtTG_Sim` SmarTac profile.
  - The Miller signals are left untouched.
  - The exact EIO/PROC additions, and how to remove them, ship with the P0 probe module
    in `robotstudio_setup.md`.

- **D5 (was part of Q4): `TEACHMODE_ON` = Fronius TPS/i Teach mode on, ported as the cell
  macro `TG_TeachModeOn`.**
  - Its place in the block is the same as on FANUC.
  - Owner: touch sensing ON disables Teach mode automatically, so it never disturbs a
    measurement.
  - `TG_TouchSearch` therefore switches touch sensing on and off around each search.
  - Not stated in Fronius's public interface doc; P6 observes it. (§1.2, §2.5, §3.1)
- **D6 (was Q5): v1 covers indexed welds.** Coordinated welds are P7. (§3.3)
- **D7 (was Q7): "Fillet/Lap or Simple" for two touches only describes how schedule 2 is
  set up on the controller.** It is irrelevant to the HMI math and needs no ABB
  counterpart. Endpoint touch sense is a separate task and investigation. (§1.5, Scope)
- **D8 (was Q8): I draft the HMI (§5.1) and Weld Planner (§5.2) changes as well**, after
  P1 has frozen the wire. (§5)

- **D9 (was Q4): search at 15 mm/s over a maximum stroke of 150 mm**, the FANUC schedule
  values.
  - The stroke is measured from the approach, where the FANUC search starts.
  - On ABB both values are cell data in `TG_Touch.sys`: `PERS` `nTG_TouchSpeed:=15`,
    `nTG_TouchStroke:=150`. They stay on the controller as FANUC keeps them in the
    schedule, so the exporter never hard-codes them. (§3.5)
- **D10 (was Q4): no wire trim for offset touch sensing; assume the stickout is fine.**
  - The planner does have a wire-snip program call, but it is emitted before the
    **endpoint** touch-sense groups (hence the planner's `skip_wiresnip` field) and is not
    set up for offset touch sensing.
  - `WELD_PREP` (`TG_WeldPrep`) is only a placeholder.
  - So the TSP block gets no trim call on ABB either, and the nominal stickout is taken
    as given (owner, 2026-09-25). (§4)
- **D11 (was Q4): Teach mode turns off automatically during welding.** No
  `TG_TeachModeOff` is needed, matching the planner never emitting `TEACHMODE_OFF`. (§1.2)

- **D12 (owner, 2026-09-26): no automatic return after a touch.**
  - FANUC runs its touch schedules with "auto return" OFF, because the Weld Planner emits
    the return move itself (the `L P[54] FINE` back to the approach point, where the search
    started).
  - So on a hit, `TG_TouchSearch` leaves the robot where `\Stop` left it, and the exported
    program's own `MoveL rtApproach` is the return.
  - A miss is different: going back to the start is part of the D3 retry (the search
    restarts from there). That is the only move `TG_TouchSearch` makes on its own.
  - VC-proven in P2: the robot was left 0.94 mm past the hit.

### 8.2 Still open

None.

---

## 9. Sources

ABB:
- *Application manual - SmarTac*, 3HAC024845-001 rev A: [ManualsLib 1713354](https://www.manualslib.com/manual/1713354/Abb-Smartac.html). Pages used: 11-12, 17-21, 25-26, 32, 41-74, 81-103.
- *Technical reference manual - RAPID Instructions, Functions and Data types*, RW 6.08, 3HAC050917-001 rev H, §1.234 `SearchL`: [PDF](https://robotum.cz/wp-content/uploads/2019/12/Rapid_3HAC050917-TRM-RAPID-RW-6-en.pdf).
- RobotWare 6.15.8029 on disk: `options\arc\RS\MoveInstructionDescriptions\Search_1D.xml`, `Search_Groove.xml`; `RS\SearchTemplates\Search_Wire_{1,2,3}D.xml`; `options\smartac\install.cmd`, `config\mmcSmarTac.cfg`, `procSmarTac*.cfg`, `rules\smtc_cfgrules.xml`, `language\en\smtc_text.xml`; `weldequip\pws\FroniusTPSi\Cfg\…`.
- *Application manual - Arc and Arc Sensor* 3HAC050988-001 rev L (`D:\ABB\`): checked; it does not cover SmarTac.
- ABB forum threads: [4433 "Running search simulations"](https://tech-community.robotics.abb.com/discussion/4433/running-search-simulations), [10749 "Simulation for SearchL\Stop"](https://tech-community.robotics.abb.com/discussion/10749/simulation-for-searchl-stop).
- Not evaluated: ABB *WireSense for Fronius TPS/i* (3HAC082151), which is CMT-hardware edge detection, a different feature.

FANUC / Fronius:
- FANUC touch-sensing setup manual (excerpt): [pdfcoffee](https://pdfcoffee.com/fanuc-touch-sensing-5-pdf-free.html). Touch frames, Contact Record PR 32, Master Flag, patterns, THSR-017.
- Fronius *TPS/i Interface Signal Descriptions*: [manuals.fronius.com 4204260227](https://manuals.fronius.com/html/4204260227/en-US.html). TouchSensing, Teach mode, Arc stable/Touch signal.

TetraGen code (read 2026-09-25):
- abb_support: `Resources/FANUC/KAREL/R_TS_*.kl`, `Resources/FANUC/.TGS HMI-MODE LS PROGRAM SAMPLE/TD05tRJYQd.ls`, `abb/rapid/TG_Comms.sys`, `TG_Cell.sys`, `TGS/TD05Weld.mod`.
- TGuideWeldingHMI: `RobotCell.cpp`, `WeldLibrary.cpp`, `FANUCRobot.cpp`, `Robot.h/.cpp`, `RobotBrand.h`, `ABBRobot.cpp`, `AbbPoseCodec.h`.
- curobo_suite: `fanuc_program_blocks.py`, `weld_touch_generation_native.py`, `weld_planner_touch.py`, `FanucTranslator.py`, `AbbTranslator.py`, `robot_brands/abb.py`, `docs/abb_integration_plan_v1.md` (E17, E29, E30, Phase 15), `docs/abb_hmi_request_contract_v1.md`, `docs/product/touch_sense.md`.
- Controllers: running VC via RWS GETs; real MONARC cell `4600-804589` backup of 2026-09-24.
