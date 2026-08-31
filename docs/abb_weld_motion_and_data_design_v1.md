# ABB weld motion + weld data: investigation & recommended design (v1)

Status: **research complete 2026-08-28; IMPLEMENTED 2026-08-31; VC-VALIDATED
2026-08-31** (TG_Weld.sys + TGS/TD05Weld.mod + weld-demo server mode +
test_phase4_weld.py, 41/41 tests green; weld-demo ran 2 full cycles on the VC
meeting all section-15 pass criteria of robotstudio_setup.md: 8.89/220.133/
clamp-49-to-10 on weld 1, 12.7 + library zeros on weld 2). Originally
the design recommendation
for the top-priority Phase 4 item: real welds with motion in the .tgs program
(mirroring `TD05tRJYQd.ls`, touch-sense excluded), weld data, and on-the-fly
weld-data updates; plus the Weld-Planner-side design for selecting weld data on
ABB.

**Revision note (same day):** the VC turned out to already have **633-4 Arc**
installed. Inspecting the RobotWare 6.15.8029 distribution and the VC folder on
disk then resolved most of the open ⚠ items *without needing to run anything* —
including the exact `welddata`/`seamdata` component sets, the PERS requirement,
and the Fronius I/O mapping. §2.4 is the new evidence; §3 is updated to concrete
names.

**Revision note 2 (2026-08-31, first VC run):** step 1 of
[TGArcCheck.mod](../abb/rapid/TGArcCheck.mod) PASSED, so the welddata/seamdata
shapes in 2.4a/b are now CONTROLLER-CONFIRMED, not merely descriptor-derived.
Three further facts came out of the run and the controller arc log - see 2.5:
(i) the configured welder is still NOT TPS/i but EIP_awEqFrTPS4K5K, selected by
the "Fronius TPS Integrated" key exactly as 2.4d predicted; (ii) the arc system
has a CONFIGURABLE UNIT SYSTEM (PROC/ARC_UNITS: SI_UNITS / US_UNITS /
WELD_UNITS) - US_UNITS is ipm for both weld speed and wire feed, which would
remove every unit conversion from the design; (iii) autoinhib_on = TRUE explains
why arc motion ran with no welder attached. Two claims remain UNVERIFIED and are
now instrumented in v2 of the module: that welddata.weld_speed governs the weld
speed, and which optional components exist.

Builds on [abb_weld_params_research_v1.md](abb_weld_params_research_v1.md) and
[abb_port_plan_v1.md](abb_port_plan_v1.md). Cross-repo findings verified in
`curobo_suite` (Weld Planner) and `TGuideWeldingHMI` — §2.2/§2.3.

---

## 1. What must be replicated (verified from `TD05tRJYQd.ls`)

The sample runs **two welds** (Weld2, Weld3), each a **single linear pass**
with this anatomy (touch-sense blocks omitted per scope):

```
CALL SET_SUB_ROUTINE_SR('PWeld2')      ! select weld token
CALL R_W_F                              ! weld frame + status -> R[198]
CALL CAM_CLOSE
IF R[198] = 2, JMP LBL[101]             ! abort to end
IF (R[198] = 1) THEN                    ! 0 = skip weld entirely
  J P[121] 100% CNT100                  ! approach (joint)
  L P[122] 775mm/sec CNT100             ! approach (linear)
  L P[123] 775mm/sec CNT100             ! weld start point
  CALL R_W_P                            ! weld params -> R[170..175]
  IF (R[170]=0) WELD START[1,1]         ! predefined proc/schedule
  IF (R[170]=1) WELD START[R[171],20]   ! user-defined -> proc from HMI, SCH 20
  L P[124] R[175]inch/min FINE          ! THE weld move, at HMI travel speed
  IF (R[170]=0) WELD END[1,1]
  IF (R[170]=1) WELD END[R[171],20]
  CALL R_W_S                            ! weld stats (id 13) — see §6
  L P[125] 775mm/sec FINE               ! depart
ENDIF
```

Wire-level R_W_P is already done and VC-validated (`TG_ReqWeldParams`,
[TG_Comms.sys:541](../abb/rapid/TG_Comms.sys#L541)); the weld itself is the
comment placeholder at
[TD05Test.mod:138](../abb/rapid/TGS/TD05Test.mod#L138).

## 2. Verified facts (2026-08-28)

### 2.1 ABB / RAPID (public sources)

- **Pre-weld runtime modification is plain assignment on the PERS welddata**,
  including array-based recipe selection — verbatim from the ABB community
  thread: `wd.weld_speed:=nSpeed{1}; wd.main_arc.wirefeed:=nWireFeed{1};
  wd.main_arc.voltage:=nVolts{1};` before `ArcLStart`. (Component names
  independently confirmed in §2.4.)
- **Mid-weld modification is documented**: `ArcRefresh` — *"ArcRefresh is used
  to tune aw process parameters during program execution"*; the manual example
  is a **timer trap routine** that writes `weld5.weld_voltage` then calls
  `ArcRefresh;`, effective immediately.
- **`welddata.weld_speed` governs TCP speed for the whole weld**: *"the
  weldspeed stored in the start will be used for the complete weld"* (until the
  next ArcLStart). The `v` speeddata argument applies to non-welding motion.
- **Running Arc programs without a welder**: ABB staff confirm the mechanism is
  blocking the weld process (*"Maybe you need to block weld?"*), but no public
  source names the RW6 toggle. ⚠ Still the one item to confirm on the VC (§4).

### 2.2 TGuideWeldingHMI (verified in source — answers "verify this")

The claim about the HMI is **true, with qualifications**. Per-weld panel
"Modify weld parameters" (`WidgetWeldParametersDisplay`,
`WidgetTouchupOffsets.cpp:306-633`):

| GUI field | Range | Editable | Wire | Notes |
|---|---|---|---|---|
| Proc # | 1–99, step 1 | arrows only | 2 chars, zero-padded | only when "Use Custom Parameters" checked |
| Sched # | — | **read-only label** | never sent | robot side hardcodes `SCH[20]` for custom |
| Wire Speed (IPM) | 0–9999, 1 dec | arrows + keypad | 9-char real | |
| Travel Speed (IPM) | 0–9999, 1 dec | arrows + keypad | 9-char real | sent even when UDWP=0 |
| Arc Length | 0–9999, 1 dec | arrows + keypad | 9-char real | no unit label |
| Arc Control | 0–9999 | **hidden in GUI** | 9-char real | always 0.0; FANUC never applied it |
| Use Custom Parameters | bool | checkbox | 1 char (UDWP) | |

- State is **per weld** (`Weld.library`, one `WeldLibrary` per weld); **no
  per-pass state**. Native-.tgs seed defaults: proc "1", WFS 520.0, travel 21.0,
  arc length 49.0, arc control 0.0.
- UDWP=0: HMI sends only travel speed, and sends the value **scraped from the
  controller's own schedule** (`getTravelSpeedInUse`), not a planner value.
- Welder type is config-only (`config.json` → FRONIUSTPSi → `02`).
- **No ABB robot class exists** (grep-verified); the three weld-param send
  methods live on `FANUCRobot`, not the `Robot` base.

### 2.3 curobo_suite / Weld Planner (verified in source + docs)

**The repo advanced today**: ABB Phases 1–6 are merged.

- `Translators/AbbTranslator.py` exists and **already emits
  `ArcLStart`/`ArcL`/`ArcLEnd`** (retro-patching the last process move to
  `ArcLEnd`), gated behind `configure_arc_welding(enabled=)` — **off by default,
  no production caller**. Weld-data identity is four names with defaults:
  `tool0 / wobj0 / seam1 / weld1`, set via `configure_controller_data(...)`.
- The living tracker is **`docs/abb_integration_plan_v1.md`** (the
  `robot_brand_agnostic_abb_support_analysis_v1.md` doc is superseded). It has
  **already decided** (§5.4): *"ABB has **named** `seamdata`/`welddata`/
  `weavedata`. **Add a per-brand controller-binding bucket alongside; do not
  migrate the [procedure/schedule] columns.**"* Its Phase 7 (not started) is the
  ABB weld-data work.
- FANUC selection today: bind a weld to a **weld library row** (Procedure #/
  Schedule # are columns of the installation-level `weld_libraries.db`, entered
  in the library manager, shown **read-only** in the weld editor). Codegen emits
  `WELD START[<proc>,<sched>]` / `WELD START[R[171],20]` in
  `fanuc_program_blocks.py:75-83`.
- Field census: in a live installed `weld_libraries.db`, volts/WFS/travel were
  populated **0 of 14 rows** — *"the parameters live on the power source,
  selected by schedule number"*. The library is an **identity catalog**, not a
  physical-parameter store. This shapes §3.5.
- The tracker carries a worry: ABB docs *"say do not manipulate seamdata/
  welddata programmatically."* **Resolved**: that warning (Fronius AM) targets
  *hand-editing via the data-type editor* while the system is configured.
  Program assignment + `ArcRefresh` is the ABB-documented route — it is what
  `ArcRefresh` exists for.

### 2.4 THE VC ITSELF — inspected on disk 2026-08-28 (this is the new evidence)

Installed options (from System Properties): RobotWare **6.15.08.00**;
Control Module: **616-1 PC Interface**, **841-1 EtherNet/IP Scanner/Adapter**,
**604-1 MultiMove Coordinated**, **623-1 Multitasking**; Drive Module:
**633-4 Arc**, **Fronius TPS**, **657-1 SmarTac IO version**, EtherNet/IP.

**(a) `welddata` component set — RESOLVED, it is the `main_arc` shape, not the
flat trio.** From the controller's own data descriptor
(`…\RobotWare_6.15.8029\options\arc\RS\RapidDataDescriptors\welddata.xml`):

```
welddata := [ weld_speed, org_weld_speed, main_arc, org_arc ]
   where main_arc / org_arc are arcdata :=
        [ sched, mode, voltage, wirefeed, control, current,
          voltage2, wirefeed2, control2 ]
```
Default literal (`welddata_Default.xml`):
`[10,10,[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0]]` — matching the community
example `[4.5,0,[1,0,24.75,10.5,0,266,0,0,0],[...]]`. The `org_*` twins hold the
pre-tuning originals for the FlexPendant tuning function's reset.
(ABB's "components depend on configuration" caveat is about which are *used* by
a given welder; the names and the record shape are now known.)

**(b) `seamdata` — same treatment** (`seamdata.xml`): 16 top-level slots —
`purge_time, preflow_time, ign_arc(arcdata), ign_move_delay, scrape_start,
heat_speed, heat_time, heat_distance, heat_arc(arcdata), cool_time, fill_time,
fill_arc(arcdata), bback_time, rback_time, bback_arc(arcdata), postflow_time`.
Default: `[0.2,0.05,[9×0],0,0,0,0,0,[9×0],0,0,[9×0],0.1,0,[9×0],0.05]`.
`weavedata` default: `[1,0,4,4,0,0,0,0,0,0,0,0,0,0,0]`.

**(c) `ArcLStart` signature and the PERS rule — RESOLVED**
(`MoveInstructionDescriptions\ArcLStart.xml`). Parameter order and access mode:

| Param | Type | Optional | accessMode |
|---|---|---|---|
| ToPoint | robtarget | no | In |
| \ID | identno | yes | In |
| Speed | speeddata | no | In |
| **Seam** | **seamdata** | no | **Persistent** |
| \AdvData | AdvSeamData | yes | Persistent |
| **Weld** | **welddata** | no | **Persistent** |
| \Weave | weavedata | yes | Persistent |
| Zone | zonedata | no | In |
| Tool | tooldata | no | Persistent |
| \WObj | wobjdata | yes | Persistent |
| \Corr / \Track | switch / trackdata | yes | In / Persistent |
| \SeamName, \T1..\T7, \TLoad, \FlyStart | … | yes | |

**Seam, Weld, Weave, Track, Tool and WObj are `Persistent` arguments** — the
same rule that produced finding F-3 (`CRobT`'s `\Tool`/`\WObj`). So the weld
data **must** be `PERS` (or a `PERS` parameter); a `VAR` or an expression will
raise "not a persistent reference". This is a hard constraint on the design, and
it independently confirms §3.1's conclusion.

**(d) Fronius: RW 6.15 ships TWO different equipment classes, and the VC has
the wrong one for our cell.**

| | `FroniusTPS4/5000` (`FRONIUS_EQUIP_IO`) | `FroniusTPS/i` (`FRONIUS_TPSi_EQUIP_IO`) |
|---|---|---|
| PWS folder | `pws\froniustps4k5k` | `pws\FroniusTPSi` |
| FeedReference | `aoFr1Power` (power/synergic setpoint) | **`aoFr1WFSpeed` (wire feed speed)** |
| VoltReference | `aoFr1ArcLength` | `aoFr1ArcLength` |
| ControlPort | `aoFr1Dynamic` | `aoFr1Dynamic` |
| JobPort | `goFr1JobNum` + `ProgramPort goFr1PrgNum` | `goFr1JobProgNum` |
| ModePort | `goFr1Mode` | `goFr1Mode` |
| TcpReference | present | absent |

Our cell is a **TPS 500i**, which is the `FroniusTPS/i` class.

**⚠ The TPS/i class is still not the one installed — and the reason is a
documented option-key override.** After the option was changed from "Fronius
TPS" to "Fronius TPS/i" (2026-08-28), the VC's
`HOME\Arc\ConfigTemplates\` gained **`Fronius_EIP`** (and kept
`FroniusTPS4K5K`) but **not `FroniusTPSi`** — and `Fronius_EIP` is the *TPS
4000/5000 family over EtherNet/IP* (`FRONIUS_EQUIP_IO`,
`FeedReference "aoFr1Power"`), not the TPS/i class. The proof is that
`pws\FroniusTPSi\install.cmd` does
`mkdir -path $HOME/Arc/ConfigTemplates/FroniusTPSi`, and that folder does not
exist.

The cause is in `options\arc\weldequip\pws\install_PWS.cmd`, verbatim:

```
# Check if Fronius TPS Integrated is selected, if so, set $ANSWER to FRON_EIP
getkey -id "FronIntegr1" -var 10 -strvar $MY_ANSWER -errlabel NEXT
setstr -strvar $ANSWER -value "FRON_EIP"
#NEXT
```

i.e. the **"Fronius TPS Integrated" key (`FronIntegr1`) silently overrides the
power-source selection to `FRON_EIP`**, whatever was chosen. The selection key
itself is `PSInAwTask1` ("power source in arc welding task 1"), whose values are
`ARISTOMIGINT | ESAB_W8 | F4500MW | FRON_EIP | FRON_TPSI | RPC_RPC | RPC_INT |
SKS | STDIOWELD | SIMWELD | EXTWELD`; the fieldbus is a separate key `PSFbus1`
(DNet / EIP / PNET, else `VIRT`).

**To get the TPS/i class**: select the TPS/i power source **and clear the
"Fronius TPS Integrated" checkbox** (they are mutually exclusive in effect).
Verify by the folder, not the option list: `HOME\Arc\ConfigTemplates\FroniusTPSi`
must appear, and the PROC config must show `-welder_type "FroniusTPS/i"` with
`FeedReference "aoFr1WFSpeed"`.

**(d2) There is a built-in "Simulated Welder" — the clean answer for VC runs.**
`pws\SimWeld\Cfg\proc\pARC1_EQUIP_ROB1.cfg` declares
`-welder_type "Simulated Welder"` with `-arc_eq_class "awEquipSTD"`, and all of
its supervision inputs are **simulated** signals:
`-ArcEst "siR1ArcEst" -WaterOk "siR1WaterOk" -GasOk "siR1GasOk"` (the `si`/`so`/
`sao` prefix marks simulated I/O, bound to no physical device). So an arc program
runs to completion with no welder and no fieldbus — this is very likely what the
forum's vague "block weld" advice amounts to on a VC.
Caveat worth knowing: SimWeld's `ARC_EQUIP_IO_AO` exposes only `VoltReference`
and `FeedReference` — **no `ControlPort`** — so `main_arc.control` (the HMI's Arc
Control) is not exercised under SimWeld. Hence the two-track plan in §4.

**(e) The HMI's four fields map 1:1 onto the TPS/i I/O — including Arc
Control**, which FANUC never applied:

| HMI field | welddata component | Fronius TPS/i signal |
|---|---|---|
| Travel Speed (IPM) | `weld_speed` | (robot motion, not a welder signal) |
| Wire Speed (IPM) | `main_arc.wirefeed` | FeedReference `aoFr1WFSpeed` |
| Arc Length | `main_arc.voltage` | VoltReference `aoFr1ArcLength` |
| Arc Control | `main_arc.control` | ControlPort `aoFr1Dynamic` |
| Proc # | `main_arc.sched` | JobPort `goFr1JobProgNum` |
| — | `main_arc.mode` | ModePort `goFr1Mode` |

Note the naming: ABB calls the field `voltage`, but for Fronius it is wired to
**arc length** — the mirror image of the FANUC `$CMD_VOLTS`/`$CMD_WFS`
mislabeling documented in the earlier research doc. **Map by intent, verify by
signal name**, exactly as that doc concluded.

**(f) Wire feed can be carried in IPM natively — no conversion needed.** The
Fronius PWS ships `SetWF_to_IPM_ROB1.cfg` (and `SetWF_to_MPM_*`); the ReadMe
says *"Default WireFeed unit is mm/s … SetWF_to_IPM_ROB1.cfg, scaling will be
set to Inches/min"*. The IPM file rescales both `aoFr1WFSpeed` and
`aiFr1WireFeed_M` (`MaxLog 12900.39`). Since the HMI sends **IPM**, installing
that cfg makes `main_arc.wirefeed` accept the HMI value **verbatim** — deleting
a whole class of conversion bugs. Travel speed still needs IPM→mm/s
(`weld_speed` is RAPID speed, ×0.42333).

**(g) Options that change earlier conclusions:**
- **623-1 Multitasking is installed** → the §3.4 Tier-2 caveat ("HMI-driven
  mid-weld tuning would need Multitasking, out of scope") is now a *capability
  question, not a licensing one*.
- **657-1 SmarTac** → the deferred touch-sense family (R_TS_*) has its option.
- **604-1 MultiMove Coordinated** → available for positioner work later.
- **841-1 EtherNet/IP** satisfies the TPS/i manual's "one industrial network"
  prerequisite, and the TPS/i PWS has an EtherNet/IP device config.
- ⚠ **637-1 Production Screen** is **not** in the listed options, though
  `HOME\ProdScr\config\ProductionSetup_Arc.xml` exists (Arc app
  `TpsViewArc.dll`). The TPS/i manual lists 637-1 as a prerequisite — confirm
  whether it is implied by 633-4 here, and make sure it is quoted for the real
  cell.

---

### 2.5 First VC run + controller arc log (2026-08-31 07:10)

**(a) Shapes confirmed.** Step 1 of `TGArcCheck.mod` compiled and printed
`weld_speed=8.89`, `main_arc.wirefeed=520`, `main_arc.voltage=4.9`,
`purge_time=0.2`, `postflow_time=0.05`. §2.4a/b are therefore confirmed against
this controller, and the three component paths `TG_ApplyWeldParams` needs are
proven writable.

**(b) The configured welder is STILL not TPS/i.** The controller logs its own
answer at every start in `<VC>\INTERNAL\arcLog_T_ROB1.log`:

```
Found ARC1 WelderType: FronTPSInt EquipmentClass: EIP_awEqFrTPS4K5K
Loaded: RELEASE:/options/arc/WeldEquip/Code/EIP_awEqFrTPS4K5K.mod
```

`FronTPSInt` = "Fronius TPS **Integrated**" → the `FronIntegr1` override of
§2.4d, landing on the TPS 4000/5000 EtherNet/IP class. Corroborated on disk:
`HOME\Arc\ConfigTemplates\` has `Fronius_EIP` + `FroniusTPS4K5K`, **no
`FroniusTPSi`** (whose installer would create that folder). **The arc log is the
authoritative check from now on** — cheaper and less ambiguous than the option
list.

**(c) NEW — the arc system has a configurable unit system.** The log shows
`GetCfgDataStr: units = SI_UNITS`, referring to `PROC/ARC_UNITS`. RobotWare ships
three instances (`arcbase\config\proc\pARC_UNITS.cfg`), and the type allows more
(`addRemoveInstances="true"`):

| ARC_UNITS | `arc_length_units` | `arc_velocity_units` | `arc_feed_units` |
|---|---|---|---|
| `SI_UNITS` *(active)* | mm | **mm_s** | **mm_s** |
| `US_UNITS` | inch | **ipm** | **ipm** |
| `WELD_UNITS` | mm | mm_s | m_min |

**This is a design-relevant discovery.** `US_UNITS` is exactly the HMI's
convention — it sends travel speed and wire speed both in IPM. Under `US_UNITS`,
`TG_ApplyWeldParams` becomes a straight copy with **no unit conversion at all**,
which is strictly less error-prone than ×0.42333 on travel speed plus a decision
about wire feed. It also supersedes the §2.4f `SetWF_to_IPM` I/O-scaling route:
`ARC_UNITS` acts at the welddata level and covers *both* fields, whereas the
signal-scaling cfg covers only wire feed. ⚠ Must confirm that `ARC_UNITS` changes
the numeric interpretation of `welddata`, not merely the FlexPendant display —
that is what the §4 Phase-A timing measurement decides.

**(d) Why arc motion ran with no welder.** The equipment prop has
`override_on = TRUE` and **`autoinhib_on = TRUE`**, and the inhibit I/O
(`DI WeldInhib`, `DI WeaveInhib`, `DI TrackInhib`, `DO AWBlock`) is **unassigned**.
Auto-inhibit means the process self-inhibits when the equipment is unavailable,
so `ArcLStart`/`ArcLEnd` degrade to plain motion. This is the concrete mechanism
behind the ABB forums' vague "block weld" advice (§2.1's last ⚠), and it means
the VC needs no special trick to run weld programs. `WeldInhib` is also the named
signal to wire if we ever want *commanded* dry-run blocking — a better ABB
analogue for FANUC's `DRY_RUN_ON`/`OFF` than the empty `TG_DryRunOn`/`Off`
placeholders currently in `TG_Cell.sys`.

**(e) Still unverified (instrumented in module v2).** That
`welddata.weld_speed` governs the weld speed — the v1 log had no timestamps — and
which optional components exist, since v1's step-3 message printed identically
whether or not the probes were uncommented. Both are now measured/read back.

## 3. Recommended design

### 3.1 Weld data lives on the controller, NOT in the .tgs program

**Do not define weld data in the .tgs module.** Keep FANUC's shape — global at
the controller:

1. **HMI mode makes .tgs-resident values dead data.** Parameters arrive over the
   wire per weld (R_W_P) at runtime; the program needs only a *reference*.
2. **Dynamic-load lifecycle**: a `PERS` in a `Load \Dynamic` module loses its
   runtime updates on a *plain* `UnLoad` — silently, since nothing warns.
   FlexPendant tuning during a run would vanish at end of cycle, while
   controller-resident `.sys` data survives and is saved with the system.
   *(Corrected 2026-08-31: the earlier "never written back to the .mod" was too
   strong. `UnLoad \Save` and `Save` do write back, and PERS current values are
   folded into their declarations on any module save. The narrow true statement
   is the one above — plain `UnLoad` discards. Full analysis, and what it means
   for operator touch-ups, in
   [abb_program_touchup_and_retrieval_v1.md](abb_program_touchup_and_retrieval_v1.md).)*
   Retrieval adds a second, harder reason to keep PERS out of the .tgs: a saved
   module serializes current PERS values, so a PERS-carrying .tgs would produce a
   different byte image on every save and the HMI's byte-compare would re-push the
   program every run (that doc §6.1).
3. **The instruction demands PERS anyway** (§2.4c) — and the natural home for a
   long-lived `PERS` that the pendant's tuning UI also edits is a system module,
   not a module that is unloaded after every cycle.
4. **Namespace rule** (plan §2.8): consecutive .tgs modules redeclaring the same
   globals is a collision class we already engineered away once.
5. Both repos already point this way (FANUC schedules live in controller `.VR`
   files; the planner tracker's Phase 7 says "weld data resident on the
   controller").

The .tgs carries only the *selection* and the weld motion targets — the exact
analog of `WELD START[proc,sched]`.

### 3.2 New module `TG_Weld.sys` (not TG_Comms)

Separate on purpose: `welddata`/`seamdata`/`ArcL*` exist **only with option
633-4** — folding them into `TG_Comms.sys` would make the comms layer unloadable
on a non-Arc system and break the existing non-Arc VC tests. `TG_Comms` stays
option-independent.

```
! TG_Weld.sys  (SYSMODULE, requires 633-4 Arc)
PERS seamdata sdTG_Weld := [0.2,0.05,[0,0,0,0,0,0,0,0,0],0,0,0,0,0,
                            [0,0,0,0,0,0,0,0,0],0,0,[0,0,0,0,0,0,0,0,0],
                            0.1,0,[0,0,0,0,0,0,0,0,0],0.05];   ! RW6.15 defaults (§2.4b)
PERS welddata wdTG_Weld := [10,10,[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0]];
PERS welddata wdTG_Lib{10} := [ ... ];    ! predefined recipe library (AWE1WPxx analog)
                                          ! index = HMI proc number; grows later

PROC TG_ApplyWeldParams(num nProcDefault)
  ! THE single mapping point — all brand/config specifics live here.
  IF nTG_UdwpFlag=1 THEN
    wdTG_Weld := wdTG_Lib{nTG_WeldProc};              ! base recipe, then override
    wdTG_Weld.main_arc.wirefeed := nTG_WireFeed;      ! IPM verbatim if SetWF_to_IPM installed (§2.4f)
    wdTG_Weld.main_arc.voltage  := tgClamp(nTG_ArcLength);   ! = arc length on Fronius (§2.4e)
    wdTG_Weld.main_arc.control  := tgClamp(nTG_ArcControl);  ! = dynamic correction (beyond FANUC parity)
  ELSE
    wdTG_Weld := wdTG_Lib{nProcDefault};              ! planner-emitted recipe
  ENDIF
  wdTG_Weld.weld_speed := nTG_TravelSpeed * 0.42333;  ! IPM -> mm/s; ALWAYS (FANUC always wrote $CMD_WSPEED)
ENDPROC
```

Notes:
- `nProcDefault` is emitted by the planner (the `WELD START[<proc>,…]` analog),
  so UDWP=0 has exact FANUC parity with no hidden state; UDWP=1 uses the HMI's
  proc, matching `WELD START[R[171],20]`.
- Copy-by-value (`wdTG_Weld := wdTG_Lib{n}`) is deliberate here — a controlled
  snapshot at apply time, the opposite of the F-2 stale-frame hazard (which was
  about *implicit* copies of data a request later updates).
- Leave `org_weld_speed` / `org_arc` alone: they are the tuning function's
  originals. ⚠ VC check that pendant tuning still behaves after we write
  `main_arc`.
- `main_arc.sched` / `.mode`: only needed if the cell drives the TPS/i by job or
  needs an explicit mode; decide with the cell's operating mode (§5).
- **Clamping matters**: the HMI GUI allows 0–9999 while Fronius corrections are
  ±10 steps / 0.1 increments — and the HMI's *default* arc length is 49.0, which
  is out of range on day one. Clamp robot-side so the HMI needs no change.

### 3.3 The two-weld .tgs shape

Per weld (replacing the comment block; targets in `wobjTG_Weld`):

```
stTG_SubName:="PWeld2";
TG_ReqWeldFrame \Tool:=tTG_Weld \WObj:=wobjTG_Weld;   ! R_W_F
TG_CamClose;
IF nTG_WeldStatus=2 GOTO abort_end;
IF nTG_WeldStatus=1 THEN
  MoveJ pW2Approach,v100,z100,tTG_Weld\WObj:=wobjTG_Weld;   ! J P[121]
  MoveL pW2Near,v200,z10,tTG_Weld\WObj:=wobjTG_Weld;        ! L P[122]
  TG_ReqWeldParams;                                          ! R_W_P
  TG_ApplyWeldParams 1;                                      ! proc default from planner
  ArcLStart pW2Start,v200,sdTG_Weld,wdTG_Weld,fine,tTG_Weld\WObj:=wobjTG_Weld;  ! L P[123] + WELD START
  ArcLEnd   pW2End,  v200,sdTG_Weld,wdTG_Weld,fine,tTG_Weld\WObj:=wobjTG_Weld;  ! L P[124] + WELD END
  ! (R_W_S here on FANUC — deferred, §6)
  MoveL pW2Depart,v200,fine,tTG_Weld\WObj:=wobjTG_Weld;     ! L P[125]
ENDIF
```

- Argument order matches the verified signature (§2.4c):
  `ToPoint, Speed, Seam, Weld, Zone, Tool \WObj`.
- **Mapping rationale**: `ArcLStart` *moves to* the weld start point and ignites
  there; `ArcLEnd` runs the weld pass at `wdTG_Weld.weld_speed` and finishes
  with the seamdata end phase — a 1:1 replacement for
  `L P[123]` + `WELD START` + `L P[124] R[175]inch/min` + `WELD END`. The `v200`
  governs non-welding motion only. ⚠ VC: confirm the ArcLStart approach speed
  and that `v` is ignored during the weld itself.
- Second weld (`PWeld3`) is the same block with its own targets — exercising the
  per-weld R_W_F/R_W_P/apply cycle twice per run, like the sample.
- Recommend a **new** sample `abb/rapid/TGS/TD05Weld.mod` (PROC `TD05Weld`)
  rather than editing `TD05Test.mod`: TD05Test stays the comms regression
  program that runs on a non-Arc system. The Python server already copies
  whatever program name is requested.

### 3.4 "Update weld data on-the-fly" — two tiers

- **Tier 1 (= the FANUC behavior; in scope now):** *per-weld* runtime update.
  The operator edits Wire Speed / Travel Speed / Arc Length (+ proc, + UDWP) in
  the HMI; next time the weld runs, R_W_P delivers the values and
  `TG_ApplyWeldParams` lands them in `wdTG_Weld` **before** `ArcLStart`. No
  program regeneration — the whole FANUC `SET_VAR AWE1WP*` dance reduced to a
  few assignments.
- **Tier 2 (beyond FANUC parity; design for it, defer):** *mid-weld* update via
  trap + `ArcRefresh`. Because the active data is a single `PERS welddata`,
  adding it later is: `CONNECT`/`ITimer` (or `IPers`), trap writes components,
  `ArcRefresh;`. Two honest caveats: (a) FANUC offered nothing like this, so it
  is an improvement, not parity; (b) an *HMI-driven* mid-weld change needs a
  background task, because the socket request PROCs block during motion —
  **623-1 Multitasking is installed** (§2.4g), so this is now a design decision
  rather than a licensing blocker. Still out of scope for this phase.
- **Free extra**: FlexPendant welddata tuning writes the same named `PERS`, so
  pendant tweaks coexist with the HMI path (last writer wins at apply time —
  FANUC-equivalent).

### 3.5 Weld Planner selection design for ABB

FANUC selection = bind weld → library row (identity: Procedure #/Schedule #);
the planner emits the numbers into `WELD START[p,s]`. The planner repo has
already decided the container: **per-brand controller-binding bucket alongside
the FANUC columns**. Recommended content:

```json
{"fanuc": {"procedure_number": 7, "schedule_number": 12},
 "abb":   {"welddata": "wdTG_Lib7", "seamdata": "sdTG_Weld", "recipe_index": 7}}
```

- **Keep a numeric `recipe_index`, defaulting to the FANUC procedure number.**
  (a) The HMI wire carries a 2-char proc and its spinner is 1–99, so this means
  **zero HMI changes** and byte-identical R_W_P transcripts; (b) the library
  census (§2.3) shows the row is an identity, not a parameter store; (c)
  `wdTG_Lib{n}` makes index→welddata resolution one array access; (d) it maps
  onward to `main_arc.sched` if the cell ever runs in job mode.
- **Also store the names** (`welddata`/`seamdata`): the planner's **no-HMI** ABB
  mode (its Phase 7 — `AbbTranslator.configure_controller_data` is already
  waiting for exactly these) emits named data directly into
  `ArcLStart ..., sdX, wdY, ...`. The convention `wdTG_Lib{n}` serves both: the
  *name* for no-HMI emission, the *index* for the HMI wire.
- **Schedule number has no ABB meaning** — FANUC's `[proc, sched]` pair collapses
  to one recipe identity. The HMI's read-only "Sched #" and the hardcoded SCH 20
  disappear naturally: the "custom schedule slot" analog *is* `wdTG_Weld`.
  Planner UI should show Procedure #/Schedule # for FANUC only (capability
  gating already exists there).
- For the **HMI-mode ABB .tgs**, the planner's weld blocks provider (an
  `AbbProgramBlocksProvider` does not exist yet — verified) will eventually emit
  the §3.3 shape with `TG_ApplyWeldParams <recipe_index>;`. Until then, this
  repo hand-maintains `TD05Weld.mod` as the executable spec of what that
  provider must produce — the same role the FANUC sample plays today.
- The **commissioning burden moves, not grows**: FANUC needed `AWE1WPnn.VR`
  schedules on the controller; ABB needs `wdTG_Lib{n}` values in `TG_Weld.sys`
  plus the power-source characteristics/jobs.

---

## 4. VC validation plan (pass criteria)

**Two tracks, because they answer different questions (§2.4d/d2):**

- **Track S — "Simulated Welder" (`SIMWELD`)**: proves the *choreography* —
  RAPID compiles, `ArcLStart`/`ArcLEnd` execute as motion, `welddata`
  assignment works, the two-weld cycle runs. No welder, no fieldbus, no
  supervision stalls. **Recommended for Phases B and C**, because it removes
  every variable that is not our code.
- **Track F — Fronius TPS/i (`FRON_TPSI`, "Fronius TPS Integrated"
  unchecked)**: proves the *parameter semantics* — that `main_arc.wirefeed`
  reaches `aoFr1WFSpeed`, `main_arc.voltage` reaches `aoFr1ArcLength`, and
  `main_arc.control` reaches `aoFr1Dynamic` (Arc Control is **not** available
  under SimWeld). Needed before we can claim real-cell fidelity; the DI side
  (`diFr1ArcStable`, `diFr1WelderReady`, `diFr1MainCurrent`, `diFr1HeartBeat`)
  must be forced via RobotStudio I/O simulation — we now have the exact names.

**Phase A — get one of the two installed and read the shape off the VC.**
For Track F also add `SetWF_to_IPM_ROB1.cfg` if wire feed is to be commanded in
IPM (§2.4f). Then confirm on the running VC:
1. the equipment class actually installed (`HOME\Arc\ConfigTemplates\<name>`
   plus the PROC config's `-welder_type`) — do not trust the option list alone,
   given the `FronIntegr1` override;
2. that `welddata` on the *configured* system shows the §2.4a components (the
   descriptor is the generic Arc shape; a configured welder may hide unused
   ones — SimWeld notably lacks `ControlPort`);
3. ⚠ whether an explicit weld-blocking toggle exists in the RobotWare Arc
   FlexPendant menu — now a nice-to-have rather than a blocker, since SimWeld
   removes the need for it.
THE SCRATCH MODULE FOR THIS IS WRITTEN:
[abb/rapid/TGArcCheck.mod](../abb/rapid/TGArcCheck.mod) - standalone (no TG_*
dependency, tool0/wobj0), carrying the 2.4a/b literals, the three component
assignments TG_ApplyWeldParams will make, an ArcLStart/ArcLEnd pair, and a
commented probe block for the optional components. Run instructions and pass
criteria: [robotstudio_setup.md](robotstudio_setup.md) section 14.

Pass: TGArcCheck.mod program-checks clean (the PERS declarations ARE the shape
test), TGArcCheck prints the expected values, and TGArcMoveCheck completes
without an ignition-supervision stop - with the weld segment visibly running at
weld_speed (~34 s for 300 mm at 8.89 mm/s) rather than at the v200 argument.

**Phase B — `TG_Weld.sys` + apply mapping.** Add the module with the §2.4a
component names; new fake-robot expectations: HMI defaults 21 IPM travel /
520 IPM WFS must appear in RAPID as `weld_speed = 8.89` mm/s and
`main_arc.wirefeed = 520` (IPM verbatim, or 13.21 if the m/min scaling is kept)
— verified by TPWrite of `wdTG_Weld` after `TG_ApplyWeldParams`, numerically, in
the usual transcript loop.

**Phase C — `TD05Weld.mod`, two welds end-to-end.** Full cycle on the Arc VC
with welding blocked: transfer → load → run; Python serves two R_W_F/R_W_P
rounds (one UDWP=1, one UDWP=0 — both branches in one run). Pass criteria:
(1) both arc segments execute as motion; (2) the transcript shows two complete
weld request sequences byte-compatible with the FANUC choreography; (3)
TPWrite'd `wdTG_Weld` matches the served values per Phase B; (4) the UDWP=0 leg
shows `wdTG_Lib{n}` values with only `weld_speed` overridden; (5) a second cycle
repeats clean (load/unload hygiene unchanged).

**Phase D (optional, deferred) — mid-weld `ArcRefresh` demo** behind a flag:
timer trap ramps `weld_speed` during the pass. Explicitly beyond FANUC parity.

## 5. What must come from the customer / cell (updated)

Most of the old list is now **answered by the VC** (§2.4). What genuinely
remains:

1. **The recipe catalog** (new): mapping of planner/HMI proc numbers 1–99 to
   Fronius characteristics (or jobs) and the seed values for `wdTG_Lib{n}`.
2. **Operating mode** on the real power source (characteristics/synergic vs job)
   — decides whether `main_arc.sched`/`.mode` need driving, and whether the HMI
   proc number is a characteristic or a job number.
3. **Real-cell option key** must carry what the VC has: 633-4 Arc, a fieldbus,
   **and ⚠ 637-1 Production Screen** (TPS/i manual prerequisite, not in the VC's
   listed options).
4. **Fronius-side RI-FB inside/i variant** configured for ABB (the FANUC cell's
   card is the FANUC variant) + TPS/i firmware version.
5. Wire-feed unit decision for the real cell (IPM scaling cfg vs mm/s default) —
   we now know it is a one-file choice.

## 6. Known gaps carried forward (deliberate)

- **R_W_S (id 13, weld stats)**: called after every FANUC weld; its KAREL source
  is **not in `resources/FANUC/KAREL/`** (verified) and it stays out of scope.
  The §3.3 block leaves a marked slot.
- **Arc Control**: FANUC never applied it (hidden in the HMI GUI, always 0.0,
  SET_VAR commented out). On ABB it now has an obvious home
  (`main_arc.control` → `aoFr1Dynamic`), so applying it is a cheap improvement —
  gated on clamping (§3.2).
- **Weave**: not in the FANUC sample; `\Weave:=weavedata` slots into the §3.3
  instructions later without structural change (`weavedata` default in §2.4b).

## 7. Sources

- **VC / RobotWare 6.15.8029 on disk (primary evidence, §2.4)**:
  `options\arc\RS\RapidDataDescriptors\{welddata,seamdata}.xml`;
  `options\arc\RS\RapidDataDefaults\{welddata,seamdata,weavedata}_Default.xml`;
  `options\arc\RS\MoveInstructionDescriptions\ArcLStart.xml`;
  `options\arc\weldequip\pws\FroniusTPSi\Cfg\{proc\pARC1_EQUIP_ROB1.cfg,
  eio\SetWF_to_IPM_ROB1.cfg}` and its `install.cmd`;
  `options\arc\weldequip\pws\froniustps4k5k\Cfg\proc\pARC1_EQUIP_T_ROB1.cfg`;
  `options\arc\weldequip\pws\Fronius_EIP\Cfg\proc\EIP_pARC1_EQUIP_PROP_T_ROB1.cfg`;
  **`options\arc\weldequip\pws\install_PWS.cmd`** (the `PSInAwTask1` /
  `FronIntegr1` selection logic — §2.4d);
  `options\arc\weldequip\pws\SimWeld\Cfg\proc\pARC1_EQUIP_ROB1.cfg` (§2.4d2);
  VC `HOME\Arc\ConfigTemplates\{FroniusTPS4K5K,Fronius_EIP}\ReadMe.txt`;
  VC `HOME\ProdScr\config\ProductionSetup_Arc.xml`.
- In-repo: `TD05tRJYQd.ls`, `TG_Comms.sys`, `TD05Test.mod`,
  [abb_weld_params_research_v1.md](abb_weld_params_research_v1.md),
  [abb_port_plan_v1.md](abb_port_plan_v1.md).
- TGuideWeldingHMI: `WidgetTouchupOffsets.cpp/.h`, `WeldLibrary.cpp/.h`,
  `RobotCell.cpp` (~2505), `FANUCRobot.cpp` (491–537).
- curobo_suite: `Translators/AbbTranslator.py`,
  `robot_brand_program_blocks/fanuc_program_blocks.py` (52–127),
  `workflows/weld_planner_rows.py` (739), `workflows/weld_library_schema.py`,
  `docs/abb_integration_plan_v1.md` (§5.4, Phase 7), `docs/
  robot_brand_agnostic_abb_support_analysis_v1.md` (§5.7, §9.2),
  `docs/vanilla_installer_autoupdate_v1.md` (E28 field census).
- [Runtime welddata modification + ArcRefresh (ABB community)](https://tech-community.robotics.abb.com/discussion/11721/i-would-like-to-change-wire-feed-rate-and-speed-dynamically-in-program-while-welding-is-in-progress)
- [ArcRefresh manual pages (RobotWare-Arc manual 3HAC16591, pp. 160–161)](https://www.yumpu.com/en/document/view/29697239/application-manual-technology/160)
- [Weld blocking on the VC (ABB staff comment)](https://tech-community.robotics.abb.com/discussion/3866/how-to-simulate-a-weld-arcwelding-powerpac)
- [welddata vs seamdata roles; weld_speed governs the weld (ABB community)](https://forums.robotstudio.com/discussion/8802/welddata-and-seamdata)
