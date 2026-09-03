# R_W_P → RobotWare Arc weld-data research — v1 (facts, no code)

Status: **research complete 2026-08-28**, updated the same day with the
cell's confirmed power source (**Fronius TPS 500i /600V/nc**, §3.1) — Phase 4
item "RobotWare Arc mapping for R_W_P" of
[abb_port_plan_v1.md](abb_port_plan_v1.md) §5. Wire-level
handling of the request is **already done and VC-validated** on both sides
(Phase 2): `TG_ReqWeldParams` receives UDWP flag / travel speed / welder type /
proc / wirefeed / arc length / arc control into `PERS` data, and the HMI side
(`RobotCell.cpp` `RequestWeldingParameters` → `FANUCRobot::
SendUserDefinedWeldingScheduleParameters`) sends them. This document is the
verified know-how for the *application* layer — how those values become live
weld parameters in RAPID — plus what can only come from the customer's cell.
Per team decision, applying the data is a separate future task.

Everything below is sourced (links in §7); items marked ⚠ could not be fully
verified from public material and need the manual on the quoted RobotWare
system or a team/vendor confirmation.

## 1. What the FANUC side actually does (verified from `r_w_p.kl`)

For UDWP=1, after storing the registers, the KAREL writes into the ArcTool
weld-procedure file selected by the received proc number
(`AWE1WP<proc>`, e.g. `AWE1WP05` — a `.VR` weld-schedule file), schedule 20 —
matching the `.ls` weld line `WELD START[R[171],20]`:

| HMI field (GUI label, units) | Register | SET_VAR target (`AWE1WP<proc>` `SCH[20]`) |
|---|---|---|
| Travel Speed (IPM) | R[175] | `$CMD_WSPEED` |
| Wire Speed (IPM) | R[172] | **`$CMD_VOLTS`** |
| Arc Length (unitless trim) | R[173] | **`$CMD_WFS`** |
| Arc Control (unitless, 0.1 steps) | R[174] | *nothing* — the SET_VAR is commented out ("HARDCODED TO ZERO IN PREDEFINED SCHEDULE") |

The Miller (welder_type 1) and FroniusTPSi (welder_type 2) branches are
byte-identical. UDWP=0 resets R[171..174] and sends only travel speed.

**The apparent wirefeed↔arclength "swap" is (very likely) deliberate.**
Robot-forum users document that in ArcTool (observed on 6.4 and confirmed
still on 7.7) the schedule variables are **mislabeled in the firmware**:
`$CMD_VOLTS` *actually controls WFS* and `$CMD_WFS` *actually controls Trim*
(verified by them for `$AWESCH[1,34]`/`[1,35]` rows; they advise trusting
tested behavior over the variable descriptions). Read through that lens, the
KAREL's crossed writes land exactly right:

| HMI field | written to | actually controls (per forum) |
|---|---|---|
| Wire Speed | `$CMD_VOLTS` | **wire feed speed** ✓ |
| Arc Length | `$CMD_WFS` | **trim / arc length correction** ✓ |

~~⚠ confirm with whoever commissioned the FANUC cell~~ — **resolved in-house
2026-08-28**: `RobotCell.cpp` (~820-846, the read-back path that regex-parses
the controller's schedule dump) documents the semantics per Fronius operating
mode in a comment block:

```
// IN SPECIAL 2-STEP MODE          // IN JOB MODE
// $CMD_VOLTS :  WIRE FEED SPEED   // $CMD_VOLTS :  JOB NUMBER
// $CMD_WFS   :  ARC LENGHT CORRECTION (TRIM)
// $CMD_WSPEED:  TRAVEL SPEED      // $CMD_WSPEED:  TRAVEL SPEED
// $CMD_TIME  :  DELAY TIME        // $CMD_TIME  :  DELAY TIME
```

Two facts follow, both load-bearing for the ABB port: (a) the crossed writes
are intentional — HMI Wire Speed really is WFS and HMI Arc Length really is
arc-length correction/trim, exactly as the forum's mislabeling report
predicts; (b) **the FANUC cell drives the Fronius in _Special 2-step_ mode,
not Job mode** — in Job mode `$CMD_VOLTS` would carry a job number instead.
The Miller branch uses byte-identical regexes, so the same crossed variables
serve both brands and the difference is purely in welder-side interpretation.
**Consequence for ABB: map by INTENT (wirefeed→wirefeed, arc length→arc-length
correction), not by copying the FANUC variable names.**

## 2. RobotWare Arc — how weld parameters work on ABB (verified)

- **Option**: RobotWare 6 option **633-4 Arc** (651-1 "Additional Arc Systems"
  for multi-system cells). Manual: *Application manual — Arc and Arc Sensor*,
  3HAC050988 (RW6); *ArcWare for OmniCore*, 3HAC084370 (RW7).
- **No controller-table gymnastics.** Unlike FANUC's schedule files +
  `SET_VAR`, ABB weld parameters are **ordinary RAPID data** passed to each
  weld instruction: `ArcLStart`/`ArcL`/`ArcLEnd` (and `ArcC*`) take
  `seamdata` (ignition/start + end/fill phases), `welddata` (the phase with
  the arc established), and optional `weavedata`. *"Welddata is used to
  control the weld during the weld phase, that is, from when the arc is
  established until the weld is completed"* (ABB community).
- **Runtime modification is plain assignment on a `PERS`** — the direct
  analog of the FANUC SET_VAR, but simpler: `wd.weld_speed:=nSpeed;
  wd.main_arc.wirefeed:=nWireFeed;` before `ArcLStart` (ABB forum, verbatim
  pattern). Mid-weld changes use an interrupt + `ArcRefresh`. This slots
  perfectly under our design: `TG_ReqWeldParams` already lands the values in
  `PERS num`s; an apply-PROC copies them into a `PERS welddata` that the .tgs
  weld instructions reference.
- **The welddata component set is NOT fixed** — it materializes from the
  cell's Arc Equipment / process configuration: *"some of the components of
  welddata depend on the configuration of the robot; if a given feature is
  omitted, the corresponding component is not present"* (e.g. `weld_voltage`
  exists only if weld voltage is defined). Public sources show two shapes:
  flat `weld_speed`/`weld_wirefeed`/`weld_voltage` (RW6 generic arc system,
  the trio the welddata *tuning* function exposes) and `weld_speed` +
  `main_arc.<wirefeed|voltage|...>` (arcdata substructure). ⚠ The concrete
  component names for our cell can only be read off the quoted controller's
  configuration — this is the part that genuinely needs customer info.
- **`weld_speed` governs TCP speed while welding** (*"the weldspeed stored in
  the start [welddata] will be used for the complete weld"*), so the HMI's
  travel speed maps to `welddata.weld_speed`, not to the instruction's `v`
  argument. ⚠ confirm the exact `v`-argument precedence in 3HAC050988 §ArcL
  when implementing.
- **Do not hand-edit** seam/welddata via the RAPID data-type editor while the
  system is configured (Fronius AM warning) — program assignments and the
  FlexPendant tuning interface are the supported routes.

## 3. Brand integrations (both welders the HMI knows)

### 3.1 THIS CELL — Fronius TPS 500i /600V/nc (confirmed 2026-08-28)

**Model designation decoded** (Fronius operating instructions / product
naming): `TPS 500i` = 500 A-class TPS/i power source; `/600V` = the
575/600 V three-phase mains variant (North-American high-voltage); `/nc` =
**gas-cooled, no cooling unit** (as opposed to the water-cooled build).

What this settles:

- **The ABB integration path applies verbatim.** ABB's application manual is
  titled *Fronius TPS 320i/400i/500i/600i with RI-FB inside/i interface* —
  the 500i is explicitly in scope (3HAC065012 for RW6, 3HAC089028 for RW7).
- **The HMI is already configured for it**: `config.json`
  `"WELDER_MODEL_MILLER_OR_FRONIUSTPSi": "FRONIUSTPSi"` → `GetWelderType()`
  → `WelderType::FRONIUSTPSi` → the wire carries welder type **`02`**, which
  is the `welder_type = 2` branch of `r_w_p.kl`. No HMI change needed for the
  ABB cell; `nTG_WelderType` already receives 2.
- `/nc` (gas-cooled) means **no coolant circuit** → no coolant-flow
  interlock/error signals in the robot interface, and torch duty cycle is the
  gas-cooled rating. Electrical-install detail, no RAPID impact.
- `/600V` is a mains-supply variant only — no protocol or RAPID impact.

**Controller prerequisites — hard requirements from 3HAC065012** (quote the
manual's Prerequisites section):

| Requirement | Note for us |
|---|---|
| RobotWare **6.05 or higher** | our VC runs 6.15 ✓ |
| **[633-4] RobotWare Arc** | must be on the real key ⚠ |
| **[637-1] Production Screen** | second option, easy to miss when quoting ⚠ |
| **one** industrial network: [709-1] DeviceNet M/S, [841-1] EtherNet/IP Scanner/Adapter, [888-2] PROFINET Controller/Device, or [969-1] PROFIBUS Controller | choice is a cell-architecture decision ⚠ |

**Procurement flag ⚠** — the Fronius-side robot interface is *robot-brand
specific*: the existing FANUC cell uses the "TPSi RI FB/i **FANUC**" variant.
The ABB cell needs the RI FB inside/i configured for the ABB side and the
chosen fieldbus above. Whether an existing card can be re-configured or a
different variant must be ordered is a Fronius question to settle at quoting
time — do not assume the FANUC cell's hardware transfers.

**Operating mode — the key design input.** The FANUC cell runs *Special
2-step* (§1): absolute wire-feed-speed command + arc-length correction +
travel speed. Fronius's robot-interface modes are Job mode, characteristics
(synergic) mode, internal mode and special 2-step — the RI FB/i controls
Special 2-step, Job Mode and Internal Mode. In **characteristics mode** the
interface signals are *welding characteristic*, *wire feed speed command
value* and *arc length correction* — the direct equivalent of what the HMI
already sends, so this (not Job mode) is the natural target for the ABB port.
Job mode would instead require the HMI's proc number to become a Fronius job
number, changing the meaning of a field we already transmit.

- *Job mode with correction* (TPS/i firmware ≥ 2.3.0) is the alternative:
  tunables Weld Speed, Wirefeed **±20 %**, Arc Length Correction **±10 steps
  in 0.1 increments**. Note those are *correction* ranges, whereas special
  2-step commands wire feed **absolutely** — a real semantic difference for
  the HMI's Wire Speed field (IPM absolute today).
- Fronius *dynamic/pulse correction* is the natural home for the HMI's Arc
  Control field, which FANUC never applied at all (§1, §6).
- ⚠ **HMI range gap worth flagging**: `WidgetTouchupOffsets.cpp` lets Arc
  Length and Arc Control run 0–9999, while Fronius corrections are ±10 steps
  / 0.1 increments. Whatever mode is chosen, the apply-PROC (or the HMI)
  should clamp — out-of-range corrections raise a Fronius "correction outside
  specified range" signal.
- ⚠ Exact RAPID component names for characteristic / wirefeed / corrections
  under this add-in: the 3HAC065012 PDF is CID-encoded and not machine-
  readable here — read them from the manual inside RobotStudio's
  documentation once the controller is configured.

### 3.2 Miller (welder_type 1) — not this cell, kept for completeness

- Integration: ABB *Application manual — Miller Ethernet I/P Interface and
  Weld Editor* (3HAC054885, RW6): Auto-Axcess (Insight/Continuum family)
  connects over EtherNet/IP to the IRC5 (LAN2), and the **Weld Editor**
  FlexPendant app edits/validates the data against the welder.
- welddata there exposes the flat trio: *"the welddata components
  `weld_speed`, `weld_wirefeed` and `weld_voltage` can be tuned using the
  welddata tuning function"*. In synergic programs the voltage field acts as
  trim → HMI Arc Length maps there; ⚠ Miller-side "Arc Control"/dig mapping
  depends on the selected weld program on the power source.

### Unit conversions (HMI sends IPM in 9-char fixed-width reals)

| From (HMI) | To | Factor |
|---|---|---|
| travel speed IPM | `weld_speed` mm/s | × 0.42333 |
| wire feed IPM | m/min (Fronius wirefeed convention) | × 0.0254 |
| wire feed IPM | mm/s (if the component wants RAPID speed) | × 0.42333 |
| arc length / arc control | trim steps | 1:1 (already unitless trims) |

⚠ Which target unit each component expects is configuration-dependent —
verify against the configured Arc Equipment before converting. For this
cell's TPS/i in characteristics mode the wire feed command is **m/min**
(× 0.0254 from IPM); in a correction-based mode the same HMI field would
instead have to become a *percentage* — one more reason the mode decision
(§3.1) precedes any code.

## 4. Proposed ABB design (for the future implementation task)

1. `TG_Comms.sys` gains `PERS welddata wdTG_Weld` + `PERS seamdata sdTG_Weld`
   (+ `weavedata` if used) and one PROC `TG_ApplyWeldParams` that maps the
   already-received `nTG_*` values into `wdTG_Weld` with the §3 unit
   conversions. All brand/config specifics live in this ONE PROC — the wire
   protocol, the request PROC, and the .tgs shape stay brand-agnostic
   (`nTG_WelderType` is already stored if branching is ever needed).
2. The .tgs weld section calls `TG_ApplyWeldParams` after `TG_ReqWeldParams`,
   then welds with `ArcLStart/ArcL/ArcLEnd ..., sdTG_Weld, wdTG_Weld, ...` —
   replacing today's comment block in `TD05Test.mod`.
3. UDWP=0 (predefined): weld with named welddata prepared on the controller —
   an indexed `PERS welddata wdTG_Proc{n}` library is the direct `AWE1WPxx`
   analog (or Fronius Job numbers in Job mode); travel speed still overrides
   `weld_speed` (FANUC always wrote `$CMD_WSPEED`, and the HMI always sends
   it).
4. VC testability: RobotWare Arc can be added to the virtual controller and
   the weld process blocked/simulated so `ArcL*` executes as motion without
   equipment ⚠ (standard practice; confirm the exact block-process setting in
   3HAC050988 when starting the task). Wire choreography needs no new tests —
   R_W_P is already covered.

## 5. What must come from the customer / cell (cannot be researched)

1. RobotWare version + **[633-4] Arc** + **[637-1] Production Screen** +
   **one industrial network option** on the real key — exact list and
   minimum RW 6.05 now known (§3.1); still needs confirming against the
   quoted controller.
2. ~~Welder brand & model~~ **ANSWERED 2026-08-28: Fronius TPS 500i
   /600V/nc** (§3.1); HMI already set to FRONIUSTPSi. Still needed: the
   **TPS/i firmware version** (≥ 2.3.0 only if a correction-based mode is
   chosen).
3. Chosen operating mode — FANUC ran **Special 2-step** (§1), so the
   like-for-like ABB target is **characteristics/synergic mode**. Confirm
   the ABB cell will run the same mode, and therefore whether the HMI proc
   number maps to a *welding characteristic/program number* (expected) or a
   *Fronius job number* (Job mode).
4. The configured Arc Equipment class → the **actual welddata component
   set** (flat trio vs `main_arc` substructure, and their units).
5. The predefined procedure library: mapping of HMI proc numbers 1–99 to
   characteristics/jobs on the power source.
6. Which fieldbus the cell will use (DeviceNet / EtherNet-IP / PROFINET /
   PROFIBUS) and whether the Fronius-side RI FB inside/i card for ABB is
   included in the quote (§3.1 procurement flag).

## 6. Facts that close FANUC-parity questions

- FANUC never applied Arc Control to the schedule (commented out) — so the
  ABB port applying it (e.g. to Fronius dynamic correction) would be an
  *improvement*, not parity; equally defensible to store-only, like today.
  Decide at implementation time.
- FANUC's per-welder branches being identical means there is no hidden
  brand-specific wire behavior to port — brand handling is entirely
  robot-side. Confirmed twice now (KAREL read + HMI sender).

## 7. Sources

- In-repo/verified code: `Resources/FANUC/KAREL/r_w_p.kl`;
  `TGuideWeldingHMI` `RobotCell.cpp` (~2505), `FANUCRobot.cpp` (~509),
  `WidgetTouchupOffsets.cpp` (GUI units); sample `TD05tRJYQd.ls`.
- [ArcTool $CMD_VOLTS/$CMD_WFS mislabeling (robot-forum)](https://www.robot-forum.com/robotforum/thread/12610-system-variable-for-welder-control/)
- [AWE1WPxx.VR weld-schedule files (robot-forum)](https://www.robot-forum.com/robotforum/thread/17427-fanuc-weld-schedules-saved-where/)
- [Runtime welddata modification + ArcRefresh (ABB community)](https://tech-community.robotics.abb.com/discussion/11721/i-would-like-to-change-wire-feed-rate-and-speed-dynamically-in-program-while-welding-is-in-progress)
- [welddata vs seamdata roles (ABB community)](https://tech-community.robotics.abb.com/t/welddata-and-seamdata/6251)
- [Arc and Arc Sensor manual 3HAC050988 (RW6)](https://www.scribd.com/document/437855699/3HAC050988-en) /
  [ArcWare for OmniCore 3HAC084370](https://www.uzivatelskadokumentace.cz/Software%20Products/Arc%20welding%20software/en/3HAC084370-001.pdf)
- [Fronius TPS x00i with RI-FB inside/i, 3HAC065012 (RW6)](https://library.e.abb.com/public/3800d45179bb4ec3a7b5762253d79393/3HAC065012%20AM%20Fronius%20TPS%20x00i%20with%20RI-FB%20inside-i%20interface-en.pdf) /
  [3HAC089028 (RW7)](https://library.e.abb.com/public/b8bda7baf3524f4980ac8e2623f4a056/3HAC089028%20AM%20Fronius%20TPS%20x00i%20with%20RI-FB%20inside-i%20interface%20RW%207-en.pdf)
- [Miller Ethernet I/P Interface and Weld Editor, 3HAC054885 (RW6)](https://library.e.abb.com/public/41c9875fe03c4348a08fef2f9b769d4d/3HAC054885%20AM%20Miller%20interface%20RW%206-en.pdf)
- [633-4 Arc option (ABB technote AW6.03)](https://library.e.abb.com/public/e63b1a07d0e34a5cb4ae770e6f4b988a/technote_160428_AW6.03.pdf)
