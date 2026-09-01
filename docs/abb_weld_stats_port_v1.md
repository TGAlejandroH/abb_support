# R_W_S (id 13, Request Welding Stats) → RAPID Port Plan (v1) — Phase 6

Plan only — no code yet. Scope: port the R_W_S request with **dummy values**
sent from RAPID; reading the real numbers on an IRC5 is a separate
investigation task (§6).

## 1. Facts from the sources (verified 2026-08-31)

### 1.1 The KAREL program (now in the repo)

[R_W_S.kl](../resources/FANUC/KAREL/R_W_S.kl) ("Request Welding Stats") was
added 2026-08-31 — the §6 gap note in
[abb_weld_motion_and_data_design_v1.md](abb_weld_motion_and_data_design_v1.md)
("its KAREL source is not in resources/FANUC/KAREL/") is now stale and must be
amended. The program:

1. sends the request id `'13'`, reads the 1-byte ack;
2. reads three FANUC ArcTool system variables:
   - `$AWEWELDSTAT[1].$WELD_DIST` (REAL, **mm** — proven by the HMI's ÷25.4),
   - `$AWEWELDSTAT[1].$ARC_ON_TIME` (INTEGER; HMI treats it as **seconds**),
   - `$AWEWELDSTAT[1].$SUCC_AE` (successful arc ends; HMI treats it as a
     0/1 flag);
3. formats each with `CNV_REAL_STR(v, 8, 3, out)` (width-8, 3 decimals),
   joins with `','`, sends the CSV, reads the 1-byte ack.

No pose, no sub-routine token — the simplest request in the family: **two
robot→HMI messages, both `tgSendAck`-shaped.**

### 1.2 Where the exported program calls it

[TD05tRJYQd.ls](../resources/FANUC/.TGS%20HMI-MODE%20LS%20PROGRAM%20SAMPLE/TD05tRJYQd.ls)
lines 223 and 392: `CALL R_W_S` sits **inside the weld branch
(`R[198]=1`), immediately after `WELD END`, before the depart move** — once
per weld, welds 2 and 3 alike. Both marked slots already exist in the RAPID
samples: [TD05Weld.mod:107](../abb/rapid/TGS/TD05Weld.mod#L107) and
[TD05Test.mod:142](../abb/rapid/TGS/TD05Test.mod#L142).

### 1.3 What the real HMI does with it (the wire contract)

`RobotCell.cpp:1806` (`case CellStatus::RequestWeldingStats`) →
`FANUCRobot::ReceiveWeldingStats()` (`FANUCRobot.cpp:18`):

- one `do_receive()` (recv payload, answer 1-byte ack `"0"`);
- parses the CSV with `std::stringstream >> double >> char >> double >> char
  >> double` — **whitespace/sign tolerant**, so our `tgFmtReal` fixed-width
  format (`+0200.000`) parses cleanly;
- returns `(length_mm/25.4, arc_on_sec, succ_ae == 1.0)`;
- **only if `succ_ae` is true**: `analyticsManager->InsertWeldEntry(length_in,
  arc_on_sec)` — the weld-analytics DB entry. `succ_ae=0` (dry run / arc
  failure) is received and discarded.

Because we keep the CSV byte-compatible, the eventual C++ `ABBRobot` class
reuses `ReceiveWeldingStats()` unchanged.

## 2. RAPID design (TG_Comms.sys)

### 2.1 New request PROC

```
PROC TG_ReqWeldStats()
    ! FANUC: R_W_S (HMI request id 13) - welding stats after WELD END.
    ! Sequence: id -> "dist,arc_on_time,succ_ae" (one message, tgFmtReal
    ! fields). No pose, no sub token. Values are DUMMY for now (plan
    ! abb_weld_stats_port_v1.md section 6 tracks the real sensing).
    tgSendAck "13";
    tgSendAck tgFmtReal(nTG_WeldDist)+","+tgFmtReal(nTG_ArcOnTime)
              +","+tgFmtReal(nTG_SuccArcEnd);
    TPWrite "TG: weld stats sent, dist = "\Num:=nTG_WeldDist;
ENDPROC
```

(Sketch to fix the shape — final code written in the implementation step.)
One `SocketSend` per message = one KAREL `WRITE` (parity). Wire errors are
already covered: `tgSendAck` recovers via `tgCycleAbort`/`ExitCycle` (matrix
F-F), nothing new needed.

### 2.2 New shared PERS (register map, §4.3 of the port plan)

```
! R_W_S stats (KAREL read $AWEWELDSTAT[1] directly - no FANUC register
! equivalent). DUMMY VALUES until the sensing task lands: the .tgs program
! (or later TG_Weld.sys) writes them before calling TG_ReqWeldStats.
PERS num nTG_WeldDist:=0;      ! $WELD_DIST, mm
PERS num nTG_ArcOnTime:=0;     ! $ARC_ON_TIME, s
PERS num nTG_SuccArcEnd:=0;    ! $SUCC_AE, 1 = weld really ran
```

The .tgs program sets them before the call — mimicking the sensing and
making per-weld values distinguishable in transcripts.

### 2.3 Call sites in the sample .tgs programs

- **TD05Weld.mod** (both marked slots, after each `ArcLEnd`, before the
  depart `MoveL` — FANUC line-order parity): set semi-plausible per-weld
  dummies first, e.g. weld 2 `200 / 22.5 / 1` (its seam *is* 200 mm),
  weld 3 `200 / 15.75 / 1`. Distinct values ⇒ the transcript proves two
  independent servings.
- **TD05Test.mod** (line-142 slot, inside the weld branch): dummies
  `123.456 / 7.89 / 1` — recognizable, assertable, and it makes the request
  exercisable on the **non-Arc VC** (TD05Test is the phase 1–3 comms
  regression program; keeps R_W_S testable without option 633-4).

Dry-run note: FANUC calls R_W_S unconditionally inside the weld branch; with
welding inhibited `$SUCC_AE` stays 0 and the HMI skips the analytics insert.
Optional fidelity touch (decide at implementation): the sample programs set
`nTG_SuccArcEnd:=1-nTG_DryRun` instead of a constant 1.

## 3. Python prototype (hmi_prototype/abb_server.py)

- `handlers["13"] = self.handle_weld_stats_req` (id free, verified).
- Handler mirrors `ReceiveWeldingStats`:

```python
def handle_weld_stats_req(self):
    """FANUC R_W_S (id 13): one CSV message in - dist_mm, arc_on_s, succ_ae."""
    parts = self.do_receive().split(",")
    dist_mm, arc_on_s, succ_ae = (float(p) for p in parts)
    self.last_weld_stats = (dist_mm, arc_on_s, succ_ae)
    if succ_ae == 1.0:                      # HMI inserts analytics only then
        self.weld_stats_entries.append((dist_mm / 25.4, arc_on_s))
    self._log(f"  weld stats: dist={dist_mm:.3f} mm "
              f"({dist_mm / 25.4:.3f} in), arc_on={arc_on_s:.3f} s, "
              f"succ_ae={succ_ae:g}")
```

- `__init__`: `self.last_weld_stats = None`, `self.weld_stats_entries = []`
  (reset per `serve_cycle`, like `request_log`). `float()` accepts the
  `+0200.000` fixed width — same tolerance as the C++ parse.

## 4. Tests (extend the fake-robot executable specs)

New `hmi_prototype/test_phase6_weldstats.py` plus **updates to every test
whose choreography changes** (adding the call to TD05Test/TD05Weld moves their
expected request logs — phase 2/3 fake-robot specs and `test_phase4_weld.py`
must gain the id-13 exchange or they go red):

1. **Unit, handler level**: fake robot sends `"13"`, reads ack, sends
   `"+0123.456,+0007.890,+0001.000"`, reads ack → assert
   `last_weld_stats == (123.456, 7.89, 1.0)` and one analytics entry
   `(4.860…, 7.89)` (mm→inch ÷25.4, the HMI's conversion).
2. **succ_ae gate**: same with `…,+0000.000` → stats stored, **no** analytics
   entry (mirrors `RobotCell.cpp:1810`).
3. **Full cycle (TD05Test spec)**: id 13 appears exactly once, after 14 and
   before 100; values match the module's dummies.
4. **Weld-demo cycle (TD05Weld spec)**: two id-13 servings with the two
   distinct per-weld value sets, in weld order.
5. **Format parity**: `fmt_real`-style round-trip — the RAPID `tgFmtReal`
   output shape parses to the exact value (guards the C++ stringstream
   contract).

Expected count: ~5 new tests, 63 → ~68 total green, plus the amended specs.

## 5. RobotStudio VC validation (I run nothing — user runs, pastes transcripts)

**Check A — non-Arc VC, TD05Test (comms regression).**
`python abb_server.py 127.0.0.1 2000 2 <VC-HOME>` (2 cycles, program 1).
Expected new transcript lines per cycle, between request 14 and 100:

```
serving request 13
  robot -> '+0123.456,+0007.890,+0001.000'
  weld stats: dist=123.456 mm (4.860 in), arc_on=7.890 s, succ_ae=1
```

FlexPendant: `TG: weld stats sent, dist = 123.456`.
**Pass criterion**: the CSV is byte-identical to the expected string on
BOTH cycles, and `weld_stats_entries` (printed at cycle end) shows exactly
one entry per cycle with dist 4.860 in ±0.001.

**Check B — Arc VC, weld-demo.**
`python abb_server.py 127.0.0.1 2000 2 <VC-HOME> weld-demo`.
**Pass criterion**: two id-13 servings per cycle, weld 2 then weld 3, values
`200/22.5/1` then `200/15.75/1` exactly; existing section-15 weld criteria
(8.89/220.133/clamp; 12.7 + zeros) still met — proves the insertion did not
disturb the phase-4 choreography.

Setup steps land as a new section in
[robotstudio_setup.md](robotstudio_setup.md) (after §16).

## 6. Follow-up task (separate): real stats on the ABB controller

Out of this phase; tracked here so the dummies have a named successor.
Candidate sources, in recommended order of investigation:

1. **RAPID-computed (leading candidate — VC-testable, no new options):**
   - `weld_dist`: `Distance()` between the ArcLStart/ArcLEnd targets'
     `.trans` in the weld wobj (exact for straight seams; accumulate per
     segment for multi-segment seams). Natural home: TG_Weld.sys wrappers.
   - `arc_on_time`: `ClkStart` at ArcLStart / `ClkRead` after ArcLEnd.
     ⚠ Includes ignition/crater-fill, while FANUC `$ARC_ON_TIME` counts arc
     actually on — quantify the difference on the real cell.
   - `succ_ae`: 1 when ArcLEnd returns normally, 0 via the `AW_*` Arc error
     path (hooks into the F-series error matrix). ⚠ Verify what the VC's
     simulated/blocked weld does to Arc errors before trusting VC results.
2. **RobotWare Arc built-ins**: grep the RW 6.15 `options\arc\RS\` descriptor
   XMLs for stats-shaped symbols (same offline technique as the welddata
   research); check Production Screen (637-1) statistics facilities.
3. **Power-source truth (real cell only)**: Fronius TPS/i process image via
   RI FB inside/i (arc time, wire consumed, job stats) over the fieldbus.

Unit checks to close along the way: `$ARC_ON_TIME` unit on FANUC (KAREL
declares INTEGER; HMI assumes seconds), and `$SUCC_AE` semantics — the HMI
tests `== 1.0`, so a multi-arc-end weld would fail the gate **on FANUC too**;
match whatever FANUC actually reports, don't improve it.

## 7. Docs & comments to update in the same change

- [abb_port_plan_v1.md](abb_port_plan_v1.md): §5 add the Phase 6 entry;
  §6 assumption 5 ("R_W_S out of v1") amended; §4.2 naming map + §4.3
  register map gain the new symbols.
- [abb_weld_motion_and_data_design_v1.md](abb_weld_motion_and_data_design_v1.md)
  §6: KAREL source now in the repo; gap closed (dummy values), pointer here.
- [README.md](../README.md) line 59: remove R_W_S from the not-ported list
  (note the dummy-values status).
- [TD05Weld.mod](../abb/rapid/TGS/TD05Weld.mod) lines 29/107 and
  [TD05Test.mod](../abb/rapid/TGS/TD05Test.mod) line 142: "not ported"
  comments become the actual calls.
- [robotstudio_setup.md](robotstudio_setup.md): new validation section (§5
  above). [fanuc_hmi_request_program_calls_v1.md](fanuc_hmi_request_program_calls_v1.md)
  already lists id 13 — no change.

## 8. Delivery steps (each green before the next)

1. RAPID: PERS + `TG_ReqWeldStats` in TG_Comms.sys; calls + dummy values in
   TD05Test.mod and TD05Weld.mod.
2. Python: handler + state in abb_server.py.
3. Tests: new phase-6 file + amend the phase 2/3/4 fake-robot specs; full
   suite green.
4. Docs of §7.
5. Hand over the §5 RobotStudio instructions; user validates on the VC and
   pastes transcripts; numeric analysis against the pass criteria.
