# R_W_S (id 13, Request Welding Stats) → RAPID Port (v1) — Phase 6

**Status: IMPLEMENTED AND VC-VALIDATED 2026-08-31**, 84 tests green; all
three VC checks green over 2 cycles each
([robotstudio_setup.md](robotstudio_setup.md) §17). Scope: port the R_W_S
request with **dummy values** sent from RAPID; reading the real numbers on an
IRC5 is a separate task (§6) that changes only where the three numbers come
from, never the wire.

**What shipped**
| Piece | Where |
|---|---|
| `TG_ReqWeldStats` PROC | [TG_Comms.sys](../abb/rapid/TG_Comms.sys) |
| `nTG_WeldDist` / `nTG_ArcOnTime` / `nTG_SuccArcEnd` PERS | [TG_Comms.sys](../abb/rapid/TG_Comms.sys) |
| Call + dummy values, non-Arc program | [TD05Test.mod](../abb/rapid/TGS/TD05Test.mod) |
| Call + dummy values, both Arc welds | [TD05Weld.mod](../abb/rapid/TGS/TD05Weld.mod) |
| `handle_weld_stats_req` + `dry-run` mode | [abb_server.py](../hmi_prototype/abb_server.py) |
| 17 new tests (+2 amended specs) | [test_phase6_weldstats.py](../hmi_prototype/test_phase6_weldstats.py) |

## 1. Facts from the sources (verified 2026-08-31)

### 1.1 The KAREL program (now in the repo)

[R_W_S.kl](../resources/FANUC/KAREL/R_W_S.kl) ("Request Welding Stats") was
added 2026-08-31, which is what unblocked this port — the §6 gap note in
[abb_weld_motion_and_data_design_v1.md](abb_weld_motion_and_data_design_v1.md)
("its KAREL source is not in resources/FANUC/KAREL/") was written before that
and has been struck through. The program:

1. sends the request id `'13'`, reads the 1-byte ack;
2. reads three FANUC ArcTool system variables (semantics researched
   2026-08-31, confidence marked per field — see §1.4):
   - `$AWEWELDSTAT[1].$WELD_DIST` — **CONFIRMED** REAL, **mm**, **per-weld**,
   - `$AWEWELDSTAT[1].$ARC_ON_TIME` — declared INTEGER in the KAREL; seconds
     is **inferred**, not confirmed,
   - `$AWEWELDSTAT[1].$SUCC_AE` — **UNCONFIRMED by any source**; the HMI
     treats it as a 0/1 flag;
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

### 1.4 What `$AWEWELDSTAT` actually is (researched 2026-08-31)

Needed so the follow-up (§6) reproduces the right quantity rather than a
plausible-looking one.

**CONFIRMED — `$AWEWELDSTAT` is per-weld, `$AWEPRODSTAT` is cumulative.**
FANUC's *System Variable Listing* (R-J3iB), on `$AWEPRODSTAT[1].$weld_dist`:
"updated while welding to reflect the **total distance welded**… **The units
are millimeters**… Note there is a … field in `$aweweldstat[1]` that reflects
only the distance for the **current weld. It is reset at the start of each
weld**." The same source states the pattern outright for heat:
`$AWEWELDSTAT[1]` holds "the heat input for **each weld**",
`$AWEPRODSTAT[1].$WELD_HEAT` "for **all welds**". So the pair is a deliberate
per-weld / lifetime split, and the KAREL reads the per-weld side.
Also confirmed: the `[1]` index is the **weld equipment number**, not a weld
number (B-83284EN-3/03 uses `[i] : weld equipment number` throughout).

**CONFIRMED — mm.** Two independent official sources: the listing above, and
the FANUC *Arc Welding Function* operator's manual B-83284EN-3/03 §19.4.6
("The unit is mm") and its log-field list (`WeldDist --- Weld Distance (mm)`,
`WeldTime --- Weld Time (sec)`). This independently confirms what the HMI's
÷25.4 already implied.

**INFERRED — `$ARC_ON_TIME` in seconds.** The variable itself is in no public
listing. Every confirmed time in ArcTool's log/status subsystem is seconds
(`WeldTime … (sec)`; §19.4.6 "The unit is sec"), and ArcTool is confirmed to
track per-weld *and* total arc-on time as separate quantities (§18: "Arc on
time per a welding"; total shown in parentheses). Seconds is very likely but
should be treated as inferred. The `H:M:S` on the Weld Status screen is
display formatting of the **cumulative** figure, not the storage unit.

**⚠ UNCONFIRMED — `$SUCC_AE`.** Not documented in the system-variable
listings, in B-83284EN-3/03 (full-text searched: zero hits), or in any public
code. This is the one field the HMI's `== 1.0` test depends on, so it is worth
stating plainly rather than assuming:
- It is **unlikely to be a controller-lifetime counter** — every confirmed
  cumulative counter in this family lives in `$AWEPRODSTAT` with a
  turnover-at-1,000,000 and a Weld Status RESET key. So the "breaks on weld
  #2" worry raised in the original plan is probably unfounded.
- The **live risk is different**: it may be a *per-weld count* of successful
  arc ends rather than a boolean. ABB's directly analogous per-seam field is
  exactly that — `ArcStarts`, "Number of arc starts for the seam - **ideally
  1**" (§6). If FANUC's is a count too, then `== 1.0` silently reports failure
  for any weld that legitimately **re-ignited after an arc retry**, dropping a
  real weld from the analytics. That is a latent defect in the FANUC path as
  well, not something our port introduces — flagged here, not "fixed", since
  matching FANUC behaviour is the port's contract.
- **Settles in ~10 minutes on real FANUC hardware**: run three consecutive
  welds reading `$AWEWELDSTAT[1].$SUCC_AE` after each, then force one arc
  loss/retry. `1,1,1` then `1` → boolean. `1,1,1` then `2` → per-weld count.
  `1,2,3` → cumulative, and the HMI is broken today.

## 2. RAPID (as built, `TG_Comms.sys`)

`TG_ReqWeldStats` builds the payload into a local string, then sends two
messages: the id and the csv. One `SocketSend` per message = one KAREL
`WRITE` (parity). Wire errors needed nothing new — `tgSendAck` already
recovers via `tgCycleAbort`/`ExitCycle` (error matrix F-F).

Three new PERS carry the values (`nTG_WeldDist` mm, `nTG_ArcOnTime` s,
`nTG_SuccArcEnd` flag). They have **no FANUC register equivalent**: KAREL read
`$AWEWELDSTAT[1]` straight out of the controller, so this is the one request
whose inputs are not part of the R[] register map in plan §4.3.

**Number format — deliberate deviation, verified safe.** RAPID sends the
`tgFmtReal` form `+0123.456` (signed, zero-padded, 9 chars), the same field
every other scalar on this wire already uses. FANUC's producer is different:
`CNV_REAL_STR(v,8,3)` pads with **blanks** to a *minimum* width, always with
at least one leading blank and no `+` — so the real FANUC payload looks like
`" 123.456,   7.890,   1.000"`. Both decode identically because the sole
consumer, `std::stringstream >> double`, skips leading whitespace and accepts
a leading `+` (`float()` does the same). Keeping ONE real format across the
ABB port beat byte parity on a field nobody compares bytewise — and the
tolerance is pinned by a test that feeds the handler the FANUC form.

**Call sites.** Once per weld, immediately after the weld instruction and
before the depart move, matching FANUC lines 223/392:
- `TD05Weld.mod`, both welds: `200 / 22.5 / …` then `200 / 15.75 / …`. Not
  arbitrary — both seams really are 200 mm, and the times are that length
  divided by the weld speed each weld is actually served (21 IPM = 8.89 mm/s,
  30 IPM = 12.7 mm/s), so a transcript is self-consistent. Distinct per weld,
  so a repeated payload cannot masquerade as two servings.
- `TD05Test.mod`: `123.456 / 7.89 / …` — recognizable and assertable, and it
  keeps R_W_S exercisable on a VC **without** the Arc option (633-4).

**`succ_ae` is derived, not constant**: `nTG_SuccArcEnd := 1-nTG_DryRun`.
FANUC calls R_W_S unconditionally inside the weld branch, and with welding
inhibited `$SUCC_AE` stays 0, so the HMI skips the analytics insert. Deriving
it reproduces that for free and makes the dry-run path testable.

## 3. Python (as built, `hmi_prototype/abb_server.py`)

`handle_weld_stats_req` (registered as `handlers["13"]`) mirrors
`ReceiveWeldingStats` **plus** the `RobotCell.cpp` gate around it: parse three
reals, record `(length_in, arc_on_sec)` in `weld_stats_entries` **only when
`succ_ae == 1`**, and log which of the two happened. The mm→inch conversion
lives here, on the HMI side, because `AnalyticsManager::InsertWeldEntry` takes
inches. A payload without exactly three fields raises rather than recording a
bogus weld. `weld_stats_entries` resets per `serve_cycle`, mirroring the real
HMI's one-weld-session-per-run, and `main()` prints the tally at the end of
each cycle so a VC run shows it without a debugger.

`main()` also gained a **`dry-run` mode** next to `corrupt-cam` /
`corrupt-weld` / `weld-demo`, so the succ_ae gate is one command to check.

## 4. Tests (as built)

17 new tests in `hmi_prototype/test_phase6_weldstats.py`; the phase-2 and
phase-4 fake-robot specs gained the id-13 exchange (phase 3 reuses phase 2's,
so it followed for free) and their expected request logs now read
`… 14, 13, 100`. **84 total green** (was 65).

What they pin, beyond the happy path:
- **Both producers' number formats** parse — the RAPID `+0123.456` form and
  FANUC's blank-padded `" 123.456"` — which is what keeps one HMI
  implementation valid for both robot brands.
- **The analytics gate**: `succ_ae = 0` is received and *not* recorded;
  a dry-run cycle serves the request and records nothing.
- **The mm→inch conversion** happens HMI-side (254 mm → 10.0 in).
- **A malformed payload raises** instead of recording a bogus weld.
- **Both messages are acked** (two robot→HMI messages, two `"0"` acks).
- **Skipped and aborted welds serve no stats at all** — the call lives inside
  the weld branch.
- **Two independent servings** in the Arc program, with the per-weld values,
  and those arc-on times agree with the weld speeds actually served.
- **The verbose transcript line** renders — every other test runs quiet, so
  the line the VC checks are read from would otherwise be untested.
- **No leakage between cycles** in the entry list.

## 5. RobotStudio VC validation

Full instructions, expected transcripts and pass criteria:
**[robotstudio_setup.md](robotstudio_setup.md) §17** — check A (non-Arc,
`TD05Test`), check B (`dry-run` mode, the analytics gate), check C (Arc,
`weld-demo`, two servings + the §15 weld criteria as a regression).

The expected transcript lines there were generated by running `main()`
against the phase-2 fake robot, not written by hand, so they are byte-exact.

**Result: all three checks PASSED 2026-08-31**, 2 cycles each. What the run
actually proved, beyond "it works":
- The payload is byte-identical to the Python-side prediction on every cycle,
  so `tgFmtReal` and `fmt_real` agree in practice, not just by inspection —
  the property the C++ parse will depend on.
- **Check B is the load-bearing one.** Between check A and check B only the
  third field changed (`+0001.000` → `+0000.000`), with distance, time, request
  order and everything else identical. That is direct evidence that
  `nTG_SuccArcEnd := 1-nTG_DryRun` really evaluates on the controller, and that
  the analytics gate is driven by the robot's report rather than by anything
  HMI-side.
- The two Arc servings carried different payloads in weld order, and their
  arc-on times matched the weld speeds logged in the very same transcript
  (200/8.89 = 22.50 s, 200/12.7 = 15.75 s) — the self-consistency the dummy
  values were chosen for.
- The runs used the **RWS** delivery path (`http://localhost:80`) instead of a
  VC HOME folder. R_W_S is downstream of the transfer so this does not affect
  it, and it incidentally re-validated the phase-5 RWS leg.
- Not covered by the VC run (covered by tests only): the aborted- and
  skipped-weld paths, where no request 13 is served at all.

## 6. Follow-up task (separate): real stats on the ABB controller

Out of this phase; tracked here so the dummies have a named successor.
ABB-side research done 2026-08-31 — findings and confidence below.

### 6.1 ABB already computes exactly these three numbers

**CONFIRMED** (*Product specification - Controller software IRC5*,
3HAC050945-001 §14.2.2.2): the option **[659-1] Production Monitoring**
maintains a `SeamResults` table with "a record for **each weld seam that is
finished**", whose columns map one-to-one onto our payload:

| `SeamResults` column | ABB's description | our field |
|---|---|---|
| `SeamLen` | "Length of actual weld completed for the seam" | `nTG_WeldDist` |
| `Duration` | "Time in seconds to complete seam" | `nTG_ArcOnTime` |
| `Completed` | "True if all welds finished to completion" | `nTG_SuccArcEnd` |

Two things this settles. First, the quantities are **well-defined on ABB** —
"length of actual weld completed" is the same notion as FANUC's per-weld
`$WELD_DIST`, so the port is reproducing a real equivalent, not inventing one.
Second, and most useful: ABB states these "result tables contain data that is
**calculated on the fly within RAPID**" — RobotWare Arc's own RAPID code
already derives seam length and duration, which is strong evidence a
RAPID-level computation is the right shape of answer.

⚠ **Do not simply buy the option.** Its documented delivery path requires
"WebWare Server 4.5 or higher" on the PC — WebWare is ABB's **discontinued**
PC product. The option is still listed for RW6 and the 6.08+ Fronius TPS/i
incompatibility is fixed (so 6.15 is fine), but the transport is obsolete for
a new PC integration. Treat `SeamResults` as the **specification to match**,
and only investigate buying 659-1 if a supported modern read path turns up.

### 6.2 Recommended implementation: compute in RAPID, read over RWS

Still the leading candidate, now with the §6.1 column semantics as the target.

- **`weld_dist`** — `Distance()` between the seam targets' `.trans` in the
  weld wobj (exact for straight seams; accumulate per segment otherwise).
  Natural home: `TG_Weld.sys` wrappers around the Arc instructions.
  ⚠ The community alternative (speed × arc-on time, from a background task
  watching Arc Established) **overestimates**: ignition and crater-fill are
  arc-on at low or zero travel. `Distance()` on the taught geometry avoids
  that error entirely, which is why it is preferred here.
- **`arc_on_time`** — `ClkStart`/`ClkRead` around the seam. **CONFIRMED**
  (3HAC050917-001): `ClkRead` returns **seconds**, resolution normally
  0.001 s, `ERR_OVERFLOW` only at ~49 days. ⚠ Still includes ignition and
  crater-fill, whereas FANUC's counter is arc-on; quantify the gap on the real
  cell and consider subtracting the start/end delays.
- **`succ_ae`** — the RAPID equivalent of "the arc ended successfully" is
  *`ArcLEnd` returned without entering its ERROR handler*. **CONFIRMED** ERRNO
  set (*Application manual - Arc and Arc Sensor*, 3HAC050988):
  `AW_START_ERR`, `AW_IGNI_ERR`, `AW_WELD_ERR`, `AW_EQIP_ERR`, `AW_WIRE_ERR`,
  `AW_STOP_ERR`. Relevant detail: **wire-stick supervision is checked at the
  end of the weld**, so a failed *end* specifically is observable. Pattern:
  set the flag TRUE before the seam, FALSE in the ERROR handler.
  ⚠ **A VC weld is simulated** — do not trust VC results for the failure
  branch; only the real cell proves it.

The PC side needs no new mechanism: there is no arc-statistics RWS domain, so
the pattern is to compute into PERS and read them via RWS symbol access, which
[rws_client.py](../hmi_prototype/rws_client.py) already does.

### 6.3 Corrections to earlier assumptions in this doc

- **[637-1] Production Screen carries no weld statistics.** An earlier draft
  of this section listed it as a place to look. It is a FlexPendant launcher
  shell — "Production Screen is only used to launch applications"
  (3HAC050945-001 §11.10) — and it is already **bundled inside [633-4] Arc**.
  The statistics option is **659-1**, above.
- **`ArcRefresh` is a write path**, for retuning voltage/wirefeed/speed
  mid-weld from a sensor trap. Unrelated to reading statistics; ruled out.
- Base RobotWare Arc exposes **no** predefined RAPID variable for seam
  distance or arc time (unconfirmed as an absolute, but nothing surfaced in
  the RAPID reference or the Arc manual). The only "statistics" language in
  the Arc manual is voltage/current means for **ARCITEC** systems — not our
  quantities and not our power source.

### 6.4 Power-source truth (real cell only)

Fronius TPS/i process image via RI FB inside/i (arc time, wire consumed, job
stats) over the fieldbus — unchanged from the earlier plan, and still the only
route to numbers measured by the welder rather than derived by the robot.

### 6.5 Open item to close on FANUC hardware

`$SUCC_AE` semantics (§1.4): boolean vs per-weld count. It decides whether the
HMI's `succ_ae == 1.0` gate is exactly right or drops welds that re-ignited.
The three-weld + forced-retry experiment in §1.4 settles it; match whatever
FANUC actually reports rather than improving on it.

## 7. Docs updated with this change (all done)

- [abb_port_plan_v1.md](abb_port_plan_v1.md): Phase 6 entry added to §5;
  §6 assumption 5 ("R_W_S out of v1") amended in place.
- [abb_weld_motion_and_data_design_v1.md](abb_weld_motion_and_data_design_v1.md)
  §6: the gap note claiming the KAREL source is "not in
  `resources/FANUC/KAREL/`" is struck through and closed — it was written
  before `R_W_S.kl` was added, and the remaining gap is now narrower (dummy
  values, not a missing request).
- [README.md](../README.md): R_W_S removed from the out-of-scope list, with
  the dummy-values caveat kept explicit.
- [robotstudio_setup.md](robotstudio_setup.md): new §17 (checks A/B/C).
- `TD05Weld.mod` / `TD05Test.mod`: the "not ported" / "out of v1" comments are
  now the actual calls.
- [fanuc_hmi_request_program_calls_v1.md](fanuc_hmi_request_program_calls_v1.md)
  already listed id 13 and R_W_S is not a frame-PR-writing routine, so its
  load-bearing marker table needed no change.

## 8. Delivery steps

1. ✅ RAPID: PERS + `TG_ReqWeldStats` in `TG_Comms.sys`; calls + dummy values
   in `TD05Test.mod` and `TD05Weld.mod`.
2. ✅ Python: handler, per-cycle state, `dry-run` mode, per-cycle tally.
3. ✅ Tests: new phase-6 file + amended phase-2/4 specs — 84 green.
4. ✅ Docs (§7).
5. ✅ VC checks run and analysed against the pass criteria — all green; this
   doc, the plan's Phase 6 entry and setup §17 carry the VC-VALIDATED stamp.

**Phase 6 is complete.** The only work left on R_W_S is replacing the dummy
values with real ones (§6), which is a separate task and does not touch the
wire, the request order, or the HMI.

## 9. Sources for the §1.4 / §6 research (2026-08-31)

FANUC:
- *FANUC Robotics R-J3iB System Variable Listing* — `$AWEPRODSTAT[1].$weld_dist`
  entry (mm; per-weld vs cumulative split; the `$AWEWELDSTAT` cross-reference).
- *Operator's Manual, R-30iB/R-30iB Mate, Arc Welding Function*, B-83284EN-3/03
  — §6.1 (Weld Status screen, RESET), §18 (arc-on time per weld vs total),
  §19.4.6 (weld distance in mm, weld time in sec), §19.5.2 (log field units),
  and the `[i] = weld equipment number` convention. Full-text searched:
  **no** occurrence of `SUCC_AE`, `ARC_ON_TIME` or `AWEWELDSTAT`.
- *KAREL Reference Manual* — `CNV_REAL_STR` padding semantics (minimum length,
  at least one leading blank, asterisk fill on overflow), which is what makes
  the §2 format deviation safe.

ABB:
- *Product specification - Controller software IRC5*, 3HAC050945-001 Rev AE —
  §14.2.2.2 (Production Monitoring [659-1], `SeamResults` / `CycleResults`
  columns, "calculated on the fly within RAPID", WebWare requirement),
  §11.10 / §14.2.2.4 (Production Screen [637-1] is a launcher), and the
  [633-4] Arc bundle contents.
- *Technical reference manual - RAPID Instructions, Functions and Data types*,
  3HAC050917-001 Rev F — `ClkRead` (seconds, 0.001 s resolution,
  `ERR_OVERFLOW` at 4,294,967 s).
- *Application manual - Arc and Arc Sensor*, 3HAC050988 — the `AW_*` ERRNO
  set, the start/continuous supervision input lists, and the end-of-weld
  wire-stick check.
- ABB Developer Center, Robot Web Services API reference — RAPID symbol
  read/write; no arc-statistics domain exists.
- forums.robotstudio.com "Calculating weld seam length" — the background-task
  speed×time idiom and its overestimation caveat (community, not manual).
