# Weld Frame Update Strategy — `uframe` or `oframe`? (v1)

Summary: **the vision correction writes `oframe`, always** — for an indexed weld exactly as for a
coordinated one — and `uframe` states *what the part is bolted to*. This resolves **O-1** of the
Weld Planner's [`abb_hmi_request_contract_v1.md`](https://github.com/TetraGenAdmin/curobo_suite/blob/develop/docs/abb_hmi_request_contract_v1.md),
which left the write target open and assumed `TG_ReqWeldFrame` would have to detect which case it
was in. It does not: there is one case.

**Status:** implemented in this repo 2026-09-03 (`TG_Comms.sys`, `TGS/TD05Test.mod`,
`TGS/TD05Weld.mod`). The Weld Planner side is recorded as **D33** in `abb_integration_plan_v1.md`.
**Not yet run on a controller** — the change is numerically inert on the current declarations
(see §4), so the existing VC validation still stands, but the coordinated/`ufmec` shape has never
been loaded.

---

## 1. The rule

| Component | Means | Who sets it | When |
|---|---|---|---|
| `ufprog` / `ufmec` / `uframe` | **the mount** — what the part is bolted to | the exported program | once, at entry, before any motion |
| `oframe` | **the part on the mount** | the exported program (nominal), then `TG_ReqWeldFrame` / `TG_ReqCamFrame` (measured) | at entry, then per weld |

- Part on the chuck / rotate positioner → `ufprog:=FALSE`, `ufmec:="<station>"`. The controller
  computes `uframe` live from the station angles; the declared `uframe` is ignored.
- Part on a static table → `ufprog:=TRUE` and an identity (or fixture) `uframe`.
- **A request PROC writes `oframe` and nothing else, in every case.**

## 2. Why indexed welds take the same shape as coordinated ones

The indexed/coordinated distinction is **whether the station moves during the weld**. It is not a
statement about what the part is bolted to — an indexed part rides the chuck just as a coordinated
one does. Mount and motion are different questions, and only the mount decides the work object.

Three consequences follow, and the third is the one that pays.

1. **`uframe` is not writable anyway once the part is on the chuck.** With `ufprog:=FALSE` the
   controller ignores the declared `uframe` outright. A routine that writes it is silently a no-op
   — no error, no warning, wrong geometry, which is failure mode **F-2** all over again.
2. **`TG_ReqWeldFrame` stops needing to know which case it is in.** O-1 assumed the routine would
   branch. Under one rule there is no branch to get wrong, and the case knowledge stays where it is
   actually known — in the exporter, which holds the weld's positioner group. RAPID is the layer
   that is hardest to change and hardest to test; keep it dumb.
3. **Scan-at-index-A / weld-at-index-B stops depending on the PC's positioner calibration.** This
   is the real win. See §3.

## 3. Why the calibration cancels

`CRobT(\Tool \WObj)` reports the TCP in the **object frame** — that is, through `uframe · oframe`.
`TG_ReqWeldFrame` reports the pose and then replaces `oframe`. So the loop is:

```
reported pose      p_rep    = inverse(uframe(θ) · oframe_prev) · TCP_world
served value       oframe_new = oframe_prev · Δ            (Δ measured in the frame p_rep is in)
```

`uframe(θ)` appears on both sides and cancels. The HMI never needs to know the station angle, the
plate pose, or the robot↔positioner calibration to compute what it serves — it needs `oframe_prev`,
which it either served itself or read from the exported program.

For a scan at θA and a weld at θB the residual is therefore only the **controller's own** relative
station accuracy between A and B — dominated by the axis-of-rotation direction, and far smaller
than the absolute robot↔positioner base calibration. Under the world-`uframe` scheme the PC has to
transport the correction from A to B through *its* model of the positioner, so both that model and
the base calibration enter at full magnitude.

⚠ **The cancellation requires the camera and weld work objects to share a mount.** The capture
registers against the pose reported in `wobjTG_Cam`; if that work object is static while
`wobjTG_Weld` rides the plate, the two reported poses are not comparable and the positioner model
comes straight back in. `wobjTG_Cam` and `wobjTG_Weld` take the same `ufprog`/`ufmec`, differing
only in `oframe`. Both `TG_ReqCamFrame` and `TG_ReqWeldFrame` write `oframe` for the same reason.

## 4. Why the switch is free today

`TG_Comms.sys` declares both work objects with an **identity `uframe` and an identity `oframe`**.
A `wobjdata` resolves as `world ← uframe ← oframe ← target`, so motion and `CRobT` alike see only
the product `uframe · oframe`: with an identity `uframe`, writing the served frame to `oframe`
instead of `uframe` is the same geometry. Nothing else in the repo reads `.uframe`. The change is
adopted now, ahead of the coordinated work, at no cost — and what it removes is a reverse
migration later, under a station, where the mistake is silent.

⚠ **The composition is manual-backed, not yet measured.** 3HAC050917-001's `wobjdata` definition is
the source. `NON-ESSENTIAL-NICE-TO-HAVE/TGToolFrameSet.mod` step **4B** is written to measure it
(both a `uframe` and an `oframe` shift of −100 mm should move the TCP identically) and has **not
been run**. Run it with the other pending VC checks; it is a two-minute confirmation of the
premise this whole rule rests on.

## 5. What is NOT free, and belongs to the Weld Planner / HMI

- **The wire meaning changes for a part on a station.** The served value is *the part's pose in
  mount coordinates* — not a world frame. Both sides must agree; a silent disagreement is a rigid
  offset that approach and retract motion will not reveal.
- **The HMI must decide per weld.** It has the project, so it knows the weld's positioner group and
  the mechanism's `positioner_ee_link`. RAPID does not and must not.
- **Home and transition moves must stay on `wobj0`.** A work object that rides the plate moves
  every point bound to it, the rest-home move included. The Weld Planner already hit this once
  (plan **E49**: 61 of 66 moves bound to the part's work object where 30 should have been); under a
  chuck-attached work object it stops being cosmetic.
- **Activation.** A `ufmec` mechanical unit must be active. On this cell `STN1`/`STN2` are
  `-activate_at_start_up FALSE`, and in HMI mode Production Manager owns activation.
  ⚠ Note this is **not** a new requirement introduced by the rule: RAPID's `extjoint` is an
  absolute per-point command, and an indexed weld's points already carry the station angle, so the
  station already has to be active for an indexed weld to run at all.

## 6. Open

- **Controller check.** The `ufprog:=FALSE`/`ufmec` work object has not been loaded on a VC in
  either repo. Bundle it with the Weld Planner's outstanding coordinated `633-4` Arc VC check.
- **Look-ahead (contract O-3) is unchanged** by this rule. The entry-time mounting assignment sits
  ahead of all motion, so only the per-weld `oframe` write is exposed, exactly as before.
