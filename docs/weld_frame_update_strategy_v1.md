# Weld Frame Update Strategy — `uframe` or `oframe`? (v1)

Summary: **the request PROCs write `oframe`, always** — one write target, no case detection in RAPID.
**What the HMI sends is decided by one thing: whether the weld is coordinated.** A weld the
positioner does not move during — a static table, or a part held at an index on a 1- or 2-axis
positioner — gets an identity `uframe` and the CAD frame **w.r.t. the robot base**, exactly what the
HMI sends FANUC today. Only a genuine coordinated-motion weld gets `ufprog:=FALSE`/`ufmec:="STN1"`,
and there the served frame is the part **w.r.t. the positioner frame** — again mirroring what FANUC
does with its dynamic frame.

This resolves **O-1** of the Weld Planner's `abb_hmi_request_contract_v1.md`, which left the write
target open and assumed `TG_ReqWeldFrame` would have to detect which case it was in. It does not.

**Status:** implemented in this repo 2026-09-03 (`TG_Comms.sys`, `TGS/TD05Test.mod`,
`TGS/TD05Weld.mod`). **Not yet run on a controller** — the `oframe` write is numerically inert on
the current declarations (§4), so the existing VC validation still stands, but the
coordinated/`ufmec` shape has never been loaded.

⚠ **This document was revised the day it was written.** The first version branched on *mounting*
("is the part bolted to the chuck?") rather than on *coordination*, which put every station weld on
the `ufmec` shape. §6 records why that was dropped, including the calibration argument that turned
out not to hold — worth reading before anyone proposes it again.

---

## 1. The rule

| | Not coordinated — static table, **or indexed on a positioner** | Coordinated motion |
|---|---|---|
| `ufprog` | `TRUE` | `FALSE` |
| `ufmec` | `""` | `"STN1"` (the station) |
| `uframe` | **identity** | computed live by the controller from the station angles; the declared value is ignored |
| `oframe` | the CAD/part frame **w.r.t. the robot base** | the part **w.r.t. the positioner/coordinated frame** |
| What the HMI sends | base-referenced — **identical to FANUC today** | positioner-referenced — mirrors FANUC's dynamic coordinated frame |
| What a request writes | `oframe` | `oframe` |

**The branch is coordinated-vs-not, and nothing else.** Not brand, not whether the part sits on a
positioner, not whether the cell has one. A part indexed on a 2-axis positioner and a part clamped
to a fixed table are the same case here: the positioner is stationary during the weld, so a
base-referenced frame describes the part correctly for the whole weld.

This is the axis the rest of the system already branches on — the `coordinated_positioner_group`
marker in the Weld Planner, and FANUC's own static-UFRAME vs dynamic-UFRAME split. One rule, both
brands.

## 2. Why `oframe` is the write target in both cases

`TG_ReqWeldFrame` and `TG_ReqCamFrame` write `.oframe` unconditionally.

- In the coordinated case there is no choice: with `ufprog:=FALSE` the controller defines the user
  frame continuously from the positioner angles and **ignores the declared `uframe` outright**.
  Writing `uframe` there is a silent no-op — no error, no warning, wrong geometry, which is
  finding **F-2**'s failure mode again.
- In the non-coordinated case `uframe` is identity, so writing the served frame to `oframe` is the
  same geometry as writing it to `uframe` (§4). Both work; picking `oframe` makes it the same line
  of RAPID in both cases.

So the routine needs no branch, and the case knowledge stays where it is actually known — in the
exporter, which holds the weld's positioner group. RAPID is the layer hardest to change and hardest
to test. Keep it dumb.

## 3. Who states the mounting, and when

The exported program assigns the **whole `wobjdata` record** at entry, before any motion:

```rapid
! not coordinated (static table, or indexed on a positioner)
wobjTG_Weld := [FALSE, TRUE, "", [[0,0,0],[1,0,0,0]], <part w.r.t. robot base>];

! coordinated motion
wobjTG_Weld := [FALSE, FALSE, "STN1", [[0,0,0],[1,0,0,0]], <part w.r.t. positioner frame>];
```

Assigning the whole record, not a component, is what makes the statement complete — a program that
inherited the previous run's `ufprog`/`ufmec` would weld against the wrong thing. At entry, ahead of
motion, also keeps it clear of RAPID's look-ahead (contract **O-3**).

⚠ **Sequencing, and this is the one thing to get right.** A base-referenced frame describes the part
**at the index it was reported at**. So for an indexed weld the station must already be at the weld
index when `TG_ReqWeldFrame` runs — index first, then request the frame, then weld. This is the same
rule the FANUC path already lives under; confirm the exporter sequences it that way rather than
assuming, because the failure is silent and geometric.

## 4. Why the `oframe` switch is free today

`TG_Comms.sys` declares both work objects with an **identity `uframe` and an identity `oframe`**.
A `wobjdata` resolves as `world ← uframe ← oframe ← target`, so motion and `CRobT` alike see only
the product `uframe · oframe`: with an identity `uframe`, writing the served frame to `oframe`
instead of `uframe` is the same geometry. Nothing else in the repo reads `.uframe`.

⚠ **The composition is manual-backed, not yet measured.** 3HAC050917-001's `wobjdata` definition is
the source. `NON-ESSENTIAL-NICE-TO-HAVE/TGToolFrameSet.mod` step **4B** is written to measure it
(both a `uframe` and an `oframe` shift of −100 mm should move the TCP identically) and has **not
been run**. Two minutes on the same VC session as the coordinated load-check.

## 5. What the coordinated case costs the HMI

Only coordinated welds change what the HMI sends, and only in the last step: it serves the part's
pose in the **positioner frame** instead of the robot base. Everything upstream — socket protocol,
request sequence, pose codec, registration — is untouched, and the FANUC path is untouched entirely.

Two requirements come with it:

- **`wobjTG_Cam` and `wobjTG_Weld` must agree on the convention for a coordinated weld.** The
  capture registers against the pose reported in the camera work object, and a pose reported in the
  positioner frame is not comparable with one reported in base. For a non-coordinated weld both are
  base-referenced, which is what the HMI already assumes, so nothing to do there.
- **Home and transition moves stay on `wobj0`.** True in both cases, but under a coordinated work
  object it stops being cosmetic: a rest-home move bound to the part's work object would follow the
  turntable.

⚠ **And one consequence further downstream, from the HMI's own composition.**
[abb_port_plan_v1.md](abb_port_plan_v1.md) §1.4.1 establishes (read off `RobotCell.cpp`) that the
HMI consumes exactly one reported pose — `R_C`'s — and lands the cloud in base via
`bTpart · act_pose · cloud`, where `bTpart` is *the frame the HMI itself sent*. For a
non-coordinated weld that still lands in base, unchanged, because `bTpart` is base-referenced. For a
**coordinated** weld it lands in the **positioner frame**, since `bTpart` now is. Anything
downstream that assumes a base-frame cloud — collision checking, display, anything combining the
scan with a fixed cell feature — needs the extra `plate(θ)` for that case. Not a defect, but it is
the one place the coordinated convention reaches past the frame exchange itself.

*(Unchanged by any of this: the camera-calibration handlers expect base-frame poses, so those
requests keep `\WObj:=wobj0` — port plan §1.4.1 corollary (b). Cam-cal is out of v1 scope anyway.)*

## 6. Two things deliberately NOT done, and why

### 6.1 The `ufmec` work object is not used for indexed welds

The first draft of this rule branched on *mounting* — "the part rides the chuck, so the work object
rides the chuck" — putting indexed welds on the `ufprog:=FALSE`/`ufmec` shape too. Dropped, for
three reasons.

1. **Risk.** The `ufprog:=FALSE`/`ufmec` record has never been loaded on a controller. The static
   shape has (`LOCAL PERS wobjdata` accepted as a `\WObj` argument, RW6.15.08/IRB4600, Check Program
   clean). Routing *every* station weld — which is most welds — through the unverified path buys
   nothing and risks the common case.
2. **Brand parity.** Coordinated-vs-not is the axis FANUC already branches on and the axis the Weld
   Planner already marks. Mounting-vs-motion would have been a second, ABB-only axis for the same
   decision.
3. **It brought consequences that vanish under the coordinated-vs-not rule** — a wider `ActUnit`
   gate (a `ufmec` unit must be active before coordinated motion, so an indexed weld naming one
   would inherit that requirement), and the camera/weld convention agreement extending to every
   weld instead of just coordinated ones.

### 6.2 The accuracy argument for it did not hold

The stated reason for the mounting rule was that a plate-referenced `oframe` would make the
positioner calibration cancel on **scan at index A, weld at index B**. Written out, it does not.

Let `plate(θ) = B · R(θ)`, with `B` the robot-base→station-origin calibration and `R(θ)` the
rotation about the station axis. Transporting a correction from A to B costs:

```
base-referenced, PC transports:          B_pc   · R(Δθ) · inverse(B_pc)
positioner-referenced, controller does:  B_ctrl · R(Δθ) · inverse(B_ctrl)
```

Same formula. `B` **conjugates rather than cancels**, so an error in it scales with `Δθ` and with
the lever arm from the axis to the part — identically in both. The only real difference is *whose*
`B` is used, which is a single-source-of-truth argument, not an accuracy one; and on this cell the
PC's model is derived from the controller's `MOC.cfg` anyway. Where it does cancel exactly is
Δθ = 0 — scan and weld at the same index — and there the base-referenced scheme needs no transport
at all, so it is a wash there too.

⚠ **Neither convention fixes scan-at-A / weld-at-B.** Both run the correction through a model of the
positioner. If that path matters, the fix is the calibration itself, or re-scanning at B — not the
frame convention. Do not let this decision be read as having solved it.

## 7. Open

- **Controller check.** The `ufprog:=FALSE`/`ufmec` work object has not been loaded on a VC. Bundle
  it with the Weld Planner's outstanding coordinated `633-4` Arc VC check.
- **Step 4B** (§4) — the composition premise this rule rests on, written and unrun.
- **Sequencing confirmation** (§3) — that the exporter indexes the positioner before requesting the
  frame on an indexed weld.
- **Look-ahead (contract O-3) is unchanged** by this rule. The entry-time mounting assignment sits
  ahead of all motion, so only the per-weld `oframe` write is exposed, exactly as before.
