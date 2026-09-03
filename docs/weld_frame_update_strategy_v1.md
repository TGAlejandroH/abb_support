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
the current declarations (§4), so the existing VC validation still stands, but `wobjTG_WeldStn1` —
the coordinated shape — has never been loaded.

⚠ **This document was revised twice the day it was written.** The first version branched on
*mounting* ("is the part bolted to the chuck?") rather than on *coordination*, which put every
station weld on the `ufmec` shape — §6 records why that was dropped, including the calibration
argument that turned out not to hold. The second put both shapes in one `wobjdata` and swapped the
record at entry; §3 records why they are now separate symbols. Read both before re-proposing either.

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
| Work object | `wobjTG_Weld` | `wobjTG_WeldStn1` (per station) |

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

## 3. Two work objects, one per shape — and `ufprog`/`ufmec` are never written at runtime

The two shapes are **separate resident symbols**, not one record that changes kind:

```rapid
! TG_Comms.sys — declared once, per cell
PERS wobjdata wobjTG_Weld     := [FALSE, TRUE,  "",     <identity>, <oframe>];
PERS wobjdata wobjTG_WeldStn1 := [FALSE, FALSE, "STN1", <identity>, <oframe>];
```

An exported program **picks one by passing it**, and assigns only its `.oframe` nominal at entry:

```rapid
wobjTG_Weld.oframe := <part w.r.t. robot base>;
...
TG_ReqWeldFrame \Tool:=tTG_Weld \WObj:=wobjTG_Weld;       ! indexed / static
TG_ReqWeldFrame \Tool:=tTG_Weld \WObj:=wobjTG_WeldStn1;   ! coordinated
```

**`TG_ReqWeldFrame` needs no branch, and cannot have one.** It writes `.oframe` on *whatever it was
handed* — it never names a work object. That is the point of the `\PERS wobjdata WObj` parameter
style adopted as the fix for **F-2**: the frame is visible at the call site instead of hidden in
global state. The exporter knows which case each weld is, so it encodes that in the argument.

*(The deprecated modal fallback writes `wobjTG_Weld` **by name**, so it can only express the
non-coordinated case. That is fine — it is the static back-pocket path. Do not extend it.)*

### Why separate symbols rather than reassigning one record

The first version of this document had a single `wobjTG_Weld` whose whole record was assigned at
entry, taking either shape. Four problems, in order of severity:

1. **Runtime `ufmec` re-binding is unverified.** Assigning the string component is syntactically
   legal; whether the controller re-resolves the mechanical-unit binding mid-execution — rather than
   at load or first use — is not established, and normal ABB practice is a declared constant naming
   a configured unit. Declaring each shape once makes the question moot.
2. **A mixed program breaks it.** A part with some seams welded at an index and some with the
   turntable turning would have to reassign the record *between* welds — exactly the hazard **O-3**
   tracks, a `PERS` write landing after RAPID has already prepared queued instructions. The
   entry-time assignment dodged that only by being ahead of all motion.
3. **Robtargets bind by name.** Every target says `\WObj:=wobjTG_Weld`. If that record's *kind*
   changes mid-program, the same named target means different things at different points, and a
   pendant watch shows one mutating record.
4. **The static path stops being the verified one.** `wobjTG_Weld` keeping its declaration means the
   common case stays byte-identical to a shape a controller has actually accepted.

⚠ **Accepted cost:** `ufprog`/`ufmec` correctness now lives on the controller — the same trust model
as `tTG_Weld`, and the same **E47**-class hazard. A wrong `ufmec` is a silent wrong-station weld.
Treat these declarations as cell configuration and verify them at commissioning.

⚠ **Sequencing, and this is the one thing to get right in the exported program.** A base-referenced
frame describes the part **at the index it was reported at**. So for an indexed weld the station must
already be at the weld index when `TG_ReqWeldFrame` runs — index first, then request the frame, then
weld. This is the same rule the FANUC path already lives under; confirm the exporter sequences it
that way rather than assuming, because the failure is silent and geometric.

**Station 2 (D4) is deliberately not declared yet.** `wobjTG_WeldStn2` with `ufmec:="STN2"` is a
one-line addition when the two-station template lands; declaring it now would add a resident symbol
nothing references.

## 4. Why the `oframe` switch is free today

`TG_Comms.sys` declares both work objects with an **identity `uframe` and an identity `oframe`**.
A `wobjdata` resolves as `world ← uframe ← oframe ← target`, so motion and `CRobT` alike see only
the product `uframe · oframe`: with an identity `uframe`, writing the served frame to `oframe`
instead of `uframe` is the same geometry. Nothing else in the repo reads `.uframe`.

⚠ **The composition is manual-backed, not yet measured.** 3HAC050917-001's `wobjdata` definition is
the source. `NON-ESSENTIAL-NICE-TO-HAVE/TGToolFrameSet.mod` step **4B** is written to measure it
(both a `uframe` and an `oframe` shift of −100 mm should move the TCP identically) and has **not
been run**. Two minutes on the same VC session as the coordinated load-check.

## 5. What the coordinated case costs the HMI — and what was decided not to change

Only coordinated welds change what the HMI sends, and only in the last step: it serves the part's
pose in the **positioner frame** instead of the robot base. Everything upstream — socket protocol,
request sequence, pose codec, registration — is untouched, and the FANUC path is untouched entirely.

**Captures stay base-referenced, on both brands.** *(Owner call, 2026-09-03.)* Every capture is taken
at a standstill at one index, and reporting them all in the base-referenced `wobjTG_Cam` keeps the
HMI's scan and registration path one code path across FANUC and ABB. There is **no coordinated camera
work object**, and none should be added without a measurement to justify it.

⚠ **What that costs, stated so it is a known item and not an oversight.** For a coordinated weld the
HMI holds the part in base, `P_base`, and must produce a positioner-referenced `oframe`:

```
oframe = inverse(plate_pc(θs)) · P_base
```

`plate_pc` is the **PC's own** positioner model, and this is a single left-multiply — so a
calibration error there lands on the weld at **full magnitude**, with nothing downstream to absorb
it, because this *is* the vision correction. Note the contrast with §6.2: there the error entered as
a conjugation that partially cancels and vanishes at Δθ = 0; here it does not.

**Two ways in if measurement later says it matters**, in increasing order of disruption:

1. **Ask the controller for the plate pose.** At a standstill, have the program report the same pose
   in both conventions and let the HMI difference them to obtain the *controller's* `plate(θ)`
   instead of using its own model. One extra exchange, no change to how captures are reported.
2. **Report a coordinated weld's captures in a station work object** (`wobjTG_CamStn1`, the symmetric
   pair to `wobjTG_WeldStn1`). Registration output is then already plate-relative and no PC model
   enters at all — but it splits the capture convention, which is precisely what the owner call above
   declined. ⚠ This is *not* "capturing while the positioner moves" — no such capture exists.
   `ufprog:=FALSE` describes how the controller **derives** the frame (from the station's current
   angles), not whether anything is moving; the capture is still an ordinary standstill capture at
   one index.

The right first step for either is to **measure the error**, not to design around it. It is the same
robot↔positioner calibration figure the cell work is chasing anyway, and it is measurable
independently of this decision.

### Two requirements that do stand

- **Home and transition moves stay on `wobj0`.** True in both cases, but under a coordinated work
  object it stops being cosmetic: a rest-home move bound to the part's work object would follow the
  turntable.

⚠ **And one consequence further downstream, from the HMI's own composition.**
[abb_port_plan_v1.md](abb_port_plan_v1.md) §1.4.1 establishes (read off `RobotCell.cpp`) that the
HMI consumes exactly one reported pose — `R_C`'s — and lands the cloud in base via
`bTpart · act_pose · cloud`, where `bTpart` is *the frame the HMI itself sent*. With captures
base-referenced that still lands in base for **every** weld, coordinated included, which is the
point of keeping the capture convention uniform. The positioner-frame conversion happens once, on
the PC, at the moment the weld frame is served.

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

- **Controller check.** `wobjTG_WeldStn1` — the `ufprog:=FALSE`/`ufmec` shape — has not been loaded
  on a VC. Bundle it with the Weld Planner's outstanding coordinated `633-4` Arc VC check. Confirm
  the station name matches this controller's `MOC` mechanical unit while you are there.
- **Step 4B** (§4) — the composition premise this rule rests on, written and unrun.
- **Sequencing confirmation** (§3) — that the exporter indexes the positioner before requesting the
  frame on an indexed weld.
- **Measure the robot↔positioner calibration error** (§5) — it sizes the accepted cost of keeping
  captures base-referenced, and decides whether either fallback is ever needed.
- **Station 2** — `wobjTG_WeldStn2` when D4's two-station template lands.
- **Look-ahead (contract O-3) is unchanged** by this rule. `ufprog`/`ufmec` are never written at
  runtime and the `.oframe` nominal is assigned ahead of all motion, so only the per-weld `oframe`
  write is exposed, exactly as before.
