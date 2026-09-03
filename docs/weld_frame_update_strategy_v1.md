# Weld Frame Update Strategy — `uframe` or `oframe`? (v1)

Summary: **the request PROCs write `oframe`, always** — one write target, no case detection in RAPID.
**What the HMI sends is decided by one thing: whether the weld is coordinated.** A weld the
positioner does not move during — a static table, or a part held at an index on a 1- or 2-axis
positioner — gets an identity `uframe` and the CAD frame **w.r.t. the robot base**, exactly what the
HMI sends FANUC today. Only a genuine coordinated-motion weld gets `ufprog:=FALSE`/`ufmec:="STN1"`.

⚠ **The coordinated half is not settled, and this summary used to claim it was.** *What* a
coordinated weld serves — the part **w.r.t. the positioner frame**, or a **displacement** in that
frame, which is what the HMI's own (unfinished, untested) coordinated FANUC path computes today — is
an open decision, and so are the controller-side prerequisites for coordinated motion. **§5.** The
non-coordinated rule below stands on its own and does not wait on any of it.

This resolves **O-1** of the Weld Planner's `abb_hmi_request_contract_v1.md`, which left the write
target open and assumed `TG_ReqWeldFrame` would have to detect which case it was in. It does not.

**Status:** the non-coordinated case is implemented in this repo 2026-09-03 (`TG_Comms.sys`,
`TGS/TD05Test.mod`, `TGS/TD05Weld.mod`); the coordinated case is **work in progress on both sides of
the wire** (§5). **Not yet run on a controller** — the `oframe` write is numerically inert *provided
the live `uframe` is identity*, which is a property of the controller's persistent values and not of
the declarations (§4), and `wobjTG_WeldStn1` — the coordinated shape — has never been loaded and will
not load at all on a system with no external axis (§5.3).

⚠ **This document was revised twice the day it was written.** The first version branched on
*mounting* ("is the part bolted to the chuck?") rather than on *coordination*, which put every
station weld on the `ufmec` shape — §6 records why that was dropped, including the calibration
argument that turned out not to hold. The second put both shapes in one `wobjdata` and swapped the
record at entry; §3 records why they are now separate symbols. Read both before re-proposing either.

⚠ **And reviewed on 2026-09-03, which changed three things.** The coordinated payload turned out to
be a *displacement* rather than a frame in the HMI code this document claimed to mirror (§5.1); the
"`oframe` is free" argument turned out to rest on the controller's *live* `uframe`, not on the
declaration, so the exported program now has to normalize it (§4); and the sequencing rule's stated
reason was wrong, though its conclusion was not (§3).

---

## 1. The rule

| | Not coordinated — static table, **or indexed on a positioner** | Coordinated motion |
|---|---|---|
| `ufprog` | `TRUE` | `FALSE` |
| `ufmec` | `""` | `"STN1"` (the station) |
| `uframe` | **identity — and the program must assign it, not just inherit it (§4)** | computed live by the controller from the station angles; the declared value is ignored |
| `oframe` | the CAD/part frame **w.r.t. the robot base** | ⚠ **open (§5.1)** — the part w.r.t. the positioner frame, or a displacement in it |
| What the HMI sends | base-referenced — **identical to FANUC today** | ⚠ **open (§5.1)** — the HMI's coordinated path is unfinished and untested |
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

An exported program **picks one by passing it**, and assigns its frame nominals at entry:

```rapid
wobjTG_Weld.uframe := <identity>;                  ! normalize the assumption (§4)
wobjTG_Weld.oframe := <part w.r.t. robot base>;    ! the nominal the points were divided by
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
as `tTG_Weld`, and the same **E47**-class hazard. A wrong `ufmec` is a silent wrong-station weld; an
`ufmec` naming no configured unit is worse in one way and better in another, since it fails loudly
and takes the whole shared `TG_Comms.sys` with it (§5.3). And the larger **E47** exposure is not the
name at all but the station's **base frame**: `uframe(θ)` is derived from the `MOC` calibration, so if
that disagrees with the positioner calibration the PC uses, every coordinated weld carries a silent
rigid offset. Treat all of it as cell configuration and verify it at commissioning.

⚠ **Sequencing, and this is the one thing to get right in the exported program — but the reason is
not the obvious one.** A base-referenced frame describes the part at **the angle the Weld Planner
authored for that weld**, not at the angle the station happens to be sitting at: the HMI rotates its
localization result from the capture angle to the weld's `tool_path_parameters.positioner_angle`
(`Weld::GetMyPositionerZAxisRotationTransformation`, `bTpos · Rz(Δθ) · inv(bTpos)`) and **never reads
the robot's station position**. Nothing on the wire carries it.

So the served value does not depend on *when* the request runs, and the real requirement is the
stronger, checkable one: **the program's index must equal the angle the Planner authored** for that
weld when the weld motion runs. That holds by construction for a Weld-Planner-exported program, since
both numbers come from the same weld record. Keep the safe order regardless — index, then request the
frame, then weld — and once a frame has been served, do not re-assign either component before the
weld that consumes it. The failure is silent and geometric either way.

**Station 2 (D4) is deliberately not declared yet.** `wobjTG_WeldStn2` with `ufmec:="STN2"` is a
one-line addition when the two-station template lands; declaring it now would add a resident symbol
nothing references.

## 4. Why the `oframe` switch is free — provided `uframe` really is identity

`TG_Comms.sys` declares both base-referenced work objects with an **identity `uframe` and an
identity `oframe`**.
A `wobjdata` resolves as `world ← uframe ← oframe ← target`, so motion and `CRobT` alike see only
the product `uframe · oframe`: with an identity `uframe`, writing the served frame to `oframe`
instead of `uframe` is the same geometry. Nothing else in the repo reads `.uframe`.

⚠ **"Identity `uframe`" is a claim about the controller's LIVE value, not about the declaration — and
that is why the exported program has to assign it.** A `PERS` keeps whatever it was last assigned,
across program loads and across a saved station; the declaration's initial value only applies to a
module loaded fresh. So a controller that ever ran the **pre-2026-09-03 code, which served the frame
into `uframe`**, still holds a frame there. Compose it with the new `oframe` write and every pose is
double-transformed — silently, with a watch on `oframe` showing exactly the expected numbers, and
with the HMI's own `bTpart · act_pose · cloud` composition (§5) off by that same stale frame.

**Requirement, therefore:** an exported program assigns `uframe := identity` at entry, next to its
`.oframe` nominal, for every base-referenced work object it uses. `TGS/TD05Test.mod` and
`TGS/TD05Weld.mod` both do; **the Weld Planner ABB exporter must too.** This is a one-time
normalization of an assumption, not a frame write — the rule against writing `uframe` is about the
*served* frame, and §2 is the reason it can never be the right destination for one. It also makes the
next VC run trustworthy: verify the live `uframe` is identity before reading anything into the
`oframe` numbers.

⚠ **The composition is manual-backed, not yet measured.** 3HAC050917-001's `wobjdata` definition is
the source. `NON-ESSENTIAL-NICE-TO-HAVE/TGToolFrameSet.mod` step **4B** is written to measure it
(both a `uframe` and an `oframe` shift of −100 mm should move the TCP identically) and has **not
been run**. Two minutes on the same VC session as the coordinated load-check.

## 5. The coordinated case — what the HMI sends is not settled, and the rest is WIP

⚠ **Read this before building anything coordinated.** §1's rule is settled for the non-coordinated
case and only for that case. The ABB side is not the unfinished half on its own: **the HMI's
coordinated FANUC path is itself unfinished and untested** (owner, 2026-09-03), so what a coordinated
weld serves is a decision still to be made rather than a contract to mirror. Everything in this
section is work in progress.

### 5.1 The open question: is the served pose a frame, or a displacement?

The first version of this document assumed the coordinated payload was the part's pose **w.r.t. the
positioner frame** — the absolute reading, and the direct analogue of the base-referenced case. What
the HMI computes today for a coordinated weld is a **displacement**:

```
// RobotCell::getWeldFrameToSendToRobot -> Weld::GetCoordinatedOffsetInPositionerFrame
offset_pos = touchup_pos ∘ [ inv(bTpos(θc)) · (corrected_bTcad · inv(nominal_bTcad)) · bTpos(θc) ]
```

— identity when the localization matches nominal, and destined for FANUC's `OFFSET,PR[n]` with
`$OFFSET_CART=TRUE` applied to every coordinated move. The non-coordinated branch of the same
function returns `getLastLocalizationTransformation()`, which *is* an absolute pose in base. **The two
FANUC cases already differ in kind**, so there is no single "the HMI sends the CAD w.r.t. X" pattern
to inherit.

⚠ The two quantities differ by **exactly the nominal part-on-plate mount transform**, so they are not
interchangeable: write the displacement into an `oframe` whose robtargets were divided by the part
frame and the weld is mislocated by the part's offset from plate centre, silently.

**Both conventions are implementable, and `TG_ReqWeldFrame` does not care either way** — it writes
`.oframe` on whatever it is handed. What the choice changes is what the *exporter* divides the
robtargets by:

| | (A) absolute | (B) displacement |
|---|---|---|
| robtargets divided by | the nominal part frame on the plate | the **plate** frame — nominal mount baked into the points |
| `oframe` nominal at entry | the nominal part-on-plate transform | **identity** |
| served value | `inv(bTpos(θc)) · corrected_bTcad` | today's `offset_pos`, unchanged |
| HMI work | a new brand-conditional computation | **none** |
| what `oframe` means | the part frame, as in the non-coordinated case | a displacement — a second meaning for one field |

**(B) is the recommendation**, for three reasons: the wire payload stays identical to FANUC's, which
is the stated goal of the whole exercise; `uframe(θ) · oframe · target` reproduces FANUC's
`uframe(θ) · Offset · P` exactly, so the composition order and the left-multiply match; and it is
better-conditioned, per §5.2. Its cost is the asymmetry in what `oframe` means between the two work
objects, which has to be stated loudly wherever either is declared. Neither option is committed to
yet, and the HMI's coordinated path needs finishing either way.

### 5.2 What keeping captures base-referenced costs — and why the figure is contingent

**Captures stay base-referenced, on both brands.** *(Owner call, 2026-09-03.)* Every capture is taken
at a standstill at one index, and reporting them all in the base-referenced `wobjTG_Cam` keeps the
HMI's scan and registration path one code path across FANUC and ABB. There is **no coordinated camera
work object**, and none should be added without a measurement to justify it.

The accuracy consequence depends on which convention §5.1 lands on. The first version of this
document stated only the worse of the two, as if it were settled:

- **Under (A)** the HMI must produce `oframe = inv(plate_pc(θs)) · P_base`, a **single left-multiply**
  by the PC's own positioner model — so a calibration error there lands on the weld at full magnitude,
  with nothing downstream to absorb it, because this *is* the vision correction.
- **Under (B)** the model enters as the **conjugation** `inv(bTpos) · Δ · bTpos` — the same structure
  §6.2 works through, which cancels at Δ = identity and scales with the size of the correction rather
  than with the part's distance from the axis.

So (B) is not merely free, it is the better arithmetic, and under it the two fallbacks below are
probably never needed. ⚠ Either way the right first step is to **measure the robot↔positioner
calibration error**, not to design around it: it is the same figure the cell work is chasing anyway,
and it is measurable independently of this decision.

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

### 5.3 Controller-side prerequisites, none of them closed (WIP)

Listed so the coordinated path is not mistaken for one line of RAPID away from working. All of these
are being worked on as of 2026-09-03:

- **`ActUnit`.** A `ufmec` station must be an **active mechanical unit** before a move — or a `CRobT`
  — in its work object resolves. Nothing in this repo activates one, and §2's "no activate step exists
  or is needed" is true only of the base-referenced objects.
- **Which motion task the station lives in.** Coordination is straightforward for a mechanical unit in
  the robot's own task; a MultiMove group needs `SyncMoveOn` and a different option set. The Weld
  Planner models the station as `independent_group` today (`_MOTION_ROLE_EXTJOINT_SLOTS`), which is
  not the same thing.
- **`extjoint` values.** Coordinated motion needs real station values in every robtarget; the exporter
  writes `9E9` for every axis it does not command (plan **D5**).
- **The station's base frame.** `uframe(θ)` is derived from the station's `MOC` calibration, so that
  must agree with the `bTpos` the PC uses. A disagreement is a silent rigid offset on **every**
  coordinated weld — **E47** one level up, and a larger risk than a mistyped `ufmec`.
- **The declaration is not loadable everywhere.** `wobjTG_WeldStn1` lives in the shared
  `TG_Comms.sys`, and the phase 1–6 validation VC (RW6.15.08 / IRB4600) has no external axis at all.
  If a controller rejects an unresolvable `ufmec` at load or at Check Program, it takes the *already
  validated* non-coordinated path down with it. The declaration carries the instruction to comment it
  out, or to rename the station, per test cell — treat that one `!` as the rollback.

### 5.4 Two requirements that do stand

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

**Non-coordinated (the implemented case).**

- **Controller check.** The `oframe` switch has not been run. Confirm the live `uframe` is identity
  first (§4) — on a station that ran the old code it will not be.
- **Step 4B** (§4) — the composition premise this rule rests on, written and unrun.
- **The exporter's `uframe := identity` at entry** (§4), and its `.oframe` nominal assigned ahead of
  all motion. Neither is optional; the first is new as of the 2026-09-03 review.
- **Sequencing** (§3) — that the program's index equals the weld's authored `positioner_angle`. True
  by construction for an exported program; worth a spot-check on one real project rather than an
  assumption.

**Coordinated (work in progress, and none of it blocks the above).**

- **What the payload is** (§5.1) — absolute part-on-plate pose, or a displacement in the positioner
  frame. Recommendation is (B), the displacement, because it needs no HMI change and is
  better-conditioned; the HMI's coordinated FANUC path is unfinished and untested, so this is a
  decision to make rather than a contract to mirror.
- **The controller-side prerequisites** (§5.3) — `ActUnit`, motion-task vs MultiMove, real `extjoint`
  values in place of `9E9`, and the station base-frame calibration.
- **`wobjTG_WeldStn1` has never been loaded**, and will not load on a system with no external axis.
  The declaration carries the comment-out instruction; bundle the load-check with the Weld Planner's
  outstanding coordinated `633-4` Arc VC check and confirm the station name against `MOC` while there.
- **Measure the robot↔positioner calibration error** (§5.2) — it sizes the accepted cost of keeping
  captures base-referenced, and its magnitude depends on which convention §5.1 lands on.
- **Station 2** — `wobjTG_WeldStn2` when D4's two-station template lands.
- **Look-ahead (contract O-3) is unchanged** by this rule. `ufprog`/`ufmec` are never written at
  runtime and the frame nominals are assigned ahead of all motion, so only the per-weld `oframe`
  write is exposed, exactly as before.

**Peer repos.** The Weld Planner's `abb_hmi_request_contract_v1.md` still lists `WObj.uframe` with
**O-1** open, its plan still says "`.oframe` for a coordinated station, `.uframe` for a static one",
and the landed `no_hmi` emitter (`AbbTranslator._wobj_token`) puts the program frame in **`uframe`**
with an identity `oframe`. That last one matters most: a module in that shape plus a runtime `.oframe`
write is the double transform of §4. The Weld Planner team is aware and will update the plan and the
implementation; nothing here waits on it.
