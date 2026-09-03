MODULE TGToolFrameSet_Mod
    !***********************************************************************
    ! TGToolFrameSet - how RAPID updates a tooldata (TCP translation +
    ! orientation) and a work object frame AT RUNTIME, with numeric
    ! read-back at every step.
    !
    ! Written 2026-09-01 for the Weld Planner "set the tooldata on the fly"
    ! case. NOT YET VC-VALIDATED: every "expect" below is a PREDICTION to be
    ! confirmed or corrected by the first run.
    !
    ! ------------------------- THE SHORT ANSWER -------------------------
    ! There is no SetTool / SetFrame instruction. A tooldata and a wobjdata
    ! are ordinary records, and updating one is a plain assignment:
    !
    !     tTG_Weld.tframe.trans := [108.5, 0.0, 607.0];      ! mm, in tool0
    !     tTG_Weld.tframe.rot   := OrientZYX(180, -45, 0);   ! deg, Rz Ry Rx
    !     wobjTG_Weld.uframe    := pReceivedFrame;           ! a pose
    !
    ! and there is no "activate" step. FANUC needs UTOOL[n]=PR[n] FOLLOWED
    ! BY UTOOL_NUM=n; RAPID does not. The tool argument and \WObj of a
    ! motion instruction name the record itself, and the controller reads
    ! whatever that record holds when the instruction is PREPARED - which is
    ! exactly where the one real hazard lives (step 4).
    !
    ! Record shapes (RAPID Instructions, Functions and Data types,
    ! 3HAC050917-001):
    !   tooldata = [ robhold, tframe, tload ]
    !     tframe = pose     = [ trans:pos, rot:orient ]
    !     tload  = loaddata = [ mass, cog:pos, aom:orient, ix, iy, iz ]
    !   wobjdata = [ robhold, ufprog, ufmec, uframe:pose, oframe:pose ]
    !
    ! ------------------------- THE FOUR RULES ---------------------------
    ! R1  PERS, not VAR. A VAR tooldata write is lost at every PP-to-main
    !     and cannot be handed to a \PERS parameter - that is finding F-3
    !     again ("Argument error(123) ... not a persistent reference").
    !     PERS also survives power fail and is what an RWS symbol write can
    !     reach, so PERS is what makes "on the fly" mean anything at all.
    ! R2  Build orientations with OrientZYX, never by hand. A quaternion
    !     that is not normalised is rejected when it is USED; anything
    !     arriving from the PC side goes through NOrient first (step 1D).
    ! R3  A write needs a MOTION BARRIER in front of it. The assignment runs
    !     in the program task while the motion planner has already prepared
    !     queued instructions against the OLD record. Land on a `fine` point
    !     (or StopMove/StartMove) before writing - step 4.
    ! R4  Every taught robtarget moves. A robtarget is stored in
    !     tool + wobj coordinates, so changing the TCP or the frame
    !     relocates every point already programmed against it. That is the
    !     feature when TG_ReqWeldFrame serves a correction, and the trap
    !     when someone "just fixes" the torch TCP after the fact.
    !
    ! Deliberately standalone, like TGArcCheck.mod: no dependency on
    ! TG_Comms.sys / TG_Cell.sys / TG_Main.mod, and it never touches
    ! tTG_Weld or wobjTG_Weld - it carries its own copies so a run cannot
    ! disturb the comms modules. Requires NO options. Load it alone and run
    ! the PROCs by hand.
    !
    ! Steps 1, 2, 3 and 5A are DATA ONLY - no motion, safe anywhere.
    ! Step 4 moves the arm. Per-step run instructions and pass criteria are
    ! in the step headers; transcribe them into docs/robotstudio_setup.md
    ! once the run happens.
    !***********************************************************************

    ! ------------------------ the data under test ------------------------
    ! Global PERS (not LOCAL) so both records are visible and editable in
    ! the RobotStudio / FlexPendant data view during a run - watching them
    ! change there is half the demonstration.
    !
    ! Starting value = the literal from the question, verbatim. It is a
    ! plausible 45-degree torch: rot [0, 0.382683, 0, 0.923880] has scalar
    ! q1 = 0, i.e. a 180 deg turn about the axis (0.382683, 0, 0.923880),
    ! which works out to tool x = (-0.7071, 0, 0.7071), tool y = (0, -1, 0),
    ! tool z = (0.7071, 0, 0.7071) - the tool z pointing 45 deg between the
    ! wrist +x and +z. Step 1 shows OrientZYX(180, -45, 0) reproduces it.
    PERS tooldata tTGFS_Tool:=[TRUE,[[108.500,0.000,607.000],[0.000000,0.382683,0.000000,0.923880]],[12.0,[-20.8,0.0,282.7],[1,0,0,0],0,0,0]];

    ! Static (non-coordinated) work object: robhold FALSE, ufprog TRUE.
    ! ufprog TRUE is what makes writing .uframe legal - see step 3C.
    PERS wobjdata wobjTGFS:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];

    ! Restore source, so every step starts from a known record and the
    ! module is re-runnable. This matters more than it looks: PERS values
    ! SURVIVE a program reset, so a half-finished run leaves the tool
    ! modified and the next run starts from garbage. CONST, so no step can
    ! corrupt the reference by accident.
    LOCAL CONST tooldata tTGFS_Decl:=[TRUE,[[108.500,0.000,607.000],[0.000000,0.382683,0.000000,0.923880]],[12.0,[-20.8,0.0,282.7],[1,0,0,0],0,0,0]];
    LOCAL CONST wobjdata wobjTGFS_Decl:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];

    ! Zero-offset probe tool = the wrist flange. Step 4 reads the FLANGE
    ! position to prove a TCP change took effect: reading the TCP with the
    ! tool you just changed always returns the programmed target, so it
    ! proves nothing. A LOCAL PERS of our own rather than tool0 keeps the
    ! module standalone and independent of whatever tool0's load literal is
    ! on this controller; PERS because CPos's \Tool is a \PERS parameter
    ! (F-3). mass 0.001 rather than 0 - see step 5B.
    LOCAL PERS tooldata tTGFS_Flange:=[TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];

    ! -------------------- step 3 / step 4 targets --------------------
    ! Three taught points for the DefFrame demonstration (step 3D). CONST
    ! rather than jogged-and-CRobT'd so step 3 stays motionless; the CRobT
    ! teach variant is shown as a comment there.
    ! p1 = origin, p1->p2 = +x, p3 = the +y side of the xy plane. Chosen so
    ! the answer is a round number: +x along world +Y, +y along world -X,
    ! +z still up  =>  a pure 90 deg turn about world z at [1000, 0, 500].
    LOCAL CONST robtarget rtFrmP1:=[[1000,0,500],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtFrmP2:=[[1000,200,500],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtFrmP3:=[[800,0,500],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    ! Step 4 probe target. Orientation [0,0,1,0] = 180 deg about y, i.e. the
    ! TOOL FRAME pointing straight down - note that this constrains the tool
    ! frame, not the flange, which is the whole point of step 4A.
    ! IRB 4600-20/2.50: with the 620 mm torch above, the flange lands at
    ! about [1308.5, 0, 1507] for this target - well inside the 2.50 m.
    LOCAL CONST robtarget rtProbe:=[[1200,0,900],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST jointtarget jtSafe:=[[0,0,0,0,30,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    ! Settle time for step 4's reads. Same reasoning and same 0.2 s as
    ! tgSendPose (rapid_validation_findings_v1.md, "related observation"): a
    ! fine point releases execution before the servos have finished
    ! converging, and step 4 judges by millimetres. PERS so it is tunable
    ! from the data view without a code reload; 0 disables it.
    LOCAL PERS num nTGFS_Settle:=0.2;

    !***********************************************************************
    ! RESTORE - put both records back to their declared values.
    !
    ! Every step below calls this on entry, so the steps are independent and
    ! re-runnable in any order. Run it by hand before walking away, because
    ! PERS survives the program reset that would otherwise clean up.
    ! No motion, safe anywhere.
    !***********************************************************************
    PROC TGToolFrameReset()
        tTGFS_Tool:=tTGFS_Decl;
        wobjTGFS:=wobjTGFS_Decl;
        TPWrite "TG TFSET: records restored to declared values";
        TPWrite "  tool trans ="\Pos:=tTGFS_Tool.tframe.trans;
        TPWrite "  tool rot   ="\Orient:=tTGFS_Tool.tframe.rot;
    ENDPROC

    !***********************************************************************
    ! STEP 1 - the assignment itself. DATA ONLY: no motion, no welding, safe
    ! to run anywhere at any time, robot in manual.
    !
    ! Four sub-steps:
    !   A  component-wise write - the normal way, and the only way that
    !      leaves the load data alone.
    !   B  whole-record write - one literal, for when the PC side supplies
    !      the complete tool (it overwrites tload too; that is the point).
    !   C  Euler round trip - OrientZYX in, EulerZYX back out.
    !   D  NOrient on a rounded quaternion - what to do with an orient that
    !      arrived over the wire.
    !
    ! PASS CRITERION (predicted):
    !   1A trans = [108.5, 0.0, 607.0], rot = [0, 0.382683, 0, 0.923880]
    !   1B identical to 1A, and mass back to 12.0
    !   1C Z = 180 (or -180), Y = -45, X = 0
    !   1D norm before less than 1, norm after = 1.000000
    !
    ! SIGN NOTE, and it is not a defect: a quaternion and its negation are
    ! the SAME rotation, and this orient has q1 = 0, where the usual
    ! "keep q1 >= 0" tie-break decides nothing. A read-back of
    ! [0, -0.382683, 0, -0.923880] therefore PASSES. Consequence for the
    ! generator: any orient comparison must be SIGN-INSENSITIVE - compare q
    ! against both +q and -q, or compare the rotation, never the four
    ! numbers straight.
    !***********************************************************************
    PROC TGToolSet()
        ! RAPID requires every declaration ahead of the first instruction of
        ! the routine, so 1D's locals live up here rather than next to their
        ! use. Not a style choice.
        VAR orient oRounded;
        VAR orient oFixed;
        VAR num nNorm;

        TPWrite "TG TFSET: ---- step 1: tooldata assignment ----";
        TGToolFrameReset;

        ! --- 1A: component-wise, deliberately from a clean slate ---
        tTGFS_Tool.tframe.trans:=[0,0,0];
        tTGFS_Tool.tframe.rot:=[1,0,0,0];

        ! Translation: the TCP offset from tool0 (the wrist flange),
        ! expressed in the tool0 frame, in MILLIMETRES. RAPID has no unit
        ! setting to get wrong here - unlike welddata.weld_speed, whose unit
        ! the arc system's ARC_UNITS governs (TGArcCheck step 2).
        tTGFS_Tool.tframe.trans:=[108.500,0.000,607.000];

        ! Orientation: OrientZYX(anglez, angley, anglex), DEGREES, applied z
        ! then y then x about the successively rotated axes - i.e. the
        ! matrix product Rz * Ry * Rx. This is the RAPID counterpart of the
        ! w/p/r the KAREL side used to send, so a PC-side Euler triple can
        ! be passed straight in and the quaternion conversion dropped.
        tTGFS_Tool.tframe.rot:=OrientZYX(180,-45,0);

        TPWrite "  1A trans ="\Pos:=tTGFS_Tool.tframe.trans;
        TPWrite "  1A rot   ="\Orient:=tTGFS_Tool.tframe.rot;

        ! A single scalar component is reachable too, for a one-axis nudge:
        !   tTGFS_Tool.tframe.trans.z := 617.0;
        ! trans.z is a plain num, so that really is a scalar edit - but it
        ! is still a TCP change, so rule R3 (motion barrier) applies to it
        ! exactly as it does to a whole-pose write.

        ! --- 1B: the whole record in one literal ---
        ! Use this when the PC side sends a complete tool. It replaces tload
        ! as well, so a partial literal silently zeroes the load - a
        ! servo-tuning and collision-detection problem, not a cosmetic one.
        ! Write tframe alone (1A) unless the load is genuinely part of the
        ! update.
        tTGFS_Tool:=[TRUE,[[108.500,0.000,607.000],[0.000000,0.382683,0.000000,0.923880]],[12.0,[-20.8,0.0,282.7],[1,0,0,0],0,0,0]];
        TPWrite "  1B trans ="\Pos:=tTGFS_Tool.tframe.trans;
        TPWrite "  1B rot   ="\Orient:=tTGFS_Tool.tframe.rot;
        TPWrite "  1B mass  ="\Num:=tTGFS_Tool.tload.mass;

        ! --- 1C: Euler round trip ---
        ! EulerZYX(\X | \Y | \Z, orient) -> num, degrees, one switch per
        ! call. Use it to log a tool in human terms, or to check that what
        ! the PC sent is what the controller ended up holding.
        TPWrite "  1C EulerZYX Z ="\Num:=EulerZYX(\Z:=tTGFS_Tool.tframe.rot);
        TPWrite "  1C EulerZYX Y ="\Num:=EulerZYX(\Y:=tTGFS_Tool.tframe.rot);
        TPWrite "  1C EulerZYX X ="\Num:=EulerZYX(\X:=tTGFS_Tool.tframe.rot);
        ! Expect Z = 180 or -180 (both name the same rotation). Y is the
        ! angle pinned to -90..+90, so Y = -45 is the load-bearing number
        ! here. X = 0.

        ! --- 1D: NOrient on a quaternion that came in rounded ---
        ! An orient MUST satisfy q1^2+q2^2+q3^2+q4^2 = 1. A quaternion
        ! printed to four decimals by the PC side does not, and the
        ! controller rejects a non-normalised orient when it is USED, not
        ! when it is assigned - so the fault surfaces at the next motion
        ! instruction, far from the write that caused it. NOrient fixes it
        ! in one call. Rule for the exporter: every orient that crosses the
        ! wire gets NOrient'd on arrival, inside tgTryStrToPose or
        ! immediately after it.
        ! Components are assigned at runtime rather than written as a
        ! declaration literal so that nothing can be validated (or silently
        ! fixed) at load time - the "before" norm has to be genuinely wrong
        ! for this probe to say anything.
        oRounded.q1:=0;
        oRounded.q2:=0.3827;
        oRounded.q3:=0;
        oRounded.q4:=0.9239;
        nNorm:=Sqrt(oRounded.q1*oRounded.q1+oRounded.q2*oRounded.q2+oRounded.q3*oRounded.q3+oRounded.q4*oRounded.q4);
        TPWrite "  1D norm before ="\Num:=nNorm;
        oFixed:=NOrient(oRounded);
        nNorm:=Sqrt(oFixed.q1*oFixed.q1+oFixed.q2*oFixed.q2+oFixed.q3*oFixed.q3+oFixed.q4*oFixed.q4);
        TPWrite "  1D norm after  ="\Num:=nNorm;
        TPWrite "  1D rot   ="\Orient:=oFixed;

        TPWrite "TG TFSET: step 1 done - compare against the header";
    ENDPROC

    !***********************************************************************
    ! STEP 2 - RELATIVE updates, which is what "on the fly" usually means:
    ! stickout compensation, tip wear, a nudge found during touch-up.
    ! DATA ONLY, safe anywhere.
    !
    ! PoseMult(p1, p2) composes two poses. Which side the delta goes on is
    ! the entire content of this step:
    !
    !   PoseMult(tframe, delta)  - delta is read in the TOOL frame.
    !       "10 mm further along the wire", "rotate the torch about its own
    !       y". This is the one for stickout and tip wear.
    !   PoseMult(delta, tframe)  - delta is read in the tool0 / WRIST frame.
    !       "the whole torch is mounted 15 deg off". This one MOVES the TCP
    !       even though the delta has no translation, because the existing
    !       offset is rotated along with it.
    !
    ! PASS CRITERION (predicted, all printed below):
    !   2A stickout +10 mm along tool z:
    !        trans [115.571, 0.000, 614.071]  (108.5+7.071, 607+7.071)
    !        rot   UNCHANGED [0, 0.382683, 0, 0.923880]
    !        printed step = 10.000
    !   2B rotate +15 deg about TOOL y:
    !        trans UNCHANGED [108.500, 0.000, 607.000]
    !        rot   [0, 0.258819, 0, 0.965926] = OrientZYX(180, -30, 0)
    !   2C rotate +15 deg about WRIST y:
    !        trans MOVED to [261.906, 0.000, 558.235]
    !        rot   [0, 0.500000, 0, 0.866025] = OrientZYX(180, -60, 0)
    !
    ! Why 2B reads -30 and 2C reads -60 when both asked for +15: this tool
    ! is flipped 180 deg about z, and Ry(15)*Rz(180) = Rz(180)*Ry(-15), so a
    ! wrist-frame +15 shows up as -15 in the ZYX decomposition
    ! (-45 - 15 = -60) while a tool-frame +15 adds the other way
    ! (-45 + 15 = -30). Not a bug and not a convention mismatch - it is the
    ! 180 deg flip, and anyone reading Euler angles off a flipped tool has
    ! to know it.
    !***********************************************************************
    PROC TGToolSetRel()
        VAR pos pBefore;
        VAR pose pInv;

        TPWrite "TG TFSET: ---- step 2: relative tooldata updates ----";

        ! --- 2A: extend the TCP 10 mm along its own z (stickout) ---
        TGToolFrameReset;
        pBefore:=tTGFS_Tool.tframe.trans;
        tTGFS_Tool.tframe:=PoseMult(tTGFS_Tool.tframe,[[0,0,10],[1,0,0,0]]);
        TPWrite "  2A trans ="\Pos:=tTGFS_Tool.tframe.trans;
        TPWrite "  2A rot   ="\Orient:=tTGFS_Tool.tframe.rot;
        TPWrite "  2A step, mm ="\Num:=Distance(pBefore,tTGFS_Tool.tframe.trans);
        ! Distance must read exactly 10.000: the delta is a pure translation
        ! in the tool frame and composition preserves its length. A wrong
        ! number here means the argument order is wrong, not the arithmetic.
        ! There is a PoseMult-free shorthand for this one case - add
        ! 10 * (the tool z as a unit pos) to trans - but the tool z has to
        ! be extracted first, and it does not generalise to a rotation.
        ! Prefer PoseMult.

        ! --- 2B: rotate 15 deg about the TOOL y ---
        TGToolFrameReset;
        tTGFS_Tool.tframe:=PoseMult(tTGFS_Tool.tframe,[[0,0,0],OrientZYX(0,15,0)]);
        TPWrite "  2B trans ="\Pos:=tTGFS_Tool.tframe.trans;
        TPWrite "  2B rot   ="\Orient:=tTGFS_Tool.tframe.rot;
        TPWrite "  2B EulerZYX Y ="\Num:=EulerZYX(\Y:=tTGFS_Tool.tframe.rot);

        ! --- 2C: the same 15 deg, about the WRIST y ---
        TGToolFrameReset;
        tTGFS_Tool.tframe:=PoseMult([[0,0,0],OrientZYX(0,15,0)],tTGFS_Tool.tframe);
        TPWrite "  2C trans ="\Pos:=tTGFS_Tool.tframe.trans;
        TPWrite "  2C rot   ="\Orient:=tTGFS_Tool.tframe.rot;
        TPWrite "  2C EulerZYX Y ="\Num:=EulerZYX(\Y:=tTGFS_Tool.tframe.rot);

        ! --- 2D: the inverse, for completeness ---
        ! PoseInv(tframe) is the flange expressed in the tool frame. Useful
        ! when a measurement was taken the other way round (a fixture
        ! measured relative to the torch tip, say), and it is how a
        ! composition is undone without keeping the original around:
        !   tframe := PoseMult(tframe, PoseInv(deltaAppliedEarlier));
        ! For a single point rather than a pose the function is
        ! PoseVect(tframe, p) - "where does this tool-frame point land on
        ! the flange" - not PoseMult.
        TGToolFrameReset;
        ! Staged in a VAR rather than written as PoseInv(...).trans:
        ! selecting a component off a function RESULT is not something
        ! the RAPID grammar promises, and TG_Comms already stages CRobT
        ! the same way for the same reason.
        pInv:=PoseInv(tTGFS_Tool.tframe);
        TPWrite "  2D flange in tool frame ="\Pos:=pInv.trans;

        TGToolFrameReset;
        TPWrite "TG TFSET: step 2 done - 2B trans UNCHANGED, 2C trans MOVED";
    ENDPROC

    !***********************************************************************
    ! STEP 3 - the work object frame. DATA ONLY, safe anywhere.
    !
    ! wobjdata = [robhold, ufprog, ufmec, uframe, oframe], and a target in
    ! this wobj resolves as   world <- uframe <- oframe <- target,
    ! so uframe and oframe COMPOSE. Step 4B measures that claim.
    !
    !   3A  uframe write - the static case. The shape TG_ReqWeldFrame used
    !       to have; it now writes .oframe in every case
    !       (weld_frame_update_strategy_v1), which is why step 4B's
    !       composition claim is load-bearing rather than a curiosity. So a
    !       "set the tool on the fly" request is the same shape with
    !       Tool.tframe := pFrame instead. Both arrive from tgTryStrToPose,
    !       which already yields a `pose` - exactly the type both fields
    !       want, no conversion anywhere.
    !   3B  oframe write - a correction ON TOP of a frame we must not touch.
    !   3C  the coordinated case (D3 / E28) - why 3B exists.
    !   3D  DefFrame - a frame from three measured points, the RAPID
    !       equivalent of FANUC's 3-point UFRAME teach.
    !
    ! PASS CRITERION (predicted):
    !   3A uframe = [[1000, 0, 500], [1, 0, 0, 0]]
    !   3B oframe = [[0, 0, -100], [1, 0, 0, 0]], uframe untouched
    !   3D pose   = [[1000, 0, 500], [0.707107, 0, 0, 0.707107]], Z = 90
    !
    ! 3D IS THE UNVERIFIED ONE. That value assumes DefFrame's convention is
    ! p1 = origin, p1->p2 = +x, p3 on the +y side of the xy plane, which for
    ! these three points puts +x along world +Y and +y along world -X, i.e.
    ! a pure +90 deg about world z. If the print differs, THE PRINT IS THE
    ! TRUTH - correct this comment from it, and check the optional \Origin
    ! switch in 3HAC050917-001 before relying on a non-default origin.
    !***********************************************************************
    PROC TGFrameSet()
        VAR pose pFrame;

        TPWrite "TG TFSET: ---- step 3: work object frame ----";
        TGToolFrameReset;

        ! --- 3A: the whole user frame, as a request handler does it ---
        pFrame.trans:=[1000,0,500];
        pFrame.rot:=[1,0,0,0];
        wobjTGFS.uframe:=pFrame;
        TPWrite "  3A uframe trans ="\Pos:=wobjTGFS.uframe.trans;
        TPWrite "  3A uframe rot   ="\Orient:=wobjTGFS.uframe.rot;
        ! Component-wise works here too, same as the tool:
        !   wobjTGFS.uframe.trans := [1000, 0, 500];
        !   wobjTGFS.uframe.rot   := OrientZYX(90, 0, 0);

        ! --- 3B: a correction into oframe, uframe left alone ---
        ! oframe is expressed IN uframe, so this means "100 mm down in the
        ! object's own frame", not "100 mm down in world" - the two coincide
        ! here only because 3A's uframe has identity rotation.
        wobjTGFS.oframe.trans:=[0,0,-100];
        wobjTGFS.oframe.rot:=[1,0,0,0];
        TPWrite "  3B oframe trans ="\Pos:=wobjTGFS.oframe.trans;
        TPWrite "  3B uframe trans still ="\Pos:=wobjTGFS.uframe.trans;

        ! --- 3C: why 3B exists (decision D3, open item E28) ---
        ! A coordinated work object riding a positioner is declared
        !     [FALSE, FALSE, "STN1", ...]
        !             ufprog = FALSE, ufmec = the mechanical unit name.
        ! With ufprog FALSE the SYSTEM computes uframe from the positioner's
        ! measured position on every interpolation step. Writing it is not
        ! an error - it is worse than an error, because the write is
        ! silently overwritten and the correction simply disappears. The
        ! correction belongs in OFRAME, which the system does not own.
        ! Not exercised here: this module has no positioner and must run on
        ! a bare VC. When the coordinated cell exists, re-run step 4B
        ! against a ufprog FALSE wobj - that is the test that closes E28.

        ! --- 3D: three-point frame definition ---
        ! DefFrame(p1, p2, p3) -> pose, built from the POSITIONS of three
        ! robtargets. All three must be expressed in the same reference
        ! frame the result will be interpreted in - here wobj0/world, since
        ! the result is destined for a uframe.
        ! On the real cell the three come from jogging the torch tip to the
        ! three corners and reading it back, the direct analogue of the
        ! FANUC teach:
        !   rtFrmP1 := CRobT(\Tool:=tTGFS_Tool \WObj:=wobj0);   ! at corner 1
        ! behind the settle ladder of step 4, and with the SAME tool for all
        ! three points - a different TCP between points tilts the frame.
        pFrame:=DefFrame(rtFrmP1,rtFrmP2,rtFrmP3);
        TPWrite "  3D DefFrame trans ="\Pos:=pFrame.trans;
        TPWrite "  3D DefFrame rot   ="\Orient:=pFrame.rot;
        TPWrite "  3D EulerZYX Z ="\Num:=EulerZYX(\Z:=pFrame.rot);

        TGToolFrameReset;
        TPWrite "TG TFSET: step 3 done";
    ENDPROC

    !***********************************************************************
    ! STEP 4 - MOTION. The proof that a write took effect, and the reason
    ! the motion barrier exists. Run on the VC, or with a clear cell: it
    ! moves the arm.
    !
    ! 4A - TCP change. The SAME programmed target, before and after the
    !   stickout change of step 2A, read with two different tools:
    !     with tTGFS_Tool   -> must NOT move. The target says where the TCP
    !        goes, so a longer TCP still lands on the target. Reading the
    !        tool you just changed can never show you the change; this line
    !        exists to make that concrete.
    !     with tTGFS_Flange -> MUST move exactly 10.0 mm. The arm had to
    !        retract to keep the longer TCP on the target. That is the
    !        actual evidence.
    !   PASS: printed TCP step ~ 0 mm (settle noise only, <= 0.05 mm behind
    !         the ladder), printed FLANGE step = 10.0 mm +/- 0.05.
    !   Predicted absolutes, so the transcript can be checked line by line:
    !         flange [1308.500, 0.000, 1507.000] before,
    !                [1315.571, 0.000, 1514.071] after.
    !
    ! 4B - frame change, and the uframe/oframe composition claim of step 3.
    !   The same target in wobjTGFS, three times:
    !     uframe identity                  -> reference
    !     uframe trans z = -100            -> TCP must drop exactly 100 mm
    !     uframe identity, oframe z = -100 -> TCP must land in the SAME
    !                                         place as the uframe version
    !   PASS: both printed steps = 100.0 mm +/- 0.05, and the two moved
    !         positions agree to within the settle noise. That agreement is
    !         what licenses E28's "write oframe instead".
    !
    ! THE BARRIER. Every write below sits behind a `fine` point, on purpose.
    ! A tooldata/wobjdata assignment is executed by the PROGRAM task while
    ! the motion planner has already prepared the queued instructions
    ! against the OLD record - so a write dropped between two zone-joined
    ! moves lands late, and the move that was supposed to use the new tool
    ! used the old one. Three ways to stop that, in order of preference:
    !   1. land on a `fine` point before the write   <- used here, and the
    !      exporter rule "a generated .tgs must end on a fine point"
    !      (robotstudio_setup.md sec 9) already gives it to us for free
    !   2. StopMove; <write>; StartMove;             <- shown commented
    !      below; decelerates the current path to a halt instead of waiting
    !      for a programmed stop point
    !   3. WaitRob \InPos;                           <- weakest: it shares
    !      its convergence criteria with the fine-point release, so it is a
    !      settle aid, not a barrier (measured, findings doc 2026-08-28)
    ! Changing the tool mid-path is not something to make work: end the
    ! path, write, then start the next one.
    !
    ! Configuration supervision is relaxed for the demo only, same as
    ! TGArcCheck step 2 and TD05Test: the stored confdata is a dummy. A
    ! production program keeps ConfJ/ConfL ON.
    !***********************************************************************
    PROC TGToolFrameMoveCheck()
        VAR pos pTcpBefore;
        VAR pos pTcpAfter;
        VAR pos pFlgBefore;
        VAR pos pFlgAfter;
        VAR pos pRefB;
        VAR pos pUfrm;
        VAR pos pOfrm;

        TPWrite "TG TFSET: ---- step 4: does the write take effect? ----";
        TGToolFrameReset;
        ConfJ\Off;
        ConfL\Off;
        MoveAbsJ jtSafe,v100,fine,tTGFS_Tool;

        ! =================== 4A: TCP change ===================
        MoveJ rtProbe,v200,fine,tTGFS_Tool\WObj:=wobj0;
        tgfsSettle;
        pTcpBefore:=CPos(\Tool:=tTGFS_Tool \WObj:=wobj0);
        pFlgBefore:=CPos(\Tool:=tTGFS_Flange \WObj:=wobj0);
        TPWrite "  4A TCP    before ="\Pos:=pTcpBefore;
        TPWrite "  4A flange before ="\Pos:=pFlgBefore;

        ! THE WRITE. Standing on the `fine` point of the MoveJ above, so
        ! nothing is queued and nothing can still pick up the old value.
        ! The StopMove variant, for a write that cannot wait for a stop
        ! point:
        !   StopMove;
        !   tTGFS_Tool.tframe := PoseMult(tTGFS_Tool.tframe,[[0,0,10],[1,0,0,0]]);
        !   StartMove;
        tTGFS_Tool.tframe:=PoseMult(tTGFS_Tool.tframe,[[0,0,10],[1,0,0,0]]);
        TPWrite "  4A new TCP offset ="\Pos:=tTGFS_Tool.tframe.trans;

        ! Same target, same instruction, new tool.
        MoveJ rtProbe,v200,fine,tTGFS_Tool\WObj:=wobj0;
        tgfsSettle;
        pTcpAfter:=CPos(\Tool:=tTGFS_Tool \WObj:=wobj0);
        pFlgAfter:=CPos(\Tool:=tTGFS_Flange \WObj:=wobj0);
        TPWrite "  4A TCP    after  ="\Pos:=pTcpAfter;
        TPWrite "  4A flange after  ="\Pos:=pFlgAfter;
        TPWrite "  4A TCP    step, mm ="\Num:=Distance(pTcpBefore,pTcpAfter);
        TPWrite "  4A flange step, mm ="\Num:=Distance(pFlgBefore,pFlgAfter);
        TPWrite "  4A expect TCP ~0, flange 10.0";

        ! =================== 4B: frame change ===================
        TGToolFrameReset;
        MoveJ rtProbe,v200,fine,tTGFS_Tool\WObj:=wobjTGFS;
        tgfsSettle;
        pRefB:=CPos(\Tool:=tTGFS_Tool \WObj:=wobj0);
        TPWrite "  4B TCP, identity frame ="\Pos:=pRefB;

        ! b1: the correction in uframe (the static / TG_ReqWeldFrame way)
        wobjTGFS.uframe.trans:=[0,0,-100];
        MoveJ rtProbe,v200,fine,tTGFS_Tool\WObj:=wobjTGFS;
        tgfsSettle;
        pUfrm:=CPos(\Tool:=tTGFS_Tool \WObj:=wobj0);
        TPWrite "  4B TCP, uframe z-100  ="\Pos:=pUfrm;
        TPWrite "  4B uframe step, mm ="\Num:=Distance(pRefB,pUfrm);

        ! b2: the same correction in oframe (the coordinated-safe way, E28)
        wobjTGFS.uframe:=wobjTGFS_Decl.uframe;
        wobjTGFS.oframe.trans:=[0,0,-100];
        MoveJ rtProbe,v200,fine,tTGFS_Tool\WObj:=wobjTGFS;
        tgfsSettle;
        pOfrm:=CPos(\Tool:=tTGFS_Tool \WObj:=wobj0);
        TPWrite "  4B TCP, oframe z-100  ="\Pos:=pOfrm;
        TPWrite "  4B oframe step, mm ="\Num:=Distance(pRefB,pOfrm);
        TPWrite "  4B uframe vs oframe, mm ="\Num:=Distance(pUfrm,pOfrm);
        TPWrite "  4B expect both steps 100.0 and the last line ~0";

        TGToolFrameReset;
        MoveAbsJ jtSafe,v100,fine,tTGFS_Tool;
        ConfJ\On;
        ConfL\On;
        TPWrite "TG TFSET: step 4 done - judge by the printed millimetres";
    ENDPROC

    !***********************************************************************
    ! STEP 5 - probes for the claims this sample makes but does not prove.
    ! Each is a PAIR of lines: uncomment, run a program check, run the
    ! routine. As in TGArcCheck step 3, a failure IS the answer for that
    ! probe - record it and re-comment.
    !
    ! 5A runs unconditionally (a harmless read). 5B and 5C are EXPECTED to
    ! fault, which is the point, so they stay commented.
    !***********************************************************************
    PROC TGToolFrameProbe()
        TPWrite "TG TFSET: ---- step 5: probes ----";
        TGToolFrameReset;

        ! --- 5A: what does tool0 actually carry on this controller? ---
        ! The docs say ABB rejects mass = 0 for a held tool, yet tool0 is
        ! held and weighs nothing. Reading it settles what the "empty" load
        ! literal really is - and gives tTGFS_Flange above a value that is
        ! known-good rather than guessed.
        TPWrite "  5A tool0 robhold ="\Bool:=tool0.robhold;
        TPWrite "  5A tool0 mass    ="\Num:=tool0.tload.mass;
        TPWrite "  5A tool0 cog     ="\Pos:=tool0.tload.cog;

        ! --- 5B: is mass 0 rejected, and WHEN? ---
        ! Expected: the assignment is accepted (it is just a record write)
        ! and the next motion instruction faults, because the load is
        ! validated at use time. If the move runs instead, the exporter's
        ! "never declare mass 0" rule is a robustness rule rather than a
        ! hard controller constraint - worth knowing which one it is.
        ! tTGFS_Tool.tload.mass:=0;
        ! TPWrite "  5B assignment accepted, mass ="\Num:=tTGFS_Tool.tload.mass;
        ! MoveAbsJ jtSafe,v100,fine,tTGFS_Tool;
        ! TPWrite "  5B MOVE ALSO ACCEPTED - mass 0 is not enforced here";

        ! --- 5C: is a VAR tooldata rejected as a \PERS argument? ---
        ! Expected: this does not COMPILE - "Argument error(123): Argument
        ! for 'PERS' parameter Tool is not a persistent reference", i.e.
        ! finding F-3 reproduced from the tool side. That compile failure is
        ! rule R1's proof, and it is why an on-the-fly tool must be PERS.
        ! Being a module-wide error it also breaks steps 1-4, so uncomment
        ! it on its own. Note the declaration has to move to the top of the
        ! routine with the others when it is uncommented.
        ! VAR tooldata tLocal:=[TRUE,[[0,0,10],[1,0,0,0]],[1,[0,0,5],[1,0,0,0],0,0,0]];
        ! TPWrite "  5C CPos with a VAR tool ="\Pos:=CPos(\Tool:=tLocal \WObj:=wobj0);

        TPWrite "TG TFSET: step 5 done";
    ENDPROC

    !***********************************************************************
    ! The settle ladder, one copy, so step 4's millimetres mean something.
    ! Same three stages and same 0.2 s as tgSendPose in TG_Comms.sys; not
    ! shared with it because this module is standalone by design.
    ! Measured (findings doc 2026-08-28): no wait 1.3 mm error, \InPos alone
    ! 0.28 mm, full ladder 0.
    !***********************************************************************
    LOCAL PROC tgfsSettle()
        WaitRob \InPos;
        WaitRob \ZeroSpeed;
        IF nTGFS_Settle>0 WaitTime nTGFS_Settle;
    ENDPROC

ENDMODULE
