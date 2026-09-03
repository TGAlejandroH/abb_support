MODULE TGArcCheck_Mod
    !***********************************************************************
    ! TGArcCheck - standalone RobotWare Arc readiness check for the VC.
    !
    ! v2 (2026-08-31): after the first VC run, two gaps were closed.
    !   - step 2 now MEASURES the weld segment with a clock, against a
    !     reference move of identical geometry. The v1 log proved only that
    !     ArcLStart/ArcLEnd executed, not that welddata.weld_speed governed
    !     the weld - which is the load-bearing assumption of the design.
    !   - step 3 now READS BACK each probed component, so the log shows
    !     which ones exist. In v1 the message printed either way, so an
    !     uncommented run and a commented run looked identical.
    !
    ! Deliberately standalone: no dependency on TG_Comms.sys / TG_Cell.sys /
    ! TG_Main.mod, and it uses tool0 / wobj0 so no calibrated torch TCP or
    ! work object is needed. Load it alone and run the PROCs by hand.
    !
    ! REQUIRES option 633-4 Arc. Run instructions and pass criteria:
    ! docs/robotstudio_setup.md section 14.
    !
    ! CONFIRMED on this controller by the v1 run (2026-08-31 07:10): both
    ! declarations below compile and read back, so these shapes are correct
    ! for this configuration, not merely descriptor-derived:
    !   welddata := [ weld_speed, org_weld_speed, main_arc, org_arc ]
    !   arcdata  := [ sched, mode, voltage, wirefeed, control, current,
    !                 voltage2, wirefeed2, control2 ]
    !   seamdata := [ purge_time, preflow_time, ign_arc, ign_move_delay,
    !                 scrape_start, heat_speed, heat_time, heat_distance,
    !                 heat_arc, cool_time, fill_time, fill_arc, bback_time,
    !                 rback_time, bback_arc, postflow_time ]
    !***********************************************************************

    ! ------------------------- the weld data -------------------------
    ! PERS is MANDATORY, not a style choice: the Seam and Weld arguments of
    ! ArcLStart are accessMode="Persistent" (verified in the controller
    ! MoveInstructionDescriptions\ArcLStart.xml), so a VAR or an expression
    ! raises "Argument error: not a persistent reference" - the same rule
    ! that produced finding F-3 for the \Tool/\WObj of CRobT.
    PERS seamdata sdArcChk:=[0.2,0.05,[0,0,0,0,0,0,0,0,0],0,0,0,0,0,[0,0,0,0,0,0,0,0,0],0,0,[0,0,0,0,0,0,0,0,0],0.1,0,[0,0,0,0,0,0,0,0,0],0.05];
    PERS welddata wdArcChk:=[10,10,[0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0]];

    ! Stopwatch for the step-2 measurement.
    LOCAL VAR clock clkChk;

    ! --------------------- safe demo targets ---------------------
    ! IRB 4600-20/2.50, tool0 in wobj0 (base frame). Orientation [0,0,1,0]
    ! = 180 deg about Y, i.e. tool z pointing straight down. The measured
    ! line is EXACTLY 300.0 mm: y goes -150 -> +150 at fixed x=1200, z=800.
    LOCAL CONST jointtarget jtSafe:=[[0,0,0,0,30,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtApproach:=[[1200,-250,900],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtWeldStart:=[[1200,-150,800],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtWeldEnd:=[[1200,150,800],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtDepart:=[[1200,250,900],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    !***********************************************************************
    ! STEP 1 - data only. No motion, no welding. Safe to run anywhere.
    ! PASSED on this controller 2026-08-31 07:10 (all six values correct).
    !***********************************************************************
    PROC TGArcCheck()
        TPWrite "TG ARC CHECK: ---- step 1: weld data ----";

        ! Travel speed. NOTE the unit is whatever ARC_UNITS the arc system
        ! is configured for - NOT necessarily mm/s. This VC currently runs
        ! units = SI_UNITS (arc_velocity_units = mm_s), so 8.89 = 21 IPM
        ! converted. Under US_UNITS it would be ipm and we would write 21
        ! directly. Step 2 measures which is actually in force.
        wdArcChk.weld_speed:=8.89;

        ! Wire feed. Same story: SI_UNITS means arc_feed_units = mm_s,
        ! US_UNITS means ipm, WELD_UNITS means m_min.
        wdArcChk.main_arc.wirefeed:=520;

        ! Arc length. ABB names the component "voltage", but on a Fronius it
        ! is wired to VoltReference -> aoFr1ArcLength, i.e. arc length
        ! correction. Map by intent, not by name.
        ! 4.9 not 49.0: Fronius corrections run about +/-10 steps, so
        ! TG_ApplyWeldParams must clamp. This value is deliberately in range.
        wdArcChk.main_arc.voltage:=4.9;

        TPWrite "TG ARC CHECK: welddata assignments accepted";
        TPWrite "  weld_speed        ="\Num:=wdArcChk.weld_speed;
        TPWrite "  main_arc.wirefeed ="\Num:=wdArcChk.main_arc.wirefeed;
        TPWrite "  main_arc.voltage  ="\Num:=wdArcChk.main_arc.voltage;
        TPWrite "  seam purge_time   ="\Num:=sdArcChk.purge_time;
        TPWrite "  seam postflow_time="\Num:=sdArcChk.postflow_time;

        TPWrite "TG ARC CHECK: step 1 PASSED";
    ENDPROC

    !***********************************************************************
    ! STEP 2 - weld motion, MEASURED.
    !
    ! Two timed runs over the SAME 300.0 mm straight line:
    !   A) REFERENCE: plain MoveL at v100. Expect ~3.0 s (300/100).
    !      This validates the stopwatch method itself. If it reads ~0,
    !      RAPID lookahead defeated the timing and the weld number below
    !      cannot be trusted - say so rather than interpreting it.
    !   B) WELD: ArcLStart + ArcLEnd, same geometry, v200 as the speed
    !      argument, weld_speed = 8.89 from step 1.
    !
    ! What B tells us (300 mm, weld_speed 8.89):
    !   ~34 s  -> welddata.weld_speed governs AND is mm/s  (SI_UNITS)
    !   ~80 s  -> weld_speed governs and is being read as IPM (US_UNITS)
    !   ~1.5 s -> the v200 ARGUMENT governs, weld_speed does NOT
    ! The third outcome would invalidate the core assumption of the weld
    ! design and must be reported, not worked around.
    !
    ! Configuration supervision is relaxed for the demo only (same reason
    ! TD05Test.mod does it): these targets are reached with whatever arm
    ! configuration the solver picks and the stored confdata is a dummy.
    ! A production program keeps ConfJ/ConfL ON.
    !***********************************************************************
    PROC TGArcMoveCheck()
        TPWrite "TG ARC CHECK: ---- step 2: weld motion (measured) ----";
        ConfJ\Off;
        ConfL\Off;
        MoveAbsJ jtSafe,v100,fine,tool0;

        ! ---- A) reference: same 300 mm line, plain MoveL at v100 ----
        MoveJ rtWeldStart,v200,fine,tool0\WObj:=wobj0;
        ClkReset clkChk;
        ClkStart clkChk;
        MoveL rtWeldEnd,v100,fine,tool0\WObj:=wobj0;
        ClkStop clkChk;
        TPWrite "  A REF  MoveL 300mm v100, sec ="\Num:=ClkRead(clkChk);

        ! ---- B) the weld: identical geometry, speed from welddata ----
        ! Argument order per ArcLStart.xml: ToPoint, Speed, Seam, Weld,
        ! Zone, Tool, \WObj. ArcLStart moves to the weld start and ignites
        ! there; ArcLEnd runs the pass and closes the seam. Together they
        ! replace FANUC L P[123] + WELD START + L P[124] R[175]inch/min +
        ! WELD END.
        MoveJ rtApproach,v200,fine,tool0\WObj:=wobj0;
        ClkReset clkChk;
        ClkStart clkChk;
        ArcLStart rtWeldStart,v200,sdArcChk,wdArcChk,z10,tool0\WObj:=wobj0;
        ArcLEnd rtWeldEnd,v200,sdArcChk,wdArcChk,fine,tool0\WObj:=wobj0;
        ClkStop clkChk;
        TPWrite "  B WELD 300mm ArcL,     sec ="\Num:=ClkRead(clkChk);
        TPWrite "    (B includes a 141mm v200 approach, about 0.7 s)";

        MoveL rtDepart,v200,fine,tool0\WObj:=wobj0;
        MoveAbsJ jtSafe,v100,fine,tool0;

        ConfJ\On;
        ConfL\On;
        TPWrite "TG ARC CHECK: step 2 PASSED - compare A and B above";
    ENDPROC

    !***********************************************************************
    ! STEP 3 - optional component probe, WITH READ-BACK.
    !
    ! Each probe is a PAIR of lines: the assignment and its read-back.
    ! Uncomment a pair, run a program check, then run the routine. If the
    ! component does not exist the module fails to COMPILE (a module-wide
    ! error that would also break steps 1 and 2) - that failure IS the
    ! answer for that component; re-comment it and move on.
    !
    ! Why the read-back matters: in v1 the routine printed the same message
    ! whether or not the probes were uncommented, so the log could not tell
    ! us anything. Values are deliberately distinctive (1.5 / 7 / 2) so they
    ! cannot be confused with the step-1 values.
    !
    ! What each buys:
    !   control -> Fronius ControlPort (aoFr1Dynamic) = the Arc Control
    !              field of the HMI, which FANUC never applied at all.
    !   sched   -> Fronius JobPort. Needed only for job-mode operation.
    !   mode    -> Fronius ModePort. Needed only if a mode is driven.
    !   org_*   -> the "original" values behind FlexPendant tune-reset.
    !              PROBE ONLY - production code must never write these.
    !***********************************************************************
    PROC TGArcProbeOptional()
        TPWrite "TG ARC CHECK: ---- step 3: optional probes ----";

        ! --- probe A: main_arc.control (HMI Arc Control) ---
        ! wdArcChk.main_arc.control:=1.5;
        ! TPWrite "  A main_arc.control ="\Num:=wdArcChk.main_arc.control;

        ! --- probe B: main_arc.sched (Fronius job number) ---
        ! wdArcChk.main_arc.sched:=7;
        ! TPWrite "  B main_arc.sched   ="\Num:=wdArcChk.main_arc.sched;

        ! --- probe C: main_arc.mode (Fronius mode port) ---
        ! wdArcChk.main_arc.mode:=2;
        ! TPWrite "  C main_arc.mode    ="\Num:=wdArcChk.main_arc.mode;

        ! --- probe D: org_weld_speed (read only, do NOT write in prod) ---
        ! TPWrite "  D org_weld_speed   ="\Num:=wdArcChk.org_weld_speed;

        TPWrite "TG ARC CHECK: step 3 done - any A/B/C/D line above exists";
    ENDPROC

ENDMODULE
