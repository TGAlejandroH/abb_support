MODULE TD05Weld_Mod
    !***********************************************************************
    ! Sample .tgs welding program - TWO REAL WELDS with arc motion.
    !
    ! Stands in for a Weld-Planner-generated program and is the executable
    ! spec of what the planner AbbProgramBlocksProvider must emit for the
    ! HMI-mode ABB weld section. Mirrors welds Weld2 and Weld3 of the FANUC
    ! sample TD05tRJYQd.ls line for line (touch-sense omitted - out of
    ! scope), with the capture set trimmed to keep the run short.
    !
    ! WHY A SEPARATE FILE FROM TD05Test.mod: this program needs option
    ! 633-4 Arc (it references TG_Weld.sys). TD05Test.mod stays the phase
    ! 1-3 comms regression program that runs on a system WITHOUT Arc.
    !
    ! Naming contract (plan 4.1): file name = PROC name = the program name
    ! the HMI sends ("TD05Weld"); the MODULE carries a _Mod suffix because
    ! RAPID module names and global routine names share one namespace.
    !
    ! FANUC weld anatomy being reproduced, per weld:
    !   CALL SET_SUB_ROUTINE_SR('PWeldN')  -> stTG_SubName := "PWeldN"
    !   CALL R_W_F                          -> TG_ReqWeldFrame
    !   IF R[198]=2 JMP LBL[101]            -> IF nTG_WeldStatus=2 GOTO abort_end
    !   IF (R[198]=1) THEN                  -> IF nTG_WeldStatus=1 THEN
    !     J/L approach moves                -> MoveJ / MoveL
    !     CALL R_W_P                        -> TG_ReqWeldParams
    !     WELD START[...]                   -+
    !     L P[n] R[175]inch/min FINE         +-> TG_ApplyWeldParams + ArcLStart/ArcLEnd
    !     WELD END[...]                     -+
    !     CALL R_W_S                        -> TG_ReqWeldStats (dummy values)
    !     L depart                          -> MoveL
    !   ENDIF
    !***********************************************************************

    ! Procedure numbers the Weld Planner emitted for these welds - the
    ! analogue of the <proc> literal in FANUC WELD START[<proc>,<sched>].
    ! Used only when the HMI reports UDWP=0 (then it sends no proc number,
    ! exactly as KAREL zeroes R[171..174]).
    LOCAL CONST num nProcWeld2:=1;
    LOCAL CONST num nProcWeld3:=2;

    ! Safe joint poses (VC demo only - a real .tgs carries its own targets).
    LOCAL CONST jointtarget jtHome:=[[0,0,0,0,30,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    ! Weld targets, expressed in wobjTG_Weld so they follow the frame the
    ! HMI serves in R_W_F. Each weld is a 200 mm straight seam.
    ! Orientation [0,0,1,0] = torch pointing along -z of the work object.
    LOCAL CONST robtarget rtW2Approach:=[[900,-200,300],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtW2Near:=[[900,-120,200],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtW2Start:=[[900,-100,200],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtW2End:=[[900,100,200],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtW2Depart:=[[900,200,300],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    LOCAL CONST robtarget rtW3Approach:=[[1100,-200,300],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtW3Near:=[[1100,-120,200],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtW3Start:=[[1100,-100,200],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtW3End:=[[1100,100,200],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtW3Depart:=[[1100,200,300],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    PROC TD05Weld()
        ! --- program header (FANUC lines 1-19) ---
        stTG_ProgPass:="TD05Weld";
        stTG_RobStatus:="Ok";
        stTG_SubName:="none";

        ! --- part mounting (weld_frame_update_strategy_v1) ---------------
        ! The exported program STATES how the part is mounted, once, before
        ! any motion. The exporter knows this - the request PROCs must not
        ! have to guess, and a program that inherited a previous run's
        ! mounting would weld against the wrong thing.
        !   uframe = THE MOUNT, oframe = THE PART ON THE MOUNT.
        ! Assigning the whole record (not just a component) is what makes
        ! the statement complete. Emitted here, ahead of motion, which also
        ! keeps it clear of RAPID's look-ahead (contract O-3).
        !
        ! This demo welds on a STATIC table, so the mount is identity and
        ! the oframe below is the nominal part frame the robtargets were
        ! divided by; TG_ReqWeldFrame overwrites just that component with
        ! the measured one. A part on the chuck/turntable instead reads
        !   wobjTG_Weld:=[FALSE,FALSE,"STN1",
        !                 [[0,0,0],[1,0,0,0]],       ! ufprog FALSE -> ignored
        !                 [[<part on the plate>]]];  ! <- vision writes here
        ! and nothing else in this program changes: the robtargets stay
        ! divided by the same part frame either way, and the request call
        ! below is identical. Indexed and coordinated welds are the same
        ! case here - whether the station MOVES during the weld does not
        ! change what the part is bolted to.
        wobjTG_Weld:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];

        MoveAbsJ jtHome,v100,fine,tTG_Weld;

        TG_ReqPassCheck \Tool:=tTG_Weld \WObj:=wobj0;
        IF nTG_PassOK=0 THEN
            ! FANUC 'END' - terminate immediately, NO end request.
            RETURN;
        ENDIF
        TG_DryRunOff;
        IF nTG_DryRun=1 TG_DryRunOn;
        TG_WeldPrep;

        ! Demo shortcut: the FANUC sample runs a capture set here. Captures
        ! are already covered by TD05Test.mod, so this program reports
        ! global-captures-done and goes straight to the welds.
        TG_ReqGlobalCapDone;
        TG_CamClose;
        TG_WeldPrep;

        ! Demo only: the two welds are taught in wobjTG_Weld and the same
        ! stored confdata cannot satisfy every frame the HMI might serve.
        ! A production program keeps configuration control ON.
        ConfJ\Off;
        ConfL\Off;

        ! ================== WELD 2 (FANUC lines 197-226) ==================
        stTG_SubName:="PWeld2";
        TG_ReqWeldFrame \Tool:=tTG_Weld \WObj:=wobjTG_Weld;
        TG_CamClose;
        IF nTG_WeldStatus=2 GOTO abort_end;
        IF nTG_WeldStatus=1 THEN
            MoveJ rtW2Approach,v200,z10,tTG_Weld\WObj:=wobjTG_Weld;
            MoveL rtW2Near,v200,z10,tTG_Weld\WObj:=wobjTG_Weld;

            TG_ReqWeldParams;
            TG_ApplyWeldParams nProcWeld2;

            ! ArcLStart positions at the seam start and ignites; ArcLEnd
            ! runs the pass and closes the seam. The v200 governs the
            ! positioning move only - the weld itself runs at
            ! wdTG_Weld.weld_speed (VC-measured, 0.07 % match).
            ArcLStart rtW2Start,v200,sdTG_Weld,wdTG_Weld,z10,tTG_Weld\WObj:=wobjTG_Weld;
            ArcLEnd rtW2End,v200,sdTG_Weld,wdTG_Weld,fine,tTG_Weld\WObj:=wobjTG_Weld;

            ! R_W_S, exactly where FANUC calls it: after WELD END, before
            ! the depart move. DUMMY stats (see TG_Comms.sys) chosen to be
            ! consistent with what this weld actually just did, so a VC
            ! transcript stays sane to read: the seam is 200 mm (rtW2Start
            ! -> rtW2End) and this, the run's FIRST weld, is served 21 IPM
            ! = 8.89 mm/s, so 200 / 8.89 = 22.5 s of arc-on time.
            nTG_WeldDist:=200;
            nTG_ArcOnTime:=22.5;
            nTG_SuccArcEnd:=1-nTG_DryRun;
            TG_ReqWeldStats;
            MoveL rtW2Depart,v200,fine,tTG_Weld\WObj:=wobjTG_Weld;
        ENDIF

        ! ================== WELD 3 (FANUC lines 366-395) ==================
        stTG_SubName:="PWeld3";
        TG_ReqWeldFrame \Tool:=tTG_Weld \WObj:=wobjTG_Weld;
        TG_CamClose;
        IF nTG_WeldStatus=2 GOTO abort_end;
        IF nTG_WeldStatus=1 THEN
            MoveJ rtW3Approach,v200,z10,tTG_Weld\WObj:=wobjTG_Weld;
            MoveL rtW3Near,v200,z10,tTG_Weld\WObj:=wobjTG_Weld;

            TG_ReqWeldParams;
            TG_ApplyWeldParams nProcWeld3;

            ArcLStart rtW3Start,v200,sdTG_Weld,wdTG_Weld,z10,tTG_Weld\WObj:=wobjTG_Weld;
            ArcLEnd rtW3End,v200,sdTG_Weld,wdTG_Weld,fine,tTG_Weld\WObj:=wobjTG_Weld;

            ! Same seam length, but the run's SECOND weld is served 30 IPM
            ! = 12.7 mm/s, so 200 / 12.7 = 15.75 s. DIFFERENT numbers from
            ! the weld above on purpose: the transcript then proves the HMI
            ! got two independent servings rather than one repeated payload.
            nTG_WeldDist:=200;
            nTG_ArcOnTime:=15.75;
            nTG_SuccArcEnd:=1-nTG_DryRun;
            TG_ReqWeldStats;
            MoveL rtW3Depart,v200,fine,tTG_Weld\WObj:=wobjTG_Weld;
        ENDIF

        ! --- return home (FANUC lines 397-411) ---
        TG_CamClose;
        MoveAbsJ jtHome,v100,fine,tTG_Weld;

abort_end:
        ! FANUC LBL[101]: both the normal exit and the abort target.
        ConfJ\On;
        ConfL\On;
        stTG_SubName:="none";
        TG_ReqEnd \Tool:=tTG_Weld \WObj:=wobjTG_Weld;
    ENDPROC

ENDMODULE
