MODULE TD05Test_Mod
    !***********************************************************************
    ! Sample .tgs welding program for the ABB prototype (Phase 2/3).
    !
    ! Stands in for a Weld-Planner-generated program. Mirrors the request
    ! call order of the FANUC sample TD05tRJYQd.ls (one capture set with two
    ! captures + one weld; touch-sense family omitted - out of v1 scope).
    !
    ! Naming contract (plan 4.1): file name = PROC name = the program name
    ! the HMI sends ("TD05Test"); TG_Main loads "HOME:/TGS/"+name+".mod" and
    ! late-binds %name%. The MODULE name carries a "_Mod" suffix because
    ! RAPID module names and global routine names share one namespace - a
    ! module named like its own PROC is a semantic error ("name ambiguous").
    ! It calls the TG_* request PROCs and reads the PERS data from
    ! TG_Comms.sys directly - no include mechanism needed (task-wide scope).
    !
    ! Motion is MoveAbsJ between safe joint poses so it runs on any virtual
    ! controller. Real capture/weld moves in the received frames are shown
    ! as comments where they belong - except the weld-frame demonstration
    ! (search "demonstration"), which does move in wobjTG_Weld on purpose:
    ! the same target before and after TG_ReqWeldFrame, to show the received
    ! frame taking effect.
    !***********************************************************************

    ! Safe joint poses (VC demo only - a real .tgs program carries its own targets)
    LOCAL CONST jointtarget jtHome:=[[0,0,0,0,30,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST jointtarget jtCap1:=[[15,10,-10,0,40,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST jointtarget jtCap2:=[[-15,10,-10,0,40,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    ! Weld-frame demonstration target. Coordinates are relative to
    ! wobjTG_Weld, so the SAME target resolves to a different Cartesian
    ! position once TG_ReqWeldFrame writes the frame received from the HMI.
    ! Orientation [0,0,1,0] = tool pointing along -z of the work object.
    LOCAL CONST robtarget rtWeldDemo:=[[1000,0,600],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    PROC TD05Test()
        ! --- program header (FANUC lines 1-8) ---
        stTG_ProgPass:="TD05Test";     ! SET_PASS_SR: program name == password
        stTG_RobStatus:="Ok";          ! SET_ROB_S_SR('Ok')
        stTG_SubName:="none";          ! deterministic start (SR[25] was stale on FANUC)
        nTG_ActTool:=8;                ! UTOOL_NUM=8 (torch)
        nTG_ActFrame:=0;               ! UFRAME[9] home frame -> base for the demo
        MoveAbsJ jtHome,v100,fine,tTG_Weld;

        ! --- password / dry-run check (FANUC lines 9-16) ---
        TG_ReqPassCheck;
        IF nTG_PassOK=0 THEN
            ! FANUC: 'END' - terminate immediately, NO end request (R_E).
            RETURN;
        ENDIF
        ! nTG_DryRun=1 would disable welding (FANUC DRY_RUN_ON/OFF macros) -
        ! weld enable signals are cell-specific, deferred to phase 4.

        ! --- capture set (FANUC lines 27-62: camera tool UT[2], frame UFRAME[5]) ---
        nTG_ActTool:=2;                ! UTOOL_NUM=2 (camera)
        nTG_ActFrame:=5;               ! UFRAME_NUM=5 - resolved LIVE by tgSendPose,
                                       ! so the frame received by TG_ReqCamFrame is
                                       ! used from the very next pose report
                                       ! (FANUC needed UFRAME[5]=PR[5] re-emits)
        MoveAbsJ jtCap1,v100,fine,tTG_Cam;
        stTG_SubName:="C1PGlobal_m45_3";
        TG_ReqCamFrame;                ! R_C_F -> wobjTG_Cam.uframe + nTG_DoCapture
        IF nTG_DoCapture=1 THEN
            ! Real program: CAM_OPEN, then move to the capture point in the
            ! just-received frame:
            !   MoveJ pCap1,v100,fine,tTG_Cam\WObj:=wobjTG_Cam;
            WaitTime 0.2;              ! FANUC: WAIT 0.20(sec)
            TG_ReqCapture;             ! R_C -> nTG_CaptureOK
            IF nTG_CaptureOK=0 GOTO abort_end;
        ENDIF
        MoveAbsJ jtCap2,v100,fine,tTG_Cam;
        stTG_SubName:="C2PGlobal_m45_3";
        TG_ReqCamFrame;
        IF nTG_DoCapture=1 THEN
            WaitTime 0.2;
            TG_ReqCapture;
            IF nTG_CaptureOK=0 GOTO abort_end;
        ENDIF

        ! --- end of captures: global localization (FANUC lines 61-63) ---
        TG_ReqGlobalCapDone;           ! R_G_C_D -> nTG_GlobalCapOK
        ! FANUC: CAM_CLOSE / WELD_PREP - cell hardware macros, phase 4.

        ! --- weld (FANUC lines 197-226: torch UT[8], frame UFRAME[6]) ---
        nTG_ActTool:=8;
        nTG_ActFrame:=6;
        stTG_SubName:="PWeld2";

        ! ---------------- weld-frame demonstration (demo only) ----------------
        ! Same target, moved to BEFORE and AFTER the frame request. Because
        ! rtWeldDemo is expressed in wobjTG_Weld, the robot ends up in two
        ! different Cartesian positions - visible proof that TG_ReqWeldFrame
        ! updated the frame the welding motions run in.
        !
        ! Reset to identity first, so the "before" position is the same on
        ! every cycle (wobjTG_Weld otherwise persists from the previous run,
        ! exactly like FANUC UFRAME[6] does). Demo-only: a production .tgs
        ! program must NOT clear a received frame.
        wobjTG_Weld.uframe:=[[0,0,0],[1,0,0,0]];
        ! The same target in two different frames needs two different arm
        ! configurations, so the stored confdata cannot satisfy both.
        ! Demo-only: a production program keeps configuration control on.
        ConfJ\Off;
        ConfL\Off;
        MoveJ rtWeldDemo,v200,fine,tTG_Weld\WObj:=wobjTG_Weld;
        TPWrite "TG DEMO: before R_W_F, TCP ="\Pos:=CPos(\Tool:=tTG_Weld \WObj:=wobj0);
        ! ----------------------------------------------------------------------

        TG_ReqWeldFrame;               ! R_W_F -> wobjTG_Weld.uframe + nTG_WeldStatus
        IF nTG_WeldStatus=2 GOTO abort_end;
        IF nTG_WeldStatus=1 THEN
            ! ------------- weld-frame demonstration, part 2 -------------
            ! Identical instruction, identical target - new frame.
            MoveJ rtWeldDemo,v200,fine,tTG_Weld\WObj:=wobjTG_Weld;
            TPWrite "TG DEMO: after  R_W_F, TCP ="\Pos:=CPos(\Tool:=tTG_Weld \WObj:=wobj0);
            ! -----------------------------------------------------------
            ! Approach in the received weld frame:
            !   MoveJ pApproach,v100,z100,tTG_Weld\WObj:=wobjTG_Weld;
            !   MoveL pWeldStart,v50,fine,tTG_Weld\WObj:=wobjTG_Weld;
            TG_ReqWeldParams;          ! R_W_P -> nTG_UdwpFlag, nTG_TravelSpeed, ...
            ! RobotWare Arc weld goes here (plan, phase 4): ArcLStart/ArcL/
            ! ArcLEnd with welddata built from nTG_WeldProc/nTG_WireFeed/
            ! nTG_ArcLength/nTG_ArcControl, travel speed nTG_TravelSpeed
            ! (FANUC: L P[124] R[175]inch/min + WELD START/END[R[171],20]).
            ! FANUC calls R_W_S (welding stats) after WELD END - out of v1.
        ENDIF

        ! --- return home (FANUC lines 397-411) ---
        MoveAbsJ jtHome,v100,fine,tTG_Weld;

abort_end:
        ! FANUC LBL[101]: both the normal exit and the abort target.
        ConfJ\On;                      ! undo the demo's relaxed config control
        ConfL\On;
        stTG_SubName:="none";
        TG_ReqEnd;                     ! R_E
    ENDPROC

ENDMODULE
