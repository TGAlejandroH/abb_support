MODULE TD05Touch_Mod
    !***********************************************************************
    ! Sample .tgs program - auto touch-ups with REAL searches (touch-sense P3).
    !
    ! The executable spec of what the Weld Planner's ABB exporter must emit
    ! for a touch-sensed weld (docs/abb_touch_sense_port_v1.md 3.5, 5.2): the
    ! TSPWeld2_full block of the FANUC sample TD05tRJYQd.ls (lines 76-196)
    ! line for line, then the weld (a dry pass here - TD05Weld.mod covers the
    ! arc instructions). Per touch, exactly as FANUC with "auto return" off:
    !     approach (fine) -> TG_TouchSearch -> TG_ReqTouchSensePoint ->
    !     WaitTime 0.5 -> MoveL back to the approach = THE return (D12)
    !
    ! The part is a 100 x 100 x 200 mm block in wobjTG_Weld (the frame the
    ! HMI serves): x 0..100, y -50..50, z -200..0. Touch 1 searches its -X
    ! face (x = 0) in +X, touch 2 its top face (z = 0) in -Z, each from 30 mm
    ! off along the snapped axis. Mirrors hmi_prototype/abb_server.py
    ! TOUCH_REAL_DEMO; on the VC the block is the World Zone box of
    ! hmi_prototype/vc_probes/TG_TrRig.mod. The three MUST agree -
    ! test_phase8_touchsense.py checks them against each other.
    !
    ! Orientation: the robot's home tool orientation [0.5,0,0.866025,0] in
    ! world, expressed in the served frame (turned 90 deg about Z). ConfJ/ConfL
    ! off, as the other demos: one stored confdata cannot fit every frame the
    ! HMI may serve. A production program keeps configuration control on.
    !
    ! Naming contract (plan 4.1): file = PROC = program name ("TD05Touch");
    ! MODULE carries "_Mod" (F-1); all data LOCAL (F-5).
    !***********************************************************************

    ! FANUC's 775 mm/sec approach speed.
    LOCAL CONST speeddata vTsApproach:=[775,500,5000,1000];

    LOCAL CONST robtarget rtTsVia:=[[-30,0,40],[0.353554,0.612372,0.612372,-0.353554],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtTs1Approach:=[[-30,0,-20],[0.353554,0.612372,0.612372,-0.353554],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtTs1Contact:=[[0,0,-20],[0.353554,0.612372,0.612372,-0.353554],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtTs2Approach:=[[50,20,30],[0.353554,0.612372,0.612372,-0.353554],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtTs2Contact:=[[50,20,0],[0.353554,0.612372,0.612372,-0.353554],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    ! The weld, 5 mm over the top face (a dry pass).
    LOCAL CONST robtarget rtW2Approach:=[[50,-30,40],[0.353554,0.612372,0.612372,-0.353554],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtW2Start:=[[50,-30,5],[0.353554,0.612372,0.612372,-0.353554],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtW2End:=[[50,30,5],[0.353554,0.612372,0.612372,-0.353554],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtW2Depart:=[[50,30,40],[0.353554,0.612372,0.612372,-0.353554],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    LOCAL FUNC robtarget tsAt(robtarget p)
        ! The positioner's CURRENT axes into a stored target. The exporter
        ! writes the station's real axes into every robtarget (plan D5); this
        ! demo copies them, so it runs at whatever index the VC stands.
        VAR robtarget r;
        VAR jointtarget jt;
        ! (A component of a function result, CJointT().extax, is a RAPID
        ! syntax error - VC-found 2026-09-26 - hence the copy.)
        jt:=CJointT();
        r:=p;
        r.extax:=jt.extax;
        RETURN r;
    ENDFUNC

    PROC TD05Touch()
        VAR jointtarget jtHome;

        ! --- program header (FANUC lines 1-8) ---
        stTG_ProgPass:="TD05Touch";
        stTG_RobStatus:="Ok";
        stTG_SubName:="none";
        wobjTG_Weld.uframe:=[[0,0,0],[1,0,0,0]];
        wobjTG_Weld.oframe:=[[0,0,0],[1,0,0,0]];
        ConfJ\Off;
        ConfL\Off;
        jtHome:=CJointT();
        jtHome.robax:=[0,0,0,0,30,0];
        MoveAbsJ jtHome,v200,fine,tTG_Weld;

        TG_ReqPassCheck \Tool:=tTG_Weld \WObj:=wobj0;
        IF nTG_PassOK=0 THEN
            RETURN;
        ENDIF

        ! ================ TouchSense for Weld2 (FANUC lines 76-196) =========
        stTG_SubName:="TSPWeld2_full";
        TG_ReqWeldFrame \Tool:=tTG_Weld \WObj:=wobjTG_Weld;
        TG_ReqTouchSenseDo;
        TG_CamClose;
        IF nTG_DoTouchSense=1 THEN
            ! No id 17 on ABB (D2). FANUC: CALL TEACHMODE_ON.
            TG_TeachModeOn;
            ! Bridge to touch 1 (FANUC's J P[20..51] CNT100 run).
            MoveJ tsAt(rtTsVia),v500,z10,tTG_Weld\WObj:=wobjTG_Weld;
            ! Touch 1 - FANUC "L P[52] FINE ; L P[53] FINE Search[+X] ;"
            MoveL tsAt(rtTs1Approach),vTsApproach,fine,tTG_Weld\WObj:=wobjTG_Weld;
            TG_TouchSearch tsAt(rtTs1Approach),tsAt(rtTs1Contact),tTG_Weld,wobjTG_Weld;
            TG_ReqTouchSensePoint;
            WaitTime 0.5;
            ! FANUC "L P[54] FINE" - the return (D12).
            MoveL tsAt(rtTs1Approach),vTsApproach,fine,tTG_Weld\WObj:=wobjTG_Weld;
            ! Bridge to touch 2.
            MoveL tsAt(rtTsVia),vTsApproach,z10,tTG_Weld\WObj:=wobjTG_Weld;
            ! Touch 2 - FANUC "L P[87] FINE ; L P[88] FINE Search[-Z] ;"
            MoveL tsAt(rtTs2Approach),vTsApproach,fine,tTG_Weld\WObj:=wobjTG_Weld;
            TG_TouchSearch tsAt(rtTs2Approach),tsAt(rtTs2Contact),tTG_Weld,wobjTG_Weld;
            TG_ReqTouchSensePoint;
            WaitTime 0.5;
            MoveL tsAt(rtTs2Approach),vTsApproach,fine,tTG_Weld\WObj:=wobjTG_Weld;
            TG_ReqTouchSenseEnd \WObj:=wobjTG_Weld;
        ENDIF

        ! ================ Start of Weld2 (FANUC lines 197-226) ==============
        ! The weld frame re-served here carries the auto touch-up.
        stTG_SubName:="PWeld2";
        TG_ReqWeldFrame \Tool:=tTG_Weld \WObj:=wobjTG_Weld;
        TG_CamClose;
        IF nTG_WeldStatus=2 GOTO abort_end;
        IF nTG_WeldStatus=1 THEN
            MoveJ tsAt(rtW2Approach),v500,z10,tTG_Weld\WObj:=wobjTG_Weld;
            MoveL tsAt(rtW2Start),vTsApproach,fine,tTG_Weld\WObj:=wobjTG_Weld;
            ! Dry pass: TD05Weld.mod shows ArcLStart / ArcLEnd here.
            MoveL tsAt(rtW2End),v20,fine,tTG_Weld\WObj:=wobjTG_Weld;
            MoveL tsAt(rtW2Depart),vTsApproach,fine,tTG_Weld\WObj:=wobjTG_Weld;
        ENDIF
        jtHome:=CJointT();
        jtHome.robax:=[0,0,0,0,30,0];
        MoveAbsJ jtHome,v200,fine,tTG_Weld;

abort_end:
        ConfJ\On;
        ConfL\On;
        stTG_SubName:="none";
        TG_ReqEnd \Tool:=tTG_Weld \WObj:=wobjTG_Weld;
    ENDPROC

ENDMODULE
