MODULE TD05TsWire_Mod
    !***********************************************************************
    ! Sample .tgs program - the auto touch-up WIRE (ids 16/18/19), no motion.
    !
    ! Touch-sense P1 (docs/abb_touch_sense_port_v1.md section 6): proves the
    ! three requests and the .oframe write end to end, against the prototype
    ! HMI, before any search primitive exists. It is the executable spec of
    ! the TSPWeld2_full block of the FANUC sample TD05tRJYQd.ls (lines
    ! 76-196) with the two searches (Search[+X], then Search[-Z]) PRETENDED:
    ! each writes the contact point the real primitive will write
    ! (TG_TouchSearch, P2) and reports it.
    !
    ! Mirrors hmi_prototype/abb_server.py TOUCH_WIRE_DEMO, which holds the
    ! NOMINAL touch points. The "measured" points below differ from them by
    ! +3.000 mm in part X (touch 1) and -2.000 mm in part Z (touch 2), and
    ! each also carries junk in the coordinates its search does NOT measure,
    ! which the HMI's per-axis math must ignore. Expected result: the frame
    ! served at id 19 - and re-served by the PWeld2 R_W_F - is the
    ! localization frame translated by R * (3, 0, -2), R = its rotation.
    !
    ! Naming contract (plan 4.1): file name = PROC name = program name the
    ! HMI sends ("TD05TsWire", 10 chars - the R_F_T limit); the MODULE name
    ! carries "_Mod" (finding F-1).
    !***********************************************************************

    ! The pretended contact points, in wobjTG_Weld coordinates = the frame
    ! the TSP R_W_F served (CAD coordinates, as the HMI expects them).
    LOCAL CONST robtarget rtTsHit1:=[[103,0.7,20.4],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    LOCAL CONST robtarget rtTsHit2:=[[59.6,25.3,-2],[0,0,1,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    PROC TD05TsWire()
        ! --- program header (FANUC lines 1-8) ---
        stTG_ProgPass:="TD05TsWire";
        stTG_RobStatus:="Ok";
        stTG_SubName:="none";

        ! Frame nominals at entry, uframe identity included (THE RULE; see
        ! TD05Weld.mod for why the reset is contract, not tidiness).
        wobjTG_Weld.uframe:=[[0,0,0],[1,0,0,0]];
        wobjTG_Weld.oframe:=[[0,0,0],[1,0,0,0]];

        TG_ReqPassCheck \Tool:=tTG_Weld \WObj:=wobj0;
        IF nTG_PassOK=0 THEN
            ! FANUC 'END' - terminate immediately, NO end request.
            RETURN;
        ENDIF

        ! ================ TouchSense for Weld2 (FANUC lines 76-196) =========
        ! The TSP token names the weld; on its first pass the HMI clears the
        ! weld's stored touch-up and serves the localization-only frame.
        stTG_SubName:="TSPWeld2_full";
        TG_ReqWeldFrame \Tool:=tTG_Weld \WObj:=wobjTG_Weld;
        TG_ReqTouchSenseDo;
        TG_CamClose;
        IF nTG_DoTouchSense=1 THEN
            ! No id 17 on ABB (plan D2). P2 adds TG_TeachModeOn here, where
            ! FANUC calls TEACHMODE_ON.
            !
            ! Touch 1 - FANUC "L P[53] 775mm/sec FINE Search[+X]": PRETENDED.
            rtTG_TouchHit:=rtTsHit1;
            bTG_TouchHitOK:=TRUE;
            TG_ReqTouchSensePoint;
            ! Touch 2 - FANUC "L P[88] 775mm/sec FINE Search[-Z]": PRETENDED.
            rtTG_TouchHit:=rtTsHit2;
            bTG_TouchHitOK:=TRUE;
            TG_ReqTouchSensePoint;
            TG_ReqTouchSenseEnd \WObj:=wobjTG_Weld;
        ENDIF

        ! ================ Start of Weld2 (FANUC lines 197-226) ==============
        ! The weld's own R_W_F re-serves the corrected frame; this module
        ! does not weld - it tests the wire only.
        stTG_SubName:="PWeld2";
        TG_ReqWeldFrame \Tool:=tTG_Weld \WObj:=wobjTG_Weld;
        TG_CamClose;
        IF nTG_WeldStatus=2 GOTO abort_end;
        TPWrite "TD05TsWire: weld frame "\Pos:=wobjTG_Weld.oframe.trans;

abort_end:
        ! FANUC LBL[101]: both the normal exit and the abort target.
        stTG_SubName:="none";
        TG_ReqEnd \Tool:=tTG_Weld \WObj:=wobjTG_Weld;
    ENDPROC

ENDMODULE
