MODULE TG_TrRig
    ! VC-ONLY TEST RIG 2026-09-26 - NOT FOR PRODUCTION, never load on a real cell.
    !
    ! Touch-sense P3 (docs/abb_touch_sense_port_v1.md section 6): gives
    ! TGS/TD05Touch.mod a part to touch while TG_Main runs it for real.
    !   TG_TrRun   point TG_Cell's welder names at the World Zone rig signals
    !              (TG_TouchSimEIO.cfg), build the part, then call the bench
    !              standalone loop tgs_main - the whole TG cycle, unchanged.
    !   TG_TrRestore  TG_Cell's names back, no part, no timer.
    !
    ! The part is TD05Touch's block (x 0..100, y -50..50, z -200..0 in the
    ! served frame [1600,0,1450 | Rz 90]) mapped to world: a box, world-
    ! aligned because the frame only turns about Z. hmi_prototype/abb_server.py
    ! TOUCH_REAL_DEMO holds the same numbers; test_phase8_touchsense.py checks
    ! that the three files agree. nTrShiftY / nTrShiftZ move the box in world
    ! before a run: +Y is part +X (touch 1), +Z is part +Z (touch 2).
    !
    ! A 0.1 s timer TRAP re-makes the box if anything erases it (a temporary
    ! world zone is erased "when a new program is loaded", TRM 3.104 - the
    ! cycle Loads the .tgs module); nTrZoneMade counts the builds, so a run
    ! that needed a re-make says so. Every global carries "Tr" (finding F-5).

    PERS num nTrShiftY:=0;
    PERS num nTrShiftZ:=0;
    PERS num nTrZoneMade:=0;
    PERS string stTrStep:="";
    PERS bool bTrSaved:=FALSE;
    PERS string stTrSavedDI:="";
    PERS string stTrSavedOnDO:="";
    PERS string stTrSavedActiveDI:="";
    PERS string stTrSavedTeachDO:="";

    LOCAL CONST pos TR_LO:=[1550,0,1250];
    LOCAL CONST pos TR_HI:=[1650,100,1450];
    LOCAL VAR wztemporary wzTr;
    LOCAL VAR shapedata shTr;
    LOCAL VAR intnum irTr;
    LOCAL VAR bool bTrWant:=FALSE;

    TRAP trTrZone
        IF bTrWant AND wzTr.wz=0 TrZoneMake;
    ENDTRAP

    LOCAL PROC TrZoneMake()
        VAR pos sh;
        VAR pos lo;
        VAR pos hi;
        sh:=[0,nTrShiftY,nTrShiftZ];
        lo:=TR_LO+sh;
        hi:=TR_HI+sh;
        WZBoxDef\Inside,shTr,lo,hi;
        WZDOSet\Temp,wzTr\Inside,shTr,doTG_SimTouch,1;
        nTrZoneMade:=nTrZoneMade+1;
    ENDPROC

    LOCAL PROC TrZoneFree()
        IF wzTr.wz<>0 TrZoneFreeNow;
        SetDO doTG_SimTouch,0;
    ENDPROC

    LOCAL PROC TrZoneFreeNow()
        WZFree wzTr;
    ERROR
        TRYNEXT;
    ENDPROC

    PROC TG_TrRun()
        stTrStep:="rig on";
        IF NOT bTrSaved THEN
            stTrSavedDI:=stTG_TouchDI;
            stTrSavedOnDO:=stTG_TouchOnDO;
            stTrSavedActiveDI:=stTG_TouchActiveDI;
            stTrSavedTeachDO:=stTG_TeachModeDO;
            bTrSaved:=TRUE;
        ENDIF
        stTG_TouchDI:="diTG_SimTouched";
        stTG_TouchOnDO:="doTG_SimSensorOn";
        stTG_TouchActiveDI:="diTG_SimSensorActive";
        stTG_TeachModeDO:="";
        IDelete irTr;
        bTrWant:=FALSE;
        TrZoneFree;
        nTrZoneMade:=0;
        bTrWant:=TRUE;
        TrZoneMake;
        CONNECT irTr WITH trTrZone;
        ITimer 0.1,irTr;
        stTrStep:="serving";
        ! The bench standalone loop (TG_Main.mod, bench name tgs_main): the
        ! handshake, the run, the late-bound TD05Touch - nothing rig-specific.
        tgs_main;
    ENDPROC

    PROC TG_TrRestore()
        IDelete irTr;
        bTrWant:=FALSE;
        TrZoneFree;
        IF bTrSaved THEN
            stTG_TouchDI:=stTrSavedDI;
            stTG_TouchOnDO:=stTrSavedOnDO;
            stTG_TouchActiveDI:=stTrSavedActiveDI;
            stTG_TeachModeDO:=stTrSavedTeachDO;
            bTrSaved:=FALSE;
        ENDIF
        SetDO doTG_SimSensorOn,0;
        stTrStep:="restored";
    ENDPROC

    PROC TG_TrHome()
        VAR jointtarget jt;
        jt:=CJointT();
        jt.robax:=[0,0,0,0,30,0];
        MoveAbsJ jt,v200,fine,tTG_Weld;
    ENDPROC
ENDMODULE
