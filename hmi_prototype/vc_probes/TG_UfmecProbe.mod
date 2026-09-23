MODULE TG_UfmecProbe
    ! VC EXPERIMENT 2026-09-23 - NOT FOR PRODUCTION, never load on a real cell.
    !
    ! Question: can ONE coordinated work object follow whichever station is at
    ! the robot, by binding it at run time (wobj.ufmec := "STN1" / "STN2")?
    ! And is every wrong binding LOUD - empty, the inactive station, no such
    ! unit - or can one of them silently coordinate with the wrong thing?
    ! Plan: TGuideWeldingHMI docs/socket_start_trigger_hmi_plan_v1.md,
    ! coordinated station agnosticism.
    !
    ! One routine per check, each run on its own (PP to routine), so an
    ! unhandled error stops only that check and lands in the event log.
    ! stPrbStep records how far a check got before it stopped.

    ! Declared UNBOUND (ufmec ""). Loading this module at all is check T0.
    PERS wobjdata wobjPrb:=[FALSE,FALSE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];

    PERS string stPrbStep:="";
    PERS string stPrbUfmec:="";
    PERS num nPrbStn:=0;
    PERS jointtarget jtPrbHome:=[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pPrbS0:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pPrbS1:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pPrbW0:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pPrbW1:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pPrbRef:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    ! Robot pose near the coordinated weld entry of abb_test_coord.tgs
    ! (its jtTG0008), chuck at 1.839 deg: known reachable, close to the part.
    CONST jointtarget jtPrbNear:=[[18.844,6.029,23.357,-12.826,65.168,114.585],[0.000,0.000,1.839,9E9,9E9,9E9]];

    ! Common start: the station at the robot is made active the production
    ! way (TG_ActMechUnit - outside PM this takes its ActStn<n> branch).
    LOCAL PROC PrbPrepare()
        TG_ActMechUnit;
        nPrbStn:=0;
        IF IsMechUnitActive(STN1) nPrbStn:=1;
        IF IsMechUnitActive(STN2) nPrbStn:=nPrbStn+2;
        jtPrbHome:=CJointT();
        ConfL \Off;
        ConfJ \Off;
    ENDPROC

    ! T1: a position read through the UNBOUND work object.
    PROC TG_PrbUnbound()
        stPrbStep:="T1 start";
        PrbPrepare;
        wobjPrb.ufmec:="";
        stPrbUfmec:=wobjPrb.ufmec;
        stPrbStep:="T1 CRobT with ufmec empty";
        pPrbS0:=CRobT(\Tool:=tool0 \WObj:=wobjPrb);
        stPrbStep:="T1 CRobT returned WITHOUT error";
    ENDPROC

    ! T2: bind to the ACTIVE station; coordinated MoveL that turns the chuck
    ! 10 deg. Coordinated = the pose relative to the station stays put while
    ! the pose relative to the world moves.
    PROC TG_PrbBind()
        VAR robtarget p;
        stPrbStep:="T2 start";
        PrbPrepare;
        MoveAbsJ jtPrbNear,v200,fine,tool0;
        IF nPrbStn=1 wobjPrb.ufmec:="STN1";
        IF nPrbStn=2 wobjPrb.ufmec:="STN2";
        stPrbUfmec:=wobjPrb.ufmec;
        stPrbStep:="T2 bound, reading";
        pPrbS0:=CRobT(\Tool:=tool0 \WObj:=wobjPrb);
        pPrbW0:=CRobT(\Tool:=tool0 \WObj:=wobj0);
        pPrbRef:=pPrbS0;
        p:=pPrbS0;
        p.extax.eax_c:=p.extax.eax_c+10;
        stPrbStep:="T2 coordinated MoveL, chuck +10";
        MoveL p,v100,fine,tool0\WObj:=wobjPrb;
        pPrbS1:=CRobT(\Tool:=tool0 \WObj:=wobjPrb);
        pPrbW1:=CRobT(\Tool:=tool0 \WObj:=wobj0);
        stPrbStep:="T2 returning";
        MoveAbsJ jtPrbNear,v200,fine,tool0;
        MoveAbsJ jtPrbHome,v200,fine,tool0;
        stPrbStep:="T2 done";
    ENDPROC

    ! T3: bound to the station that is NOT active. If the read does not
    ! error, the MoveL to T2's station-relative pose shows what it does.
    PROC TG_PrbWrongStn()
        stPrbStep:="T3 start";
        PrbPrepare;
        IF nPrbStn=1 wobjPrb.ufmec:="STN2";
        IF nPrbStn=2 wobjPrb.ufmec:="STN1";
        stPrbUfmec:=wobjPrb.ufmec;
        stPrbStep:="T3 CRobT bound to the inactive station";
        pPrbS0:=CRobT(\Tool:=tool0 \WObj:=wobjPrb);
        stPrbStep:="T3 CRobT returned WITHOUT error, trying MoveL";
        MoveAbsJ jtPrbNear,v200,fine,tool0;
        MoveL pPrbRef,v100,fine,tool0\WObj:=wobjPrb;
        pPrbW1:=CRobT(\Tool:=tool0 \WObj:=wobj0);
        stPrbStep:="T3 MoveL returned WITHOUT error";
        MoveAbsJ jtPrbHome,v200,fine,tool0;
    ENDPROC

    ! Back to PM's safe pose (robot axes 0,0,0,0,30,0; external axes left
    ! where they are) BEFORE PM restarts. Otherwise PM's EE_START GoSafe finds
    ! the robot away from safe and waits on a pendant dialog (seen 2026-09-23
    ! after T3, which stops at the part by design).
    PROC TG_PrbHome()
        VAR jointtarget jt;
        stPrbStep:="home start";
        jt:=CJointT();
        jt.robax:=[0,0,0,0,30,0];
        MoveAbsJ jt,v200,fine,tool0;
        stPrbStep:="home done";
    ENDPROC

    ! T4: bound to a unit name this controller does not have.
    PROC TG_PrbNoUnit()
        stPrbStep:="T4 start";
        PrbPrepare;
        wobjPrb.ufmec:="STN9";
        stPrbUfmec:=wobjPrb.ufmec;
        stPrbStep:="T4 CRobT bound to STN9";
        pPrbS0:=CRobT(\Tool:=tool0 \WObj:=wobjPrb);
        stPrbStep:="T4 CRobT returned WITHOUT error";
    ENDPROC
ENDMODULE
