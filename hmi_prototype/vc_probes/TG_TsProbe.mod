MODULE TG_TsProbe
    ! VC EXPERIMENT 2026-09-26 - NOT FOR PRODUCTION, never load on a real cell.
    !
    ! Touch-sense P2 (docs/abb_touch_sense_port_v1.md section 6): the
    ! PRODUCTION search primitive TG_Touch.TG_TouchSearch, driven on the P0
    ! World Zone rig (TG_TouchSimEIO.cfg must be loaded), in a work object
    ! with a non-identity oframe. Each routine is one scenario; run by
    ! touch_search_vc.py, which plays the operator in the paused ones:
    !   S1 TG_TspHit       part present: one search, a hit, NO return (D12)
    !   S2 TG_TspMissRetry no part: miss -> pause 3 -> "part placed" -> Start -> hit
    !   S3 TG_TspNotLive   welder never confirms -> pause 2 -> fixed -> Start -> hit
    !   S4 TG_TspNoSignal  contact signal name unknown -> pause 1 -> fixed -> hit
    !   S5 TG_TspAtStart   wire touching at the start -> pause 4 -> cleared -> hit
    !
    ! The rig replaces the welder by name: stTG_TouchDI = diTG_SimTouched
    ! (the World Zone box via the cross connection), the sense-on DO /
    ! active DI = doTG_SimSensorOn / diTG_SimSensorActive (a cross connection,
    ! so TG_TouchSenseOn's handshake really runs). TG_Cell's values are saved
    ! first and put back by TG_TspRestore - the runner calls it in any case.
    !
    ! The "part" follows nTspZoneWant (0 none, 1 face 50 mm below S, 2 top
    ! 20 mm above S = touching): a 0.1 s timer TRAP applies it, so the runner
    ! can change the part WHILE TG_TouchSearch is paused, by writing one PERS.
    ! Every global carries "Tsp" (finding F-5).

    PERS tooldata tTspTorch:=[TRUE,[[0,0,300],[1,0,0,0]],[2,[0,0,100],[1,0,0,0],0,0,0]];
    PERS wobjdata wobjTspPart:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];

    LOCAL CONST num TSP_DROP:=150;
    LOCAL CONST num TSP_FACE:=50;
    LOCAL VAR wztemporary wzTsp;
    LOCAL VAR shapedata shTsp;
    LOCAL VAR intnum irTsp;

    PERS string stTspStep:="";
    PERS num nTspZoneWant:=0;
    PERS num nTspZoneHave:=0;
    PERS num nTspFaceZ:=0;
    PERS num nTspPauses:=0;
    PERS num nTspLastPause:=0;
    PERS num nTspSensorAfter:=-1;
    PERS bool bTspLiveAfter:=FALSE;
    PERS bool bTspHitOK:=FALSE;
    PERS robtarget pTspStartW:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pTspStartPart:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pTspContactPart:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pTspHit:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pTspAfterW:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pTspAfterPart:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    ! TG_Cell's welder names, saved while the rig names are in.
    PERS bool bTspSaved:=FALSE;
    PERS string stTspSavedDI:="";
    PERS string stTspSavedOnDO:="";
    PERS string stTspSavedActiveDI:="";
    PERS string stTspSavedTeachDO:="";

    ! ------------------------------------------------------------------
    TRAP trTspZone
        IF nTspZoneWant<>nTspZoneHave TspZoneApply nTspZoneWant;
    ENDTRAP

    LOCAL PROC TspZoneApply(num nWant)
        VAR pos lo;
        VAR pos hi;
        VAR num nTop;
        IF wzTsp.wz<>0 TspZoneFree;
        SetDO doTG_SimTouch,0;
        IF nWant=1 OR nWant=2 THEN
            nTop:=nTspFaceZ;
            IF nWant=2 nTop:=pTspStartW.trans.z+20;
            lo:=[pTspStartW.trans.x-300,pTspStartW.trans.y-300,nTop-600];
            hi:=[pTspStartW.trans.x+300,pTspStartW.trans.y+300,nTop];
            WZBoxDef\Inside,shTsp,lo,hi;
            WZDOSet\Temp,wzTsp\Inside,shTsp,doTG_SimTouch,1;
        ENDIF
        nTspZoneHave:=nWant;
    ENDPROC

    LOCAL PROC TspZoneFree()
        WZFree wzTsp;
    ERROR
        TRYNEXT;
    ENDPROC

    ! Rig names in (saved once), park, geometry, the part as asked.
    LOCAL PROC TspPrepare(num nWant,string stDI,string stActiveDI)
        VAR jointtarget jt;
        VAR robtarget p0;
        VAR pos pOffs:=[80,-60,-120];
        VAR pos pContactW;
        IF NOT bTspSaved THEN
            stTspSavedDI:=stTG_TouchDI;
            stTspSavedOnDO:=stTG_TouchOnDO;
            stTspSavedActiveDI:=stTG_TouchActiveDI;
            stTspSavedTeachDO:=stTG_TeachModeDO;
            bTspSaved:=TRUE;
        ENDIF
        stTG_TouchDI:=stDI;
        stTG_TouchOnDO:="doTG_SimSensorOn";
        stTG_TouchActiveDI:=stActiveDI;
        stTG_TeachModeDO:="";
        nTspPauses:=nTG_TouchPauses;
        nTspLastPause:=0;
        nTG_TouchLastPause:=0;
        nTspSensorAfter:=-1;
        bTspHitOK:=FALSE;
        ! No part until the geometry is known.
        IDelete irTsp;
        nTspZoneWant:=0;
        TspZoneApply 0;
        ConfL\Off;
        ConfJ\Off;
        jt:=CJointT();
        jt.robax:=[0,0,0,0,30,0];
        MoveAbsJ jt,v200,fine,tTspTorch;
        WaitRob\InPos;
        WaitRob\ZeroSpeed;
        WaitTime 0.2;
        p0:=CRobT(\Tool:=tTspTorch\WObj:=wobj0);
        pTspStartW:=p0;
        pTspStartW.trans.z:=p0.trans.z-TSP_DROP;
        nTspFaceZ:=pTspStartW.trans.z-TSP_FACE;
        ! A tilted part frame, identity uframe (THE RULE).
        wobjTspPart.uframe:=[[0,0,0],[1,0,0,0]];
        wobjTspPart.oframe.trans:=pTspStartW.trans+pOffs;
        wobjTspPart.oframe.rot:=OrientZYX(35,10,-20);
        MoveL pTspStartW,v200,fine,tTspTorch\WObj:=wobj0;
        WaitRob\InPos;
        WaitRob\ZeroSpeed;
        WaitTime 0.2;
        pTspStartPart:=CRobT(\Tool:=tTspTorch\WObj:=wobjTspPart);
        ! Nominal contact: on the face, straight below S - in part coordinates.
        pContactW:=pTspStartW.trans;
        pContactW.z:=nTspFaceZ;
        pTspContactPart:=pTspStartPart;
        pTspContactPart.trans:=PoseVect(PoseInv(wobjTspPart.oframe),pContactW);
        CONNECT irTsp WITH trTspZone;
        ITimer 0.1,irTsp;
        nTspZoneWant:=nWant;
        WaitUntil nTspZoneHave=nWant\MaxTime:=3;
    ENDPROC

    LOCAL PROC TspSearchAndRecord()
        TG_TouchSearch pTspStartPart,pTspContactPart,tTspTorch,wobjTspPart;
        ! Where the primitive LEFT the robot (D12: no return), then the
        ! program's own return, as the exporter emits it.
        WaitRob\ZeroSpeed;
        WaitTime 0.2;
        pTspHit:=rtTG_TouchHit;
        bTspHitOK:=bTG_TouchHitOK;
        bTspLiveAfter:=bTG_TouchLive;
        nTspSensorAfter:=DOutput(doTG_SimSensorOn);
        pTspAfterW:=CRobT(\Tool:=tTspTorch\WObj:=wobj0);
        pTspAfterPart:=CRobT(\Tool:=tTspTorch\WObj:=wobjTspPart);
        nTspPauses:=nTG_TouchPauses-nTspPauses;
        nTspLastPause:=nTG_TouchLastPause;
        MoveL pTspStartPart,v200,fine,tTspTorch\WObj:=wobjTspPart;
        nTspZoneWant:=0;
        WaitUntil nTspZoneHave=0\MaxTime:=3;
    ENDPROC

    ! ------------------------------------------------------------------
    PROC TG_TspHit()
        stTspStep:="S1 start";
        TspPrepare 1,"diTG_SimTouched","diTG_SimSensorActive";
        stTspStep:="S1 searching";
        TspSearchAndRecord;
        stTspStep:="S1 done";
    ENDPROC

    PROC TG_TspMissRetry()
        stTspStep:="S2 start";
        TspPrepare 0,"diTG_SimTouched","diTG_SimSensorActive";
        stTspStep:="S2 searching";
        TspSearchAndRecord;
        stTspStep:="S2 done";
    ENDPROC

    PROC TG_TspNotLive()
        stTspStep:="S3 start";
        ! The "welder" confirms on a signal that never rises before contact.
        TspPrepare 1,"diTG_SimTouched","diTG_SimTouched";
        stTspStep:="S3 searching";
        TspSearchAndRecord;
        stTspStep:="S3 done";
    ENDPROC

    PROC TG_TspNoSignal()
        stTspStep:="S4 start";
        TspPrepare 1,"diTG_NoSuchSignal","diTG_SimSensorActive";
        stTspStep:="S4 searching";
        TspSearchAndRecord;
        stTspStep:="S4 done";
    ENDPROC

    PROC TG_TspAtStart()
        stTspStep:="S5 start";
        TspPrepare 2,"diTG_SimTouched","diTG_SimSensorActive";
        stTspStep:="S5 searching";
        TspSearchAndRecord;
        stTspStep:="S5 done";
    ENDPROC

    ! Put TG_Cell's names back, no part, no timer. The runner calls this
    ! whatever happened.
    PROC TG_TspRestore()
        IDelete irTsp;
        TspZoneApply 0;
        IF bTspSaved THEN
            stTG_TouchDI:=stTspSavedDI;
            stTG_TouchOnDO:=stTspSavedOnDO;
            stTG_TouchActiveDI:=stTspSavedActiveDI;
            stTG_TeachModeDO:=stTspSavedTeachDO;
            bTspSaved:=FALSE;
        ENDIF
        SetDO doTG_SimSensorOn,0;
        stTspStep:="restored";
    ENDPROC

    PROC TG_TspHome()
        VAR jointtarget jt;
        jt:=CJointT();
        jt.robax:=[0,0,0,0,30,0];
        MoveAbsJ jt,v200,fine,tTspTorch;
    ENDPROC
ENDMODULE
