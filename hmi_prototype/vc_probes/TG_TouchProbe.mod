MODULE TG_TouchProbe
    ! VC EXPERIMENT 2026-09-25 - NOT FOR PRODUCTION, never load on a real cell.
    !
    ! Touch-sense port, phase P0 (docs/abb_touch_sense_port_v1.md section 6).
    ! Before any production touch-sense RAPID is written, find out on the
    ! MONARC VC:
    !   X1  does plain SearchL\Stop stop when its DI rises? curobo E17 says
    !       it runs the full stroke on a VC. Is SearchPoint the DETECTION
    !       point (at the part face) or the point where the robot stopped?
    !   X2  which frame is SearchPoint in? Searched in a work object with a
    !       NON-identity oframe, it must come back in that object's
    !       coordinates (section 3.3), i.e. hit_world = oframe * hit_part.
    !   X3  the two search errors (no contact; touching at the start) and
    !       the D3 policy: retreat, Stop, Start retries - end to end through
    !       a LATE-BOUND call, the way a .tgs program runs.
    !   X6  overshoot after \Stop at 15 mm/s (D9) against 50 mm/s.
    !
    ! THE PART IS A WORLD ZONE BOX (option 608-1). WZDOSet drives the VC-only
    ! DO doTG_SimTouch high while the TCP is inside it; the VC-only cross
    ! connection copies it to diTG_SimTouched, which SearchL supervises.
    ! Load TG_TouchSimEIO.cfg first or this module will not link. The box
    ! top face is a known plane, so every hit has a number to be judged
    ! against. No Miller signal and no TG_* data is read or written.
    !
    ! GEOMETRY, derived at run time from PM's safe pose (robot axes
    ! 0,0,0,0,30,0; external axes left where they are): search start S =
    ! that TCP 150 mm lower in world Z; search straight down at 15 mm/s;
    ! box top face 50 mm below S; ToPoint 150 mm below S (the D9 stroke),
    ! i.e. 100 mm inside the box.
    !
    ! One routine per experiment, each run alone (PP to routine, cycle once)
    ! by touch_probe.py, which reads the PERS results over RWS and judges
    ! them (touch_probe_math.py). stTpStep records how far a routine got.
    !
    ! Every global name carries "Tp". VC-proven 2026-09-25: a global PERS
    ! declared in two loaded modules - same name, same type, even the same
    ! value - is a semantic error (40160) that also blocks PP-to-main, and
    ! TG_UfmecProbe (still loaded on this VC) owns stPrbStep and nPrbStn.

    ! Probe tool: a torch-like TCP 300 mm out along the flange Z axis.
    PERS tooldata tTpTorch:=[TRUE,[[0,0,300],[1,0,0,0]],[2,[0,0,100],[1,0,0,0],0,0,0]];

    ! X2's work object: identity uframe (THE RULE), non-identity oframe set
    ! at run time.
    PERS wobjdata wobjTpPart:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];

    ! 15 mm/s is the FANUC touch schedule speed (D9). speeddata has no
    ! predefined v15.
    LOCAL CONST speeddata vTpSearch:=[15,500,5000,1000];
    LOCAL CONST speeddata vTpSearch50:=[50,500,5000,1000];
    LOCAL CONST num TP_DROP:=150;
    LOCAL CONST num TP_FACE:=50;
    LOCAL CONST num TP_STROKE:=150;

    ! A temporary world zone cannot be declared inside a routine.
    LOCAL VAR wztemporary wzTpPart;
    LOCAL VAR shapedata shTpPart;
    LOCAL VAR clock ckTp;

    PERS string stTpStep:="";
    PERS num nTpErrno:=0;
    PERS num nTpFaceZ:=0;
    PERS num nTpSearchS:=0;
    PERS num nTpRetries:=0;
    PERS num nTpDiAtStart:=-1;
    PERS num nTpFrameErr:=-1;
    PERS num nTpOffLine:=-1;
    PERS num nTpHitZ{3}:=[0,0,0];
    PERS num nTpStopZ{3}:=[0,0,0];
    PERS num nTpDepth{12}:=[0,0,0,0,0,0,0,0,0,0,0,0];
    PERS robtarget pTpStart:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pTpTo:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pTpHit:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pTpStop:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pTpErrPos:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pTpStartPart:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pTpToPart:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pTpHitPart:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pTpStopPart:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS robtarget pTpStopW:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

    ! ------------------------------------------------------------------
    ! Common start: park at the safe pose, derive S / ToPoint / face.
    LOCAL PROC TpPrepare()
        VAR jointtarget jt;
        VAR robtarget p0;
        TpZoneOff;
        nTpErrno:=0;
        nTpRetries:=0;
        nTpDiAtStart:=-1;
        ConfL\Off;
        ConfJ\Off;
        jt:=CJointT();
        jt.robax:=[0,0,0,0,30,0];
        MoveAbsJ jt,v200,fine,tTpTorch;
        ! Settle before reading: the first run read S 1.4 mm off (the known
        ! "CRobT right after a fine point" effect, TG_Comms tgSendPose) because
        ! the robot had just arrived here.
        WaitRob\InPos;
        WaitRob\ZeroSpeed;
        WaitTime 0.2;
        p0:=CRobT(\Tool:=tTpTorch\WObj:=wobj0);
        pTpStart:=p0;
        pTpStart.trans.z:=p0.trans.z-TP_DROP;
        pTpTo:=pTpStart;
        pTpTo.trans.z:=pTpStart.trans.z-TP_STROKE;
        nTpFaceZ:=pTpStart.trans.z-TP_FACE;
    ENDPROC

    ! The "part": a 600 x 600 x 600 mm box under S, top face at nTopZ.
    LOCAL PROC TpZoneOn(num nTopZ)
        VAR pos lo;
        VAR pos hi;
        TpZoneOff;
        lo:=[pTpStart.trans.x-300,pTpStart.trans.y-300,nTopZ-600];
        hi:=[pTpStart.trans.x+300,pTpStart.trans.y+300,nTopZ];
        WZBoxDef\Inside,shTpPart,lo,hi;
        WZDOSet\Temp,wzTpPart\Inside,shTpPart,doTG_SimTouch,1;
    ENDPROC

    LOCAL PROC TpZoneOff()
        IF wzTpPart.wz<>0 TpZoneFree;
        SetDO doTG_SimTouch,0;
    ENDPROC

    LOCAL PROC TpZoneFree()
        WZFree wzTpPart;
    ERROR
        ! Already erased by a program start: nothing to free.
        TRYNEXT;
    ENDPROC

    ! One search from S straight down, in world. No handler here: an error
    ! reaches the calling experiment's handler (a normal call, not late
    ! bound).
    LOCAL PROC TpSearchWorld(speeddata vSearch)
        MoveL pTpStart,v200,fine,tTpTorch\WObj:=wobj0;
        WaitTime 0.2;
        nTpDiAtStart:=DInput(diTG_SimTouched);
        ClkReset ckTp;
        ClkStart ckTp;
        SearchL\Stop,diTG_SimTouched,pTpHit,pTpTo,vSearch,tTpTorch\WObj:=wobj0;
        ClkStop ckTp;
        nTpSearchS:=ClkRead(ckTp);
        WaitRob\ZeroSpeed;
        pTpStop:=CRobT(\Tool:=tTpTorch\WObj:=wobj0);
        MoveL pTpStart,v200,fine,tTpTorch\WObj:=wobj0;
    ENDPROC

    ! ------------------------------------------------------------------
    ! X1: three identical searches at 15 mm/s onto the box face.
    ! Expect: the robot stops a few mm past the face (not at the ToPoint 100
    ! mm further on); SearchPoint AT the face; the three hits within 0.3 mm.
    PROC TG_TpX1()
        VAR num i;
        stTpStep:="X1 start";
        TpPrepare;
        TpZoneOn nTpFaceZ;
        FOR i FROM 1 TO 3 DO
            stTpStep:="X1 search "+NumToStr(i,0);
            TpSearchWorld vTpSearch;
            nTpHitZ{i}:=pTpHit.trans.z;
            nTpStopZ{i}:=pTpStop.trans.z;
        ENDFOR
        TpZoneOff;
        stTpStep:="X1 done";
    ERROR
        nTpErrno:=ERRNO;
        pTpErrPos:=CRobT(\Tool:=tTpTorch\WObj:=wobj0);
        stTpStep:=stTpStep+" | ERRNO "+NumToStr(ERRNO,0);
        RETURN;
    ENDPROC

    ! X6: one search at 50 mm/s, for the overshoot comparison with X1.
    PROC TG_TpX6()
        stTpStep:="X6 start";
        TpPrepare;
        TpZoneOn nTpFaceZ;
        stTpStep:="X6 search at 50 mm/s";
        TpSearchWorld vTpSearch50;
        nTpHitZ{1}:=pTpHit.trans.z;
        nTpStopZ{1}:=pTpStop.trans.z;
        TpZoneOff;
        stTpStep:="X6 done";
    ERROR
        nTpErrno:=ERRNO;
        pTpErrPos:=CRobT(\Tool:=tTpTorch\WObj:=wobj0);
        stTpStep:=stTpStep+" | ERRNO "+NumToStr(ERRNO,0);
        RETURN;
    ENDPROC

    ! X2: the same world path as X1, searched in wobjTpPart.
    PROC TG_TpX2()
        VAR pose peInv;
        VAR pos pOffs:=[80,-60,-120];
        VAR pos u;
        VAR pos d;
        stTpStep:="X2 start";
        TpPrepare;
        wobjTpPart.uframe:=[[0,0,0],[1,0,0,0]];
        wobjTpPart.oframe.trans:=pTpStart.trans+pOffs;
        wobjTpPart.oframe.rot:=OrientZYX(35,10,-20);
        TpZoneOn nTpFaceZ;
        MoveL pTpStart,v200,fine,tTpTorch\WObj:=wobj0;
        WaitRob\InPos;
        WaitRob\ZeroSpeed;
        WaitTime 0.2;
        pTpStartPart:=CRobT(\Tool:=tTpTorch\WObj:=wobjTpPart);
        ! The ToPoint in part coordinates: world -> object is inv(oframe),
        ! because uframe is identity.
        peInv:=PoseInv(wobjTpPart.oframe);
        pTpToPart:=pTpStartPart;
        pTpToPart.trans:=PoseVect(peInv,pTpTo.trans);
        stTpStep:="X2 searching in wobjTpPart";
        SearchL\Stop,diTG_SimTouched,pTpHitPart,pTpToPart,vTpSearch,tTpTorch\WObj:=wobjTpPart;
        WaitRob\ZeroSpeed;
        pTpStopPart:=CRobT(\Tool:=tTpTorch\WObj:=wobjTpPart);
        pTpStopW:=CRobT(\Tool:=tTpTorch\WObj:=wobj0);
        ! CRobT's own consistency at the stop: expect ~0 mm.
        nTpFrameErr:=VectMagn(PoseVect(wobjTpPart.oframe,pTpStopPart.trans)-pTpStopW.trans);
        ! The hit's distance from the part-frame search line: expect ~0 mm.
        u:=pTpToPart.trans-pTpStartPart.trans;
        u:=(1/VectMagn(u))*u;
        d:=pTpHitPart.trans-pTpStartPart.trans;
        nTpOffLine:=VectMagn(d-(DotProd(d,u)*u));
        MoveL pTpStartPart,v200,fine,tTpTorch\WObj:=wobjTpPart;
        TpZoneOff;
        stTpStep:="X2 done";
    ERROR
        nTpErrno:=ERRNO;
        pTpErrPos:=CRobT(\Tool:=tTpTorch\WObj:=wobj0);
        stTpStep:=stTpStep+" | ERRNO "+NumToStr(ERRNO,0);
        RETURN;
    ENDPROC

    ! X3a: no part (no zone). Expect ERR_WHLSEARCH after the full 150 mm
    ! stroke (~10 s at 15 mm/s), the robot AT the ToPoint when the error
    ! arrives, then the manual's recovery choreography back to S.
    PROC TG_TpX3Miss()
        stTpStep:="X3a start";
        TpPrepare;
        MoveL pTpStart,v200,fine,tTpTorch\WObj:=wobj0;
        WaitTime 0.2;
        ClkReset ckTp;
        ClkStart ckTp;
        stTpStep:="X3a searching, no part";
        SearchL\Stop,diTG_SimTouched,pTpHit,pTpTo,vTpSearch,tTpTorch\WObj:=wobj0;
        stTpStep:="X3a UNEXPECTED hit";
        RETURN;
    ERROR
        ClkStop ckTp;
        nTpSearchS:=ClkRead(ckTp);
        nTpErrno:=ERRNO;
        pTpErrPos:=CRobT(\Tool:=tTpTorch\WObj:=wobj0);
        IF ERRNO=ERR_WHLSEARCH THEN
            stTpStep:="X3a ERR_WHLSEARCH, recovering";
            StorePath;
            MoveL pTpStart,v200,fine,tTpTorch\WObj:=wobj0;
            RestoPath;
            ClearPath;
            StartMove;
            pTpStop:=CRobT(\Tool:=tTpTorch\WObj:=wobj0);
            stTpStep:="X3a ERR_WHLSEARCH, back at S";
            RETURN;
        ENDIF
        stTpStep:="X3a other error "+NumToStr(ERRNO,0);
        RETURN;
    ENDPROC

    ! X3b: already touching (box top 20 mm ABOVE S, so the DI is high
    ! before the search). Expect ERR_SIGSUPSEARCH, robot stopped at S.
    PROC TG_TpX3Sig()
        stTpStep:="X3b start";
        TpPrepare;
        MoveL pTpStart,v200,fine,tTpTorch\WObj:=wobj0;
        TpZoneOn pTpStart.trans.z+20;
        WaitTime 0.3;
        nTpDiAtStart:=DInput(diTG_SimTouched);
        stTpStep:="X3b searching, touching at start";
        SearchL\Stop,diTG_SimTouched,pTpHit,pTpTo,vTpSearch,tTpTorch\WObj:=wobj0;
        stTpStep:="X3b UNEXPECTED: no supervision error";
        WaitRob\ZeroSpeed;
        pTpStop:=CRobT(\Tool:=tTpTorch\WObj:=wobj0);
        TpZoneOff;
        MoveL pTpStart,v200,fine,tTpTorch\WObj:=wobj0;
        RETURN;
    ERROR
        nTpErrno:=ERRNO;
        pTpErrPos:=CRobT(\Tool:=tTpTorch\WObj:=wobj0);
        IF ERRNO=ERR_SIGSUPSEARCH THEN
            stTpStep:="X3b ERR_SIGSUPSEARCH, recovering";
            TpZoneOff;
            StorePath;
            MoveL pTpStart,v200,fine,tTpTorch\WObj:=wobj0;
            RestoPath;
            ClearPath;
            StartMove;
            pTpStop:=CRobT(\Tool:=tTpTorch\WObj:=wobj0);
            stTpStep:="X3b ERR_SIGSUPSEARCH, back at S";
            RETURN;
        ENDIF
        stTpStep:="X3b other error "+NumToStr(ERRNO,0);
        RETURN;
    ENDPROC

    ! X3c: the D3 policy end to end, through LATE BINDING as a .tgs runs:
    ! miss -> retreat -> Stop (the runner plays the operator and presses
    ! Start) -> the part is now there (zone on) -> RETRY hits.
    PROC TG_TpX3Pause()
        stTpStep:="X3c start";
        TpPrepare;
        %"TG_TpX3PauseBody"%;
        stTpStep:=stTpStep+" | caller resumed";
    ENDPROC

    PROC TG_TpX3PauseBody()
        MoveL pTpStart,v200,fine,tTpTorch\WObj:=wobj0;
        WaitTime 0.2;
        stTpStep:="X3c searching";
        SearchL\Stop,diTG_SimTouched,pTpHit,pTpTo,vTpSearch,tTpTorch\WObj:=wobj0;
        WaitRob\ZeroSpeed;
        pTpStop:=CRobT(\Tool:=tTpTorch\WObj:=wobj0);
        stTpStep:="X3c hit after "+NumToStr(nTpRetries,0)+" retry";
        MoveL pTpStart,v200,fine,tTpTorch\WObj:=wobj0;
        TpZoneOff;
    ERROR
        nTpErrno:=ERRNO;
        IF ERRNO=ERR_WHLSEARCH AND nTpRetries<1 THEN
            pTpErrPos:=CRobT(\Tool:=tTpTorch\WObj:=wobj0);
            StorePath;
            MoveL pTpStart,v200,fine,tTpTorch\WObj:=wobj0;
            RestoPath;
            ClearPath;
            StartMove;
            nTpRetries:=nTpRetries+1;
            stTpStep:="X3c paused";
            TPWrite "TG PROBE X3c: no contact - paused, Start retries";
            Stop;
            ! The "operator" fixed the part.
            TpZoneOn nTpFaceZ;
            stTpStep:="X3c retrying";
            RETRY;
        ENDIF
        stTpStep:="X3c gave up, ERRNO "+NumToStr(ERRNO,0);
        RETURN;
    ENDPROC

    ! X7 (added 2026-09-26, for P3): the rig's own resolution. The World Zone
    ! DO is evaluated on a cycle, so the hit's distance from the face depends
    ! on where in that cycle the TCP crosses it - P0 always started exactly
    ! 50 mm away and saw one phase only. Twelve searches with the face moved
    ! 0.03 mm further each time sweep the phase; nTpDepth{k} = face - hit.
    PROC TG_TpX7()
        VAR num k;
        stTpStep:="X7 start";
        TpPrepare;
        FOR k FROM 1 TO 12 DO
            stTpStep:="X7 search "+NumToStr(k,0);
            TpZoneOn nTpFaceZ-0.03*(k-1);
            TpSearchWorld vTpSearch;
            nTpDepth{k}:=(nTpFaceZ-0.03*(k-1))-pTpHit.trans.z;
        ENDFOR
        TpZoneOff;
        stTpStep:="X7 done";
    ERROR
        nTpErrno:=ERRNO;
        stTpStep:=stTpStep+" | ERRNO "+NumToStr(ERRNO,0);
        RETURN;
    ENDPROC

    ! Back to PM's safe pose before PM restarts (TG_UfmecProbe lesson: PM's
    ! EE_START GoSafe otherwise waits on a pendant dialog).
    PROC TG_TpHome()
        VAR jointtarget jt;
        stTpStep:="home start";
        TpZoneOff;
        jt:=CJointT();
        jt.robax:=[0,0,0,0,30,0];
        MoveAbsJ jt,v200,fine,tTpTorch;
        stTpStep:="home done";
    ENDPROC
ENDMODULE
