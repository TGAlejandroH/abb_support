MODULE TG_P5Survey
    ! VC-ONLY PROBE 2026-09-26 - NOT FOR PRODUCTION, never load on a real cell.
    !
    ! Touch-sense P5 (docs/abb_touch_sense_port_v1.md section 6): the Weld
    ! Planner export needs, for every pose of TGS/TD05Touch.mod's part, the
    ! joint solution the robot actually takes there (the planner writes
    ! MoveAbsJ postures and derives confdata from joints) and the positioner
    ! axes of the station at the robot (the planner writes them into every
    ! robtarget, touch-sense D6). This measures both: TG_P5Run drives the
    ! robot to each pose in TD05Touch's own order (ConfJ/ConfL off, as
    ! TD05Touch runs), settles, and records CJointT() in jtP5*.
    !
    ! The poses are TD05Touch's, in the frame the HMI serves for it
    ! ([1600,0,1450 | Rz 90], wobjP5 below). No World Zone is active while
    ! this runs, so the contact poses are simply points in space.
    ! Every global carries "P5" (finding F-5).

    PERS jointtarget jtP5Home:=[[0,0,0,0,30,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS jointtarget jtP5Via:=[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS jointtarget jtP5A1:=[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS jointtarget jtP5C1:=[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS jointtarget jtP5A2:=[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS jointtarget jtP5C2:=[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS jointtarget jtP5WA:=[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS jointtarget jtP5WS:=[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS jointtarget jtP5WE:=[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS jointtarget jtP5WD:=[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS num nP5Done:=0;

    LOCAL PERS wobjdata wobjP5:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[1600,0,1450],[0.707107,0,0,0.707107]]];
    LOCAL CONST orient qP5:=[0.353554,0.612372,0.612372,-0.353554];
    LOCAL CONST speeddata vP5:=[200,500,5000,1000];

    LOCAL FUNC robtarget p5At(pos p)
        ! A target at part position p with the station's CURRENT axes, so the
        ! positioner stands still (TD05Touch's tsAt).
        VAR robtarget r;
        VAR jointtarget jt;
        jt:=CJointT();
        r:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
        r.trans:=p;
        r.rot:=qP5;
        r.extax:=jt.extax;
        RETURN r;
    ENDFUNC

    LOCAL FUNC jointtarget p5Settled()
        ! A fine move has ended; CJointT right after it can still read the
        ! last servo sample (P0, X2), so wait for standstill first.
        WaitRob\InPos;
        WaitRob\ZeroSpeed;
        WaitTime 0.2;
        RETURN CJointT();
    ENDFUNC

    PROC TG_P5Run()
        VAR jointtarget jt;
        nP5Done:=0;
        ! The station at the robot, active - as the TG cycle's handshake leaves it.
        TG_ActMechUnit;
        ConfJ\Off;
        ConfL\Off;
        jt:=CJointT();
        jt.robax:=[0,0,0,0,30,0];
        MoveAbsJ jt,v200,fine,tTG_Weld;
        jtP5Home:=p5Settled();
        MoveJ p5At([-30,0,40]),vP5,fine,tTG_Weld\WObj:=wobjP5;
        jtP5Via:=p5Settled();
        MoveL p5At([-30,0,-20]),vP5,fine,tTG_Weld\WObj:=wobjP5;
        jtP5A1:=p5Settled();
        MoveL p5At([0,0,-20]),vP5,fine,tTG_Weld\WObj:=wobjP5;
        jtP5C1:=p5Settled();
        MoveL p5At([-30,0,-20]),vP5,fine,tTG_Weld\WObj:=wobjP5;
        MoveL p5At([-30,0,40]),vP5,fine,tTG_Weld\WObj:=wobjP5;
        MoveL p5At([50,20,30]),vP5,fine,tTG_Weld\WObj:=wobjP5;
        jtP5A2:=p5Settled();
        MoveL p5At([50,20,0]),vP5,fine,tTG_Weld\WObj:=wobjP5;
        jtP5C2:=p5Settled();
        MoveL p5At([50,20,30]),vP5,fine,tTG_Weld\WObj:=wobjP5;
        MoveJ p5At([50,-30,40]),vP5,fine,tTG_Weld\WObj:=wobjP5;
        jtP5WA:=p5Settled();
        MoveL p5At([50,-30,5]),vP5,fine,tTG_Weld\WObj:=wobjP5;
        jtP5WS:=p5Settled();
        MoveL p5At([50,30,5]),vP5,fine,tTG_Weld\WObj:=wobjP5;
        jtP5WE:=p5Settled();
        MoveL p5At([50,30,40]),vP5,fine,tTG_Weld\WObj:=wobjP5;
        jtP5WD:=p5Settled();
        jt:=CJointT();
        jt.robax:=[0,0,0,0,30,0];
        MoveAbsJ jt,v200,fine,tTG_Weld;
        ConfJ\On;
        ConfL\On;
        nP5Done:=1;
    ENDPROC
ENDMODULE
