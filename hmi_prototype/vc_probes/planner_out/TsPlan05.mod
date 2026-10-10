MODULE TsPlan05_Mod

    LOCAL CONST speeddata vTG001 := [775.0,500.0,5000.0,1000.0];
    LOCAL CONST speeddata vTG002 := [8.9,500.0,5000.0,1000.0];
    LOCAL CONST zonedata zTG001 := [FALSE,25.0,37.5,37.5,3.75,37.5,3.75];

    LOCAL CONST robtarget pTG0001 := [[50.000,-30.000,40.000],[0.353554,0.612372,0.612372,-0.353554],[0,0,0,0],[9E9,0.000,45.000,9E9,9E9,9E9]];
    LOCAL CONST robtarget pTG0002 := [[-30.000,0.000,40.000],[0.353554,0.612372,0.612372,-0.353554],[0,0,0,0],[9E9,0.000,45.000,9E9,9E9,9E9]];
    LOCAL CONST robtarget pTG0003 := [[-30.000,0.000,-20.000],[0.353554,0.612372,0.612372,-0.353554],[0,0,0,0],[9E9,0.000,45.000,9E9,9E9,9E9]];
    LOCAL CONST robtarget pTG0004 := [[-30.000,0.000,-20.000],[0.353554,0.612372,0.612372,-0.353554],[-1,0,-1,0],[9E9,0.000,45.000,9E9,9E9,9E9]];
    LOCAL CONST robtarget pTG0005 := [[0.000,0.000,-20.000],[0.353554,0.612372,0.612372,-0.353554],[-1,0,-1,0],[9E9,0.000,45.000,9E9,9E9,9E9]];
    LOCAL CONST robtarget pTG0006 := [[50.000,20.000,30.000],[0.353554,0.612372,0.612372,-0.353554],[0,0,0,0],[9E9,0.000,45.000,9E9,9E9,9E9]];
    LOCAL CONST robtarget pTG0007 := [[50.000,20.000,30.000],[0.353554,0.612372,0.612372,-0.353554],[0,-1,0,0],[9E9,0.000,45.000,9E9,9E9,9E9]];
    LOCAL CONST robtarget pTG0008 := [[50.000,20.000,0.000],[0.353554,0.612372,0.612372,-0.353554],[0,-1,0,0],[9E9,0.000,45.000,9E9,9E9,9E9]];
    LOCAL CONST robtarget pTG0009 := [[50.000,-30.000,5.000],[0.353554,0.612372,0.612372,-0.353554],[0,-1,0,0],[9E9,0.000,45.000,9E9,9E9,9E9]];
    LOCAL CONST robtarget pTG0010 := [[50.000,30.000,5.000],[0.353554,0.612372,0.612372,-0.353554],[0,-1,0,0],[9E9,0.000,45.000,9E9,9E9,9E9]];
    LOCAL CONST robtarget pTG0011 := [[50.000,30.000,40.000],[0.353554,0.612372,0.612372,-0.353554],[0,-1,0,0],[9E9,0.000,45.000,9E9,9E9,9E9]];
    LOCAL CONST jointtarget jtTG0001 := [[0.000,0.000,0.000,0.000,30.000,0.000],[9E9,0.000,45.000,9E9,9E9,9E9]];

    PROC TsPlan05()
        wobjTG_Weld.uframe:=[[0,0,0],[1,0,0,0]];
        wobjTG_Weld.oframe:=[[1600.000,0.000,1450.000],[0.707107,0.000000,0.000000,0.707107]];
        TG_ActMechUnit;
        ConfL \Off;
        ConfJ \Off;
        stTG_ProgPass:="TsPlan05";
        stTG_RobStatus:="Ok";
        stTG_SubName:="none";
        TG_ReqPassCheck \Tool:=tTG_Weld \WObj:=wobj0;
        IF nTG_PassOK<>1 THEN
        RETURN;
        ENDIF
        TG_DryRunOff;
        IF nTG_DryRun=1 TG_DryRunOn;
        TG_WeldPrep;
        MoveAbsJ jtTG0001, vmax, fine, tTG_Weld;
        ! TouchSense for Weld2
        stTG_SubName:="TSPWeld2_full";
        TG_ReqWeldFrame \Tool:=tTG_Weld \WObj:=wobjTG_Weld;
        IF nTG_WeldStatus=2 GOTO abort_end;
        TG_ReqTouchSenseDo;
        TG_CamClose;
        IF nTG_DoTouchSense=1 THEN
        TG_TeachModeOn;
        ConfJ \Off;
        MoveJ pTG0001, vmax, zTG001, tTG_Weld \WObj:=wobjTG_Weld;
        MoveJ pTG0002, vmax, zTG001, tTG_Weld \WObj:=wobjTG_Weld;
        MoveJ pTG0003, vmax, fine, tTG_Weld \WObj:=wobjTG_Weld;
        IF Distance(CPos(\Tool:=tTG_Weld \WObj:=wobjTG_Weld),pTG0004.trans)>100 THEN
        TPWrite "TG ERROR: weld frame check failed - torch far from chain entry";
        GOTO abort_end;
        ENDIF
        ConfL \On;
        MoveL pTG0004, vTG001, fine, tTG_Weld \WObj:=wobjTG_Weld;
        TG_TouchSearch pTG0004, pTG0005, tTG_Weld, wobjTG_Weld;
        TG_ReqTouchSensePoint;
        WaitTime 0.50;
        MoveL pTG0004, vTG001, fine, tTG_Weld \WObj:=wobjTG_Weld;
        ConfJ \Off;
        MoveJ pTG0003, vmax, zTG001, tTG_Weld \WObj:=wobjTG_Weld;
        MoveJ pTG0002, vmax, zTG001, tTG_Weld \WObj:=wobjTG_Weld;
        MoveJ pTG0006, vmax, fine, tTG_Weld \WObj:=wobjTG_Weld;
        IF Distance(CPos(\Tool:=tTG_Weld \WObj:=wobjTG_Weld),pTG0007.trans)>100 THEN
        TPWrite "TG ERROR: weld frame check failed - torch far from chain entry";
        GOTO abort_end;
        ENDIF
        ConfL \On;
        MoveL pTG0007, vTG001, fine, tTG_Weld \WObj:=wobjTG_Weld;
        TG_TouchSearch pTG0007, pTG0008, tTG_Weld, wobjTG_Weld;
        TG_ReqTouchSensePoint;
        WaitTime 0.50;
        MoveL pTG0007, vTG001, fine, tTG_Weld \WObj:=wobjTG_Weld;
        ConfJ \Off;
        MoveJ pTG0006, vmax, zTG001, tTG_Weld \WObj:=wobjTG_Weld;
        MoveJ pTG0001, vmax, fine, tTG_Weld \WObj:=wobjTG_Weld;
        TG_ReqTouchSenseEnd \WObj:=wobjTG_Weld;
        ENDIF
        ! Start of Weld2
        stTG_SubName:="PWeld2";
        TG_ReqWeldFrame \Tool:=tTG_Weld \WObj:=wobjTG_Weld;
        TG_CamClose;
        IF nTG_WeldStatus=2 GOTO abort_end;
        IF nTG_WeldStatus=1 THEN
        ConfJ \Off;
        MoveJ pTG0001, vmax, fine, tTG_Weld \WObj:=wobjTG_Weld;
        TG_ReqWeldParams;
        TG_ApplyWeldParams 1;
        IF Distance(CPos(\Tool:=tTG_Weld \WObj:=wobjTG_Weld),pTG0009.trans)>100 THEN
        TPWrite "TG ERROR: weld frame check failed - torch far from chain entry";
        GOTO abort_end;
        ENDIF
        ConfL \On;
        ArcLStart pTG0009, vTG001, sdTG_Weld, wdTG_Weld, fine, tTG_Weld \WObj:=wobjTG_Weld;
        ArcLEnd pTG0010, vTG002, sdTG_Weld, wdTG_Weld, fine, tTG_Weld \WObj:=wobjTG_Weld;
        ! process end
        MoveL pTG0011, vTG001, fine, tTG_Weld \WObj:=wobjTG_Weld;
        ENDIF
        MoveAbsJ jtTG0001, vmax, fine, tTG_Weld;
        TG_CamClose;
        abort_end:
        stTG_SubName:="none";
        TG_ReqEnd \Tool:=tTG_Weld \WObj:=wobj0;
    ENDPROC

ENDMODULE
