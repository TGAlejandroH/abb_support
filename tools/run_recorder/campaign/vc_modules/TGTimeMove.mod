MODULE TGTimeMove_Mod
    ! VC campaign helper (virtual controller only): time one plain 100 mm MoveL at v100 with the
    ! controller's own clock and log it (ErrWrite \I), so the controller's time can be compared
    ! with the PC's clock for the same move (is the VC running slower than real time?), then
    ! 20 mm at a welding speed (4.233 mm/s = 10 IPM), timed the same way.
    LOCAL CONST robjoint rjRest := [0,-30,40,0,90,0];
    LOCAL CONST speeddata vWeld10ipm := [4.233,500,5000,1000];

    PROC TGTimeMove()
        VAR jointtarget jt;
        VAR robtarget p;
        VAR clock c;
        VAR num t;
        TG_ActMechUnit;
        jt := CJointT();
        jt.robax := rjRest;
        MoveAbsJ jt, v500, fine, tool0;
        p := CRobT(\Tool:=tool0 \WObj:=wobj0);
        ClkReset c;
        ClkStart c;
        MoveL Offs(p,0,100,0), v100, fine, tool0;
        ClkStop c;
        t := ClkRead(c \HighRes);
        ErrWrite \I, "TG bench move timing", "100 mm at v100 took " + NumToStr(t,3) + " s";
        MoveL p, v100, fine, tool0;
        ClkReset c;
        ClkStart c;
        MoveL Offs(p,0,20,0), vWeld10ipm, fine, tool0;
        ClkStop c;
        t := ClkRead(c \HighRes);
        ErrWrite \I, "TG bench move timing", "20 mm at 4.233 mm/s took " + NumToStr(t,3) + " s";
        MoveL p, v100, fine, tool0;
        TG_ReqEnd \Tool:=tool0 \WObj:=wobj0;
    ENDPROC
ENDMODULE
