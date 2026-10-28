MODULE TGHomeStn_Mod
    ! VC campaign helper (virtual controller only): bring the presented station's positioner
    ! back to tilt 0 / chuck 0 and the robot to its rest posture, after a run that ended at
    ! abort_end with the positioner turned. Served by bench_cycle.py like any TG program.
    LOCAL CONST jointtarget jtHome := [[0,-30,40,0,90,0],[0,0,0,9E9,9E9,9E9]];

    PROC TGHomeStn()
        VAR jointtarget jt;
        TG_ActMechUnit;
        jt := CJointT();
        jt.robax := jtHome.robax;
        MoveAbsJ jt, v500, fine, tool0;
        MoveAbsJ jtHome, v500, fine, tool0;
        TG_ReqEnd \Tool:=tool0 \WObj:=wobj0;
    ENDPROC
ENDMODULE
