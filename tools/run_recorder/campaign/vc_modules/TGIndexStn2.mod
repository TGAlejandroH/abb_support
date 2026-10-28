MODULE TGIndexStn2_Mod
    ! VC campaign helper (virtual controller only): present station 2 the way Production
    ! Manager does on a dispatch - its EE_INDEX hook EvIndexToStn2 (ActInterch1, IndexToStn2,
    ! DeactInterch1), then ActStn2 as EE_PRE_PART does. The robot goes to its rest posture
    ! first. Served by bench_cycle.py like any TG program; PM itself needs the part-run input,
    ! which RWS cannot set in AUTO.
    LOCAL CONST robjoint rjRest := [0,-30,40,0,90,0];

    PROC TGIndexStn2()
        VAR jointtarget jt;
        jt := CJointT();
        jt.robax := rjRest;
        MoveAbsJ jt, v500, fine, tool0;
        IF IsMechUnitActive(STN1) DeactStn1;
        IF IsMechUnitActive(STN2) DeactStn2;
        EvIndexToStn2;
        ActStn2;
        TG_ReqEnd \Tool:=tool0 \WObj:=wobj0;
    ENDPROC
ENDMODULE
