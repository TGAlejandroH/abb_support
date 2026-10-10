MODULE TG_SeqWrapProbe
    ! Bench probe (2026-09-25): the START cycle counter wraps before RAPID's
    ! num stops counting. Calls the DEPLOYED TG_HandshakeCom / TG_SendStart /
    ! TG_HandshakeDisc twice with nTG_CycleSeq preset to 7999999, on port 2002
    ! so the HMI's connect loop on 2001 never sees it. Expected wire: START
    ! 8000000, then START 1. Driven by seq_wrap_probe.py, which plays the
    ! client and restores nTG_HandshakePort / nTG_CycleSeq whatever happens.
    ! Never loaded on a production controller.

    PERS num nSwFirst:=0;
    PERS num nSwSecond:=0;
    PERS string stSwStep:="";

    PROC TG_SwWrap()
        stSwStep:="start";
        nSwFirst:=0;
        nSwSecond:=0;
        nTG_HandshakePort:=2002;
        nTG_CycleSeq:=7999999;
        TG_HandshakeCom;
        TG_SendStart;
        nSwFirst:=nTG_CycleSeq;
        TG_HandshakeDisc;
        TG_HandshakeCom;
        TG_SendStart;
        nSwSecond:=nTG_CycleSeq;
        TG_HandshakeDisc;
        nTG_HandshakePort:=2001;
        nTG_CycleSeq:=0;
        stSwStep:="done";
    ERROR
        nTG_HandshakePort:=2001;
        stSwStep:="error "+NumToStr(ERRNO,0);
        RETURN;
    ENDPROC
ENDMODULE
