MODULE TG_Main
    !***********************************************************************
    ! TGMAINKL equivalent for ABB - Phase 1 (comm core smoke test)
    !
    ! Production entry point: run main() from AUTO, exactly like the FANUC
    ! controller starting tgmain.ls -> TGMAINKL.
    !
    ! Phase 1 cycle:  disconnect -> accept HMI -> program selection ->
    !                 end request -> disconnect -> repeat.
    ! Phase 3 will insert: TG_ReqFileTransfer + Load \Dynamic of the .tgs
    ! module from HOME:/TGS/ + late-bound call (%stTG_ProgName%) + UnLoad.
    !
    ! Requires TG_Comms.sys loaded in the same task (T_ROB1).
    ! Design: docs/abb_port_plan_v1.md section 4.6.
    !***********************************************************************

    PROC main()
        TPWrite "TG: main started";
        ! Always start from a clean socket state (TGMAINKL line: SOCKET_DISC)
        TG_SocketDisc;
        WHILE TRUE DO
            tgMainCycle;
        ENDWHILE
    ENDPROC

    LOCAL PROC tgMainCycle()
        ! One HMI session: connect, serve one program selection, disconnect.
        TG_SocketCom;
        TG_ReqProgSel;
        IF nTG_ProgSel=1 THEN
            ! ---- Phase 3 ----
            ! TG_ReqFileTransfer;                  -> nTG_FtpStatus, stTG_ProgName
            ! IF nTG_FtpStatus=1 tgRunTgsProgram;  Load \Dynamic + %name% + UnLoad
            ! -----------------
            ! Phase 1 placeholder: end the cycle so the HMI sees the full
            ! FANUC choreography (prog-sel ... R_E ... disconnect).
            stTG_SubName:="none";
            TG_ReqEnd;
        ELSEIF nTG_ProgSel=2 THEN
            ! Camera calibration program: out of v1 scope (plan, phase 4).
            stTG_SubName:="none";
            TG_ReqEnd;
        ELSE
            TPWrite "TG: unknown program ID - ending cycle";
        ENDIF
        TG_SocketDisc;
    ERROR
        ! Any error in a cycle (HMI vanished, bad payload, socket reset):
        ! log it, drop the connection, abandon the cycle - main() reconnects.
        TPWrite "TG: cycle error, ERRNO = "\Num:=ERRNO;
        TG_SocketDisc;
        RETURN;
    ENDPROC

ENDMODULE
