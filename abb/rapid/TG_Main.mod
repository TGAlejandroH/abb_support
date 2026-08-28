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
        ! Mirrors TGMAINKL: prog-sel -> file transfer -> run the .tgs program.
        TG_SocketCom;
        TG_ReqProgSel;
        IF nTG_ProgSel=1 THEN
            TG_ReqFileTransfer;
            IF nTG_FtpStatus=1 THEN
                tgRunTgsProgram;
            ELSE
                ! FANUC: 'ErrCode 1001. Problem when copying/loading...'
                TPWrite "TG ERROR: file transfer failed - skipping program";
            ENDIF
        ELSEIF nTG_ProgSel=2 THEN
            ! Camera calibration program: out of v1 scope (plan, phase 4).
            TPWrite "TG: camera calibration not implemented - ending cycle";
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

    LOCAL PROC tgRunTgsProgram()
        ! Run the .tgs program named by the HMI (FANUC: CALL_PROG(prog_name)).
        ! Late binding: %string% calls the PROC whose name is in the string.
        ! Phase 2: the .tgs module (e.g. abb/rapid/TGS/TD05Test.mod) must be
        ! loaded manually in RobotStudio.
        ! ---- Phase 3 ----
        ! Load \Dynamic,"HOME:/TGS/"+stTG_ProgName+".mod";
        ! ... call ...
        ! UnLoad "HOME:/TGS/"+stTG_ProgName+".mod";
        ! -----------------
        TPWrite "TG: calling program "+stTG_ProgName;
        %stTG_ProgName%;
        TPWrite "TG: program "+stTG_ProgName+" finished";
    ERROR
        IF ERRNO=ERR_REFUNKPRC THEN
            ! Program not loaded / name wrong: report and end the cycle
            ! cleanly so the HMI is not left waiting mid-protocol.
            TPWrite "TG ERROR: no PROC named "+stTG_ProgName;
            stTG_SubName:="none";
            TG_ReqEnd;
            RETURN;
        ENDIF
    ENDPROC

ENDMODULE
