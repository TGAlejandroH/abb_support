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
        ! The HMI put the module file into HOME:/TGS/ during
        ! TG_ReqFileTransfer (real cell: FTP; prototype: file copy into the
        ! VC's HOME folder). Load it dynamically, late-bind the PROC
        ! (%string% calls the PROC whose name is in the string), unload it
        ! again so the next transfer can replace the file. \Dynamic also
        ! auto-unloads the module if PP is moved to main mid-run.
        VAR string sPath;
        sPath:="HOME:/TGS/"+stTG_ProgName+".mod";
        TPWrite "TG: loading "+sPath;
        Load \Dynamic,sPath;
        TPWrite "TG: calling program "+stTG_ProgName;
        %stTG_ProgName%;
        TPWrite "TG: program "+stTG_ProgName+" finished";
        UnLoad sPath;
    ERROR
        IF ERRNO=ERR_LOADED THEN
            ! A module of this name is already in the task (e.g. loaded
            ! manually during Phase 2 testing): skip the Load, run it as-is.
            TPWrite "TG WARN: module already loaded - using it";
            TRYNEXT;
        ELSEIF ERRNO=ERR_UNLOAD THEN
            ! Unload failed (module was the manually loaded one): keep going.
            TPWrite "TG WARN: could not unload "+sPath;
            TRYNEXT;
        ELSEIF ERRNO=ERR_REFUNKPRC THEN
            ! Module loaded but no PROC of that name / name wrong: report
            ! and end the cycle cleanly so the HMI is not left waiting.
            TPWrite "TG ERROR: no PROC named "+stTG_ProgName;
            stTG_SubName:="none";
            TG_ReqEnd;
            RETURN;
        ELSEIF ERRNO=ERR_IOERROR THEN
            ! File missing/unreadable in HOME:/TGS/.
            TPWrite "TG ERROR: cannot load "+sPath;
            stTG_SubName:="none";
            TG_ReqEnd;
            RETURN;
        ENDIF
        ! Anything else propagates to tgMainCycle, which resets the cycle.
    ENDPROC

ENDMODULE
