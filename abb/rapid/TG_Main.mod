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

    ! FIX 2026-08-28 (error-recovery matrix F-B/I3): tracks whether a .tgs
    ! module is currently loaded, so recovery paths can unload it. An
    ! aborted cycle would otherwise leave a stale module in the task that
    ! shadows the next cycle's freshly transferred file.
    LOCAL VAR bool tg_module_loaded:=FALSE;

    PROC main()
        TPWrite "TG: main started";
        ! FIX 2026-08-28 (F-F): main is also the ExitCycle recovery target
        ! (tgCycleAbort in TG_Comms). VC-observed: ExitCycle does NOT drop
        ! \Dynamic modules, so a stale module may still be loaded here; the
        ! flag still resets - the ERR_LOADED reload path (tgRunTgsProgram)
        ! handles the leftover on the next Load, VC-validated 2026-08-28.
        tg_module_loaded:=FALSE;
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
            ! No program context here: report torch/base (HMI ignores R_E
            ! poses, plan 1.4.1). Explicit args keep the DEPRECATED modal
            ! fallback dormant (plan 7.6).
            TG_ReqEnd \Tool:=tTG_Weld \WObj:=wobj0;
        ELSE
            TPWrite "TG: unknown program ID - ending cycle";
        ENDIF
        TG_SocketDisc;
    ERROR
        ! Any error in a cycle (HMI vanished, bad payload, socket reset):
        ! log it, drop the connection, abandon the cycle - main() reconnects.
        TPWrite "TG: cycle error, ERRNO = "\Num:=ERRNO;
        ! FIX 2026-08-28 (error-recovery matrix F-B/I3): belt-and-braces -
        ! if a module is somehow still loaded when a cycle dies, unload it
        ! so it cannot shadow the next cycle's freshly transferred file.
        ! (tgRunTgsProgram's own handler is the primary cleanup.)
        IF tg_module_loaded tgTryUnload "HOME:/TGS/"+stTG_ProgName+".mod";
        TG_SocketDisc;
        RETURN;
    ENDPROC

    LOCAL PROC tgTryUnload(string sPath)
        ! FIX 2026-08-28 (error-recovery matrix F-B/I2+I3): best-effort
        ! unload for recovery paths, where a failed UnLoad must not kill
        ! the cycle. Swallows any unload error (module not loaded, or
        ! loaded from RobotStudio rather than by Load) with a warning.
        UnLoad sPath;
        tg_module_loaded:=FALSE;
    ERROR
        TPWrite "TG WARN: could not unload "+sPath;
        ! TRYNEXT skips the failed UnLoad; the flag still clears, which is
        ! intended - retrying a hopeless unload on a later cycle would
        ! only repeat this warning (Load's ERR_LOADED path covers it).
        TRYNEXT;
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
        VAR bool bRetriedLoad:=FALSE;
        sPath:="HOME:/TGS/"+stTG_ProgName+".mod";
        TPWrite "TG: loading "+sPath;
        Load \Dynamic,sPath;
        tg_module_loaded:=TRUE;
        TPWrite "TG: calling program "+stTG_ProgName;
        %stTG_ProgName%;
        TPWrite "TG: program "+stTG_ProgName+" finished";
        UnLoad sPath;
        tg_module_loaded:=FALSE;
    ERROR
        IF ERRNO=ERR_LOADED THEN
            IF bRetriedLoad THEN
                ! The unload attempt failed and the module is still in the
                ! task (e.g. loaded from RobotStudio, not by Load): run it
                ! as-is - the pre-fix behavior, kept as a bounded fallback
                ! so Load cannot loop. TRYNEXT continues after Load, which
                ! also sets tg_module_loaded.
                TPWrite "TG WARN: module already loaded - using it";
                TRYNEXT;
            ELSE
                ! FIX 2026-08-28 (error-recovery matrix F-B/I2). A module
                ! of this name is already in the task: a leftover from an
                ! aborted cycle (UnLoad never ran) or a manual Phase-2-
                ! style load. The old behavior (run it as-is) could
                ! execute a STALE version while the freshly transferred
                ! file sits in HOME:/TGS/ - the HMI's FTP compare keeps
                ! the FILE fresh but cannot unload the MODULE. Unload and
                ! re-Load so the file just transferred is what runs.
                bRetriedLoad:=TRUE;
                TPWrite "TG WARN: module already loaded - reloading from file";
                tgTryUnload sPath;
                RETRY;
            ENDIF
        ELSEIF ERRNO=ERR_UNLOAD THEN
            ! The end-of-run unload failed (module was the manually loaded
            ! one): keep going.
            TPWrite "TG WARN: could not unload "+sPath;
            TRYNEXT;
        ELSEIF ERRNO=ERR_REFUNKPRC THEN
            ! Module loaded but no PROC of that name / name wrong: report
            ! and end the cycle cleanly so the HMI is not left waiting.
            TPWrite "TG ERROR: no PROC named "+stTG_ProgName;
            ! FIX 2026-08-28 (F-B/I3): the module IS loaded here - unload
            ! before leaving, or it shadows the next cycle's file.
            tgTryUnload sPath;
            stTG_SubName:="none";
            TG_ReqEnd \Tool:=tTG_Weld \WObj:=wobj0;
            RETURN;
        ELSEIF ERRNO=ERR_IOERROR THEN
            ! File missing/unreadable in HOME:/TGS/ (nothing was loaded).
            TPWrite "TG ERROR: cannot load "+sPath;
            stTG_SubName:="none";
            TG_ReqEnd \Tool:=tTG_Weld \WObj:=wobj0;
            RETURN;
        ENDIF
        ! FIX 2026-08-28 (error-recovery matrix F-B/I3, F-E). Any other
        ! error IN THIS PROC's OWN FRAME (Load/UnLoad/the %% call itself):
        ! unload the module before abandoning the cycle, then RAISE so
        ! tgMainCycle's handler logs it and resets the sockets. The RAISE
        ! must be explicit - a RAPID error handler that runs to its end
        ! acts as RETURN, not RAISE (F-E).
        ! NOTE (F-F, VC-observed 2026-08-28): wire errors raised INSIDE the
        ! late-bound .tgs run never reach this handler - unhandled errors
        ! do not propagate through the %% call; the program would stop at
        ! the failing instruction. Those are recovered at the source by
        ! tgCycleAbort in TG_Comms (ExitCycle to main).
        tgTryUnload sPath;
        RAISE;
    ENDPROC

ENDMODULE
