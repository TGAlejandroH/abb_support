MODULE TG_SocketProbe_Mod
    !***********************************************************************
    ! TG_SocketProbe - minimal TCP echo server for commissioning the
    ! PC <-> IRC5 link on a REAL controller (tools/comms_check/README.md).
    !
    ! Standalone on purpose: no TG_Comms, no TG_Main, no main, no motion,
    ! no I/O, no file writes. Load it, PP to routine TG_SocketProbe, Start.
    ! Unload it when done. Nothing else on the controller is touched.
    !
    ! What a run proves, in order:
    !   1. Option 616-1 PC Interface is installed. Without it this module
    !      does not even LOAD: the Socket* instructions are unknown, and the
    !      event log names them.
    !   2. The controller can bind a TCP listener on stTG_ProbeIP:nTG_ProbePort.
    !      A bind error here means the IP below is not one of the
    !      controller's own addresses.
    !   3. The PC reaches the listener, and payload flows both ways: every
    !      message is echoed back as "TG_ECHO <message>". Sending "QUIT"
    !      gets "TG_BYE" and ends the routine.
    !
    ! ===== THE ONE LINE TO EDIT BEFORE LOADING ===========================
    ! stTG_ProbeIP must be the CONTROLLER'S OWN IP on the port the PC is
    ! plugged into:
    !   service port  -> 192.168.125.1 (the default below, fixed by ABB)
    !   WAN port (X6) -> the address MONARC IT assigned to the controller
    ! 127.0.0.1 works ONLY on a virtual controller (the PC and the
    ! controller are the same machine there) - never on a real IRC5.
    ! The value can also be changed AFTER loading, without re-uploading:
    ! FlexPendant -> Program Data -> string -> stTG_ProbeIP, or from the PC
    ! with "comms_probe.py setip <ip>" (needs AUTO; see README).
    ! =====================================================================
    PERS string stTG_ProbeIP:="127.0.0.1"; ! "192.168.125.1";
    PERS num nTG_ProbePort:=2000;

    ! Per-connection wait for the PC, in seconds. In MANUAL the enabling
    ! device must stay pressed the whole time the routine runs, so this is
    ! deliberately short: the PC side connects within a few seconds.
    LOCAL CONST num ACCEPT_TIMEOUT:=90;
    LOCAL CONST num RECV_TIMEOUT:=30;
    ! Safety net so the loop can never run unattended for long.
    LOCAL CONST num MAX_CLIENTS:=20;

    LOCAL VAR socketdev srvProbe;
    LOCAL VAR socketdev cliProbe;
    LOCAL VAR string stClientIP:="";
    LOCAL VAR num nServed:=0;
    LOCAL VAR bool bQuit:=FALSE;
    LOCAL VAR bool bAccepted:=FALSE;

    !***********************************************************************
    ! Entry point. Run with "PP to Routine". Ends by itself on QUIT, on
    ! MAX_CLIENTS, on ACCEPT_TIMEOUT with no client, or on any error.
    !***********************************************************************
    PROC TG_SocketProbe()
        nServed:=0;
        bQuit:=FALSE;
        TPWrite "TG PROBE: bind "+stTG_ProbeIP+":"+NumToStr(nTG_ProbePort,0);
        ! Close-before-create: safe on sockets that were never opened
        ! (same idiom as TG_Comms and the RoboDK driver).
        SocketClose srvProbe;
        SocketClose cliProbe;
        SocketCreate srvProbe;
        SocketBind srvProbe,stTG_ProbeIP,nTG_ProbePort;
        SocketListen srvProbe;
        TPWrite "TG PROBE: listening - run comms_probe.py socket on the PC";
        WHILE (NOT bQuit) AND (nServed<MAX_CLIENTS) DO
            pServeOne;
        ENDWHILE
        SocketClose srvProbe;
        SocketClose cliProbe;
        TPWrite "TG PROBE: done, connections served = "+NumToStr(nServed,0);
    ERROR
        TPWrite "TG PROBE: SETUP ERROR, ERRNO = "+NumToStr(ERRNO,0);
        TPWrite "TG PROBE: bind failed? IP must be this controller's own";
        SocketClose srvProbe;
        SocketClose cliProbe;
        RETURN;
    ENDPROC

    !***********************************************************************
    ! One accepted connection: receive one message, answer, close.
    !***********************************************************************
    LOCAL PROC pServeOne()
        VAR string stMsg;
        bAccepted:=FALSE;
        SocketAccept srvProbe,cliProbe\ClientAddress:=stClientIP,\Time:=ACCEPT_TIMEOUT;
        bAccepted:=TRUE;
        TPWrite "TG PROBE: client connected from "+stClientIP;
        SocketReceive cliProbe,\Str:=stMsg,\Time:=RECV_TIMEOUT;
        TPWrite "TG PROBE: received '"+stMsg+"'";
        IF stMsg="QUIT" THEN
            SocketSend cliProbe,\Str:="TG_BYE";
            bQuit:=TRUE;
        ELSE
            SocketSend cliProbe,\Str:="TG_ECHO "+stMsg;
        ENDIF
        nServed:=nServed+1;
        SocketClose cliProbe;
    ERROR
        IF ERRNO=ERR_SOCK_TIMEOUT THEN
            IF bAccepted THEN
                TPWrite "TG PROBE: client sent nothing within "+NumToStr(RECV_TIMEOUT,0)+" s";
            ELSE
                TPWrite "TG PROBE: no client within "+NumToStr(ACCEPT_TIMEOUT,0)+" s - stopping";
                bQuit:=TRUE;
            ENDIF
        ELSEIF ERRNO=ERR_SOCK_CLOSED THEN
            TPWrite "TG PROBE: client closed early - waiting for the next";
        ELSE
            TPWrite "TG PROBE: SERVE ERROR, ERRNO = "+NumToStr(ERRNO,0);
            bQuit:=TRUE;
        ENDIF
        SocketClose cliProbe;
        RETURN;
    ENDPROC

ENDMODULE
