MODULE TGFsProbe_Mod
    !***********************************************************************
    ! TGFsProbe - standalone VC probe for the R_F_T "free memory" value.
    !
    ! v2 (2026-09-04): the v1 run died on its first call and produced no
    ! data. Two things were learned and both are built into this version.
    !   FINDING A: a bare FSSize(path) with no unit switch is REJECTED -
    !     40764 "The instruction FSSize must be used with one switch
    !     argument. Use one of the switch Total or Free." So one of
    !     \Total / \Free is MANDATORY, even though the manual's syntax
    !     (2.71) prints both as optional: "FSSize (Name [\Total] | [\Free]
    !     [\Kbyte] [\Mbyte])". The bare probe is gone.
    !   FINDING B: that failure came back as 40223, an UNRECOVERABLE
    !     execution error - "no error recovery attempt by an error handler
    !     routine was allowed". An ERROR handler is therefore NOT a
    !     guarantee that a probe cannot halt the whole run. Hence the
    !     ordering rule below.
    !
    ! ORDERING RULE (v2): every probe that might raise runs AFTER the ones
    ! that answer the design questions, so an unrecoverable halt can only
    ! cost the last answer, never all of them. \Free \Mbyte is expected to
    ! be the safe unit, so it goes first and the overflow discriminator
    ! (TGFsProbeLimit) goes dead last.
    !
    ! WHY ANY OF THIS: TG_ReqFileTransfer still answers the HMI with the
    ! dummy constant "99999999". The HMI parses that with std::stoull and
    ! aborts the weld session if it is below 40000 BYTES (config.json,
    ! HMI_MINIMUM_STORAGE_IN_TPP_CONTROLLER_MEMORY_IN_BYTES). FANUC produced
    ! it with VOL_SPACE of tpp: = free bytes of the TP program device. On
    ! ABB that ONE FANUC resource is TWO: the disk the .mod file is PUT to
    ! (HOME:) and the RAPID program memory it is then Load-ed into. This
    ! module measures both, plus the three things the manuals do not answer.
    !
    ! Documented facts being tested against (3HAC050917-001 rev F):
    !   2.71  FSSize(Name [\Total]|[\Free] [\Kbyte] [\Mbyte]) -> num, bytes
    !         ERR_FILEACC  "The file system does not exist"
    !         ERR_FILESIZE "The size exceeds the max integer value for a
    !                       num, 8388608"
    !   2.136 ProgMemFree() -> num, free program memory in bytes. No
    !         arguments and no documented error handling at all.
    !   3.49  num stores exact integers only in -8388607 .. 8388608.
    !
    ! QUESTION 1 (steps 1 and 5): is the ERR_FILESIZE ceiling applied to the
    !   RAW byte count, or to the value AFTER the \Kbyte / \Mbyte division?
    !   If it is applied to the raw bytes then FSSize cannot report this
    !   VC's HOME: at ANY unit and ProgMemFree has to carry the answer
    !   alone. This VC's HOME: is D:\ABB\...\Virtual Controllers\
    !   Controller1\HOME and D: had ~290 GB free of ~932 GB, so the EXPECTED
    !   outcome is: step 1 returns ~290000 MB and ~932000 MB, and step 5
    !   raises on both \Kbyte (2.97e8 KB is far past 8388608) and bytes.
    !   Anything else changes the design. Step 5 also tells us whether
    !   ERR_FILESIZE is recoverable (handler prints) or, like finding B,
    !   halts the task - which decides whether production code can rely on
    !   an error handler around FSSize at all.
    !
    ! QUESTION 2 (step 2): what is a legal Name? The kernel does a statfs()
    !   on it (kernel trace string RLFSTA_FSSIZE_STATFS_REQ), so it most
    !   likely has to EXIST - yet the manual's own example passes
    !   "HOME:/spy.log", which usually does not exist. Step 2 tries a
    !   directory, the bare device, a real file and a missing file, and asks
    !   IsFile the same question first, so an ERR_FILEACC can be blamed on
    !   the path rather than on the device. The missing path is last, for
    !   the same reason the risky units are last.
    !
    ! QUESTION 3 (steps 3-4): what does ProgMemFree() return on a VC, where
    !   program memory is simulated - and does the byte value survive
    !   formatting as PLAIN DIGITS? std::stoull reads "3.04E+11" as 3, so an
    !   exponent anywhere in the wire string is a silent session-killer.
    !   Step 3 prints the num route and the dnum route side by side.
    !
    ! Deliberately standalone: no dependency on TG_Comms.sys / TG_Cell.sys /
    ! TG_Main.mod, no motion, no sockets, and nothing is written to disk or
    ! to any PERS. Safe to run with no program loaded.
    !
    ! HOW TO RUN
    !   1. RAPID tab -> load this module into T_ROB1.
    !   2. PP to Routine -> TGFsProbeAll, run to completion (manual is fine).
    !   3. Paste the whole Operator window output back - INCLUDING the run
    !      that halts, if one does, plus the event log number, exactly as
    !      you did for v1. A halt is data, not a failed run.
    !   The five steps are independently runnable, in this order:
    !   TGFsProbeUnits, TGFsProbePaths, TGFsProbeProgMem, TGFsProbeWire,
    !   TGFsProbeLimit. If the run halts inside one of them, PP to the NEXT
    !   one and continue - everything before it has already printed.
    !
    ! IF THE MODULE FAILS TO COMPILE citing ERR_FILESIZE: that errnum is not
    ! predefined in this RobotWare after all. Delete the single line marked
    ! "FALLBACK" in pShowErr and re-run - the numeric ERRNO printed on the
    ! line above it answers Question 1 either way.
    !***********************************************************************

    ! ---------------------------- probe paths ----------------------------
    ! PATH_TGS is the real transfer target - TG_Main loads
    ! HOME:/TGS/<prog>.mod - so it is what production code would probe.
    LOCAL CONST string PATH_TGS:="HOME:/TGS";
    LOCAL CONST string PATH_DEV:="HOME:";
    LOCAL CONST string PATH_SLASH:="HOME:/";
    LOCAL CONST string PATH_FILE:="HOME:/user.sys";
    LOCAL CONST string PATH_MISS:="HOME:/TGS/no_such_file.mod";

    ! Declared as dnum CONSTs on purpose: RAPID will not silently mix num
    ! and dnum operands, and a bare literal in a dnum expression is exactly
    ! the kind of thing that turns into a compile error at the worst moment.
    LOCAL CONST dnum BYTES_PER_MB:=1048576;
    LOCAL CONST dnum HMI_MIN:=40000;
    LOCAL CONST dnum PROBE_FAILED:=-1;

    !***********************************************************************
    ! Runs all five steps, safest first (see ORDERING RULE in the header).
    !***********************************************************************
    PROC TGFsProbeAll()
        TPWrite "";
        TPWrite "TG FS PROBE: ========== start ==========";
        TGFsProbeUnits;
        TGFsProbePaths;
        TGFsProbeProgMem;
        TGFsProbeWire;
        TGFsProbeLimit;
        TPWrite "TG FS PROBE: ========== done ===========";
    ENDPROC

    !***********************************************************************
    ! STEP 1 - the two units expected to WORK, so that a halt anywhere later
    ! still leaves the design with a usable number. Free is the value R_F_T
    ! needs; Total is a cross-check that HOME: really is the D: partition
    ! (D: total was ~932 GB, so ~932000 MB is the plausible answer).
    !***********************************************************************
    PROC TGFsProbeUnits()
        TPWrite "TG FS PROBE: -- step 1: safe units on HOME:/TGS --";
        pFreeM PATH_TGS;
        pTotalM PATH_TGS;
        TPWrite "TG FS PROBE: step 1 done";
    ENDPROC

    LOCAL PROC pFreeM(string p)
        VAR num v;
        v:=FSSize(p\Free\Mbyte);
        TPWrite "  Free Mbyte = "+NumToStr(v,0)+" MB";
    ERROR
        pShowErr "Free Mbyte",ERRNO;
        RETURN;
    ENDPROC

    LOCAL PROC pTotalM(string p)
        VAR num v;
        v:=FSSize(p\Total\Mbyte);
        TPWrite "  TotalMbyte = "+NumToStr(v,0)+" MB";
    ERROR
        pShowErr "TotalMbyte",ERRNO;
        RETURN;
    ENDPROC

    !***********************************************************************
    ! STEP 2 - Question 2. Which Name forms does FSSize accept? Held at
    ! \Free \Mbyte so a raise here means the PATH was rejected, not the
    ! unit. IsFile is asked first for the same reason.
    !***********************************************************************
    PROC TGFsProbePaths()
        TPWrite "TG FS PROBE: -- step 2: Name forms (Free,Mbyte) --";
        pPath PATH_TGS,"HOME:/TGS";
        pPath PATH_DEV,"HOME:";
        pPath PATH_SLASH,"HOME:/";
        pPath PATH_FILE,"HOME:/user.sys";
        pPath PATH_MISS,"missing file";
        TPWrite "TG FS PROBE: step 2 done";
    ENDPROC

    LOCAL PROC pPath(string p,string tag)
        VAR num v;
        pShowIsFile p,tag;
        v:=FSSize(p\Free\Mbyte);
        TPWrite "  "+tag+" -> "+NumToStr(v,0)+" MB";
    ERROR
        pShowErr tag+" FSSize",ERRNO;
        RETURN;
    ENDPROC

    LOCAL PROC pShowIsFile(string p,string tag)
        ! IsFile raises ERR_FILEACC when the path does not exist and a type
        ! switch is given (2.98), so the missing-file case is expected to
        ! land in the handler - that is the control for step 2.
        TPWrite "  "+tag+" isdir ="\Bool:=IsFile(p\Directory);
        TPWrite "  "+tag+" isreg ="\Bool:=IsFile(p\RegFile);
    ERROR
        pShowErr tag+" IsFile",ERRNO;
        RETURN;
    ENDPROC

    !***********************************************************************
    ! STEP 3 - Question 3, first half. ProgMemFree is the closer analogue of
    ! FANUC's tpp: (it is what Load consumes, and Load raises ERR_PRGMEMFULL
    ! when it runs out). It returns bytes as a num with no documented error
    ! path, so above 8388608 it should lose precision rather than raise -
    ! this step is what proves that. The two formatting routes are printed
    ! side by side: if NumToStr and the dnum route disagree, or if either
    ! shows an "E", the wire format must go through dnum.
    !***********************************************************************
    PROC TGFsProbeProgMem()
        VAR num n;
        TPWrite "TG FS PROBE: -- step 3: ProgMemFree --";
        n:=ProgMemFree();
        TPWrite "  raw (TPWrite Num) ="\Num:=n;
        TPWrite "  NumToStr(n,0)     = "+NumToStr(n,0);
        TPWrite "  via dnum          = "+DnumToStr(NumToDnum(n),0);
        TPWrite "TG FS PROBE: step 3 done";
    ERROR
        pShowErr "ProgMemFree",ERRNO;
        RETURN;
    ENDPROC

    !***********************************************************************
    ! STEP 4 - Question 3, second half. Builds the exact string R_F_T would
    ! put on the wire under the proposed design: min(disk free, program
    ! memory free), in bytes, via dnum, plain digits. Sends nothing.
    ! PASS: "WOULD SEND" is digits only - no ".", no "E", no "-" - and
    ! "above 40000" is TRUE.
    !***********************************************************************
    PROC TGFsProbeWire()
        VAR dnum dnDisk;
        VAR dnum dnProg;
        VAR dnum dnSend;
        VAR string s;
        TPWrite "TG FS PROBE: -- step 4: the wire string --";
        dnDisk:=fDiskFreeBytes(PATH_TGS);
        dnProg:=NumToDnum(ProgMemFree());
        TPWrite "  disk bytes = "+DnumToStr(dnDisk,0);
        TPWrite "  prog bytes = "+DnumToStr(dnProg,0);
        IF dnDisk=PROBE_FAILED THEN
            ! Step 1 already said why; here it only decides the fallback.
            dnSend:=dnProg;
            TPWrite "  disk probe failed - program memory alone";
        ELSEIF dnDisk<dnProg THEN
            dnSend:=dnDisk;
            TPWrite "  disk is the binding constraint";
        ELSE
            dnSend:=dnProg;
            TPWrite "  program memory is the binding constraint";
        ENDIF
        s:=DnumToStr(dnSend,0);
        TPWrite "  WOULD SEND = "+s;
        TPWrite "  length     ="\Num:=StrLen(s);
        TPWrite "  above 40000 ="\Bool:=(dnSend>HMI_MIN);
        TPWrite "TG FS PROBE: step 4 done";
    ENDPROC

    LOCAL FUNC dnum fDiskFreeBytes(string p)
        ! MB -> bytes in dnum: num is exact only to 8388608 (3.49), and the
        ! byte count is far past that on any real file system.
        RETURN NumToDnum(FSSize(p\Free\Mbyte))*BYTES_PER_MB;
    ERROR
        pShowErr "disk probe",ERRNO;
        RETURN PROBE_FAILED;
    ENDFUNC

    !***********************************************************************
    ! STEP 5 - LAST ON PURPOSE. The overflow discriminator for Question 1:
    ! these two calls are the ones expected to hit the 8388608 ceiling, and
    ! after finding B we cannot assume an ERROR handler will contain them.
    ! Kbyte first, bytes second (bytes overflows by 1024x more).
    !
    ! Three outcomes, all useful:
    !   a) both print a number  -> the ceiling is not what the manual says
    !   b) handler prints ERR_FILESIZE -> recoverable; production code may
    !      wrap FSSize in an error handler and fall back
    !   c) the task HALTS       -> NOT recoverable; production code must
    !      never call FSSize in a unit that can overflow, and \Mbyte from
    !      step 1 becomes the only permissible form
    !***********************************************************************
    PROC TGFsProbeLimit()
        TPWrite "TG FS PROBE: -- step 5: overflow (may halt) --";
        pFreeK PATH_TGS;
        pFreeB PATH_TGS;
        TPWrite "TG FS PROBE: step 5 done - nothing halted";
    ENDPROC

    LOCAL PROC pFreeK(string p)
        VAR num v;
        v:=FSSize(p\Free\Kbyte);
        TPWrite "  Free Kbyte = "+NumToStr(v,0)+" KB";
    ERROR
        pShowErr "Free Kbyte",ERRNO;
        RETURN;
    ENDPROC

    LOCAL PROC pFreeB(string p)
        VAR num v;
        v:=FSSize(p\Free);
        TPWrite "  Free bytes = "+NumToStr(v,0)+" bytes";
    ERROR
        pShowErr "Free bytes",ERRNO;
        RETURN;
    ENDPROC

    !***********************************************************************
    ! Shared error printer. ERRNO is passed in rather than read here, so the
    ! value cannot be disturbed by anything between the raise and the print.
    !***********************************************************************
    LOCAL PROC pShowErr(string tag,num e)
        TPWrite "  "+tag+" ERROR, ERRNO ="\Num:=e;
        IF e=ERR_FILEACC TPWrite "  "+tag+" = ERR_FILEACC";
        IF e=ERR_FILESIZE TPWrite "  "+tag+" = ERR_FILESIZE";  ! FALLBACK
    ENDPROC

ENDMODULE
