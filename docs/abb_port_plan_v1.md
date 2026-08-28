# FANUC → ABB Port: Plan & Recommended Design (v1)

Status: **v1 PROTOTYPE COMPLETE — Phases 1–3 implemented and validated end-to-end
on the RobotStudio virtual controller (2026-08-28).** Decisions log in §7;
per-phase status notes in §5; controller-only RAPID gotchas inline in §2/§4 and
collected in the findings doc below.
Scope: prototype of the TetraGen HMI request protocol on ABB RAPID (IRC5, RobotWare
6.15.08.0, IRB 4600-20/2.5) + a Python socket prototype standing in for the HMI.

Related docs:
- [rapid_validation_findings_v1.md](rapid_validation_findings_v1.md) — **the three defects found during VC validation**, their root causes and the rules they imply for the production port and the exporter.
- [robotstudio_setup.md](robotstudio_setup.md) — how to build the VC and run each phase's smoke test.
- [fanuc_hmi_request_program_calls_v1.md](fanuc_hmi_request_program_calls_v1.md) — request-number table (authoritative).
- [abb_robot_architecture_guide.md](abb_robot_architecture_guide.md) — pre-assessment. Feasible overall; §3 below lists corrections.
- [RDK_DriverSocket_RW5_6_vs_RW7.md](RDK_DriverSocket_RW5_6_vs_RW7.md) — RW6 vs RW7 socket API (identical for our purposes).

---

## 1. Facts extracted from the FANUC system (source of truth for the port)

Read from `resources/FANUC/KAREL/*.kl`, `resources/FANUC/LS/*.ls`,
`resources/FANUC/.TGS HMI-MODE LS PROGRAM SAMPLE/TD05tRJYQd.ls`, and
`TGuideWeldingHMI/FANUCRobot.cpp|.h`.

### 1.1 Transport & roles
- **The robot is the TCP server** (KAREL server tag `S3:`, port **2000** set in
  `SOCKET_COM`). The HMI (`FANUCRobot::connectRobot`) is the TCP **client**.
- One connection per program run. `TGMAINKL` loop: disconnect → wait for HMI to
  connect → serve one program selection/run → disconnect → repeat.
- **The robot always initiates.** Every exchange is strict half-duplex,
  robot-driven:
  - **Robot → HMI data**: robot writes a string, then blocks reading a **1-byte ack**
    (HMI `do_receive()` reads the payload and answers `"0"`).
  - **HMI → robot data**: robot writes a **prompt string** (e.g. `Give me the frame x`),
    then blocks reading a fixed number of bytes. HMI `do_send()` reads the prompt
    (content ignored, only used for sync/logging), then writes the payload.
- No message framing (no length prefix / terminator). Both sides rely on one
  `send` == one `recv`, which holds because the protocol is strictly alternating
  and messages are tiny (max ~60 bytes).

### 1.2 Data formats on the wire (all ASCII)
| Item | Format | Example |
|---|---|---|
| Request ID (robot→HMI) | unpadded int string | `4`, `10`, `100` |
| Real (both directions) | sign + 8 chars, 3 decimals (`%+09.3f`), **9 bytes** | `+0905.216` |
| Small int (HMI→robot) | fixed width 1 or 2 chars | `1`, `01`, `12` |
| Pose (robot→HMI) | 6 reals comma-joined `x,y,z,w,p,r` | `+0081.125,-0129.068,...` |
| Frame (HMI→robot) | 6 separate 9-byte reals, each prompted individually | |
| Strings (names, password) | raw chars, no terminator | `TD05tRJYQd`, `PWeld2` |

- Orientation convention: FANUC `w,p,r` = rotations about fixed X, Y, Z. The HMI
  internally uses `[x,y,z,A,B,C]` (A=Rz, B=Ry, C=Rx) and **swaps indices 3↔5** in
  `karelPoseStrToPose` / `sendKarelStrPose`. **The ABB wire format deliberately
  deviates for pose/frame payloads** — it carries `x,y,z` + normalized quaternion
  instead of Euler angles; conversion lives on the HMI/Python side. See §4.5.

### 1.3 FANUC state carriers (registers) and who reads them
| Register | Written by | Read by | Meaning |
|---|---|---|---|
| R[188] | REQ_PROG_SEL | TGMAINKL | selected mode: 1=run .tgs, 2=cam-cal |
| R[189] | R_F_T | TGMAINKL | FTP transfer status (1=ok) |
| SR[24] | R_F_T, SET_PASS_SR | TGMAINKL (prog name), R_P_C (password) | **program name == project password** (same string, dual use) |
| SR[25] | SET_SUB_ROUTINE_SR | R_C_F/R_C/R_W_F/R_P_C/R_E | sub-routine token (`C1PGlobal_m45_3`, `PWeld2`, `TSPWeld2_full`…) |
| SR[23] | SET_ROB_S_SR | (status string, e.g. `Ok`) | robot status |
| R[200] | R_P_C | .tgs program | password correct (0 aborts program) |
| R[199] | R_P_C | .tgs program | dry-run flag |
| PR[5]  | R_C_F | .tgs (`UFRAME[5]=PR[5]`) | camera user frame |
| R[197] | R_C_F | .tgs program | do-capture flag |
| R[196] | R_C | .tgs program | capture succeeded |
| R[187] | R_G_C_D | .tgs program | global-captures-done status |
| PR[6]  | R_W_F | .tgs (`UFRAME[6]=PR[6]`) | weld user frame |
| R[198] | R_W_F | .tgs program | weld status: 0=skip, 1=weld, 2=abort to end |
| R[170] | R_W_P | .tgs program | user-defined weld params flag |
| R[171..174] | R_W_P | .tgs / weld schedule | proc no, wire feed, arc length, arc control |
| R[175] | R_W_P | .tgs program | travel speed (used as motion speed of the weld move) |

### 1.4 Per-request message sequences (priority set)

`>` robot sends (HMI acks `0`) `<` robot prompts then receives.

| KAREL | ID | Sequence |
|---|---|---|
| `REQ_PROG_SEL` | — | `< "Give me the program ID"` → 1 char prog idx |
| `R_F_T` | 10 | `> id` · `> free TPP bytes` · `< "Give me FTP status"` → 1 char · `< "Give me prog name"` → ≤10 chars (HMI actually sends the *project password* here) |
| `R_P_C` | 5 | `> id` · `> pose` · `> sub_name (SR25)` · `> password (SR24)` · `< "Give me the status"` → 2 chars (pass_ok, dry_run) |
| `R_C_F` | 1 | `> id` · `> pose` · `> sub_name` · `< "Give me the frame x"`…`r` → 6 × 9 chars → PR[5] · `< "Give me capture status"` → 1 char |
| `R_C` | 2 | `> id` · `> pose` · `> sub_name` · `< "Give me capture status"` → 1 char |
| `R_G_C_D` | 11 | `> id` · `< "Give me global loc status"` → 1 char |
| `R_W_F` | 4 | `> id` · `> pose` · `> sub_name` · `< frame x..r` → 6 × 9 chars → PR[6] · `< "Give me weld status"` → 1 char |
| `R_W_P` | 14 | `> id` · `< "Give me UDWP flag"` → 1 char · `< "Give me travel speed"` → 9 chars · if flag=1: `< welder type` → 2 chars · `< proc` → 2 chars · `< wire feed speed` → 9 · `< arc length` → 9 · `< arc control` → 9 |
| `R_E` | 100 | `> id` · `> pose` · `> sub_name` |
| `SOCKET_COM` / `SOCKET_DISC` | — | connection open (server accept) / close |

Pose sent in every "frame-ish" request = current TCP of the **currently active
tool in the currently active user frame** (KAREL syncs `$UTOOL/$UFRAME` to the
TP-selected ones, then `CURPOS`).

### 1.4.1 Which frame does the HMI expect reported poses in? (verified 2026-08-28)

**The active user frame — i.e. the frame the HMI most recently sent — NOT the
robot base.** Verified in `RobotCell.cpp`: of all received poses, only the
`Capture` (R_C) one is consumed (line ~2406). The scan is transformed by
`act_pose` and *afterwards* by `bTpart` (line ~2447) — `bTpart` being exactly
the frame the HMI sent in `RequestCaptureFrame` (line ~2196). The composition
`bTpart · act_pose · cloud` only lands in base if `act_pose` is
frame-relative. ⚠ The line-2411 comment "TRANSFORM POINT CLOUD TO THE ROBOT
BASE" is misleading — that step reaches the *sent frame*; base comes from the
later `Transform(bTpart)`. Do not "fix" the robot side to report base poses.

Corollaries: (a) our `nTG_ActFrame`-resolved reporting is correct as-is,
including R_C_F's pose going out in the *previous* frame (the HMI receives and
ignores it — as it does every pose except R_C's); (b) the camera-calibration
handlers (`bPf`, `bPcam_pc` in `FANUCRobot.h`) expect **base**-frame poses, so
the Phase 4 cam-cal .tgs equivalent must set `nTG_ActFrame:=0` before those
requests.

### 1.5 .tgs program call order (from TD05tRJYQd.ls, minus touch-sense which is out of v1 scope)
1. Activate home frame/tool → `SET_PASS_SR(name)` → `SET_ROB_S_SR('Ok')` → `R_P_C` → abort if `R[200]=0`; dry-run handling from `R[199]`.
2. Per capture set: activate camera frame/tool → `SET_SUB_ROUTINE_SR('C<i>PGlobal_...')` → `R_C_F` → if `R[197]=1`: refresh frame, move, `R_C`, jump to end if `R[196]=0`.
3. After all sets: `R_G_C_D`.
4. Per weld: `SET_SUB_ROUTINE_SR('P<weld>')` → `R_W_F` → if `R[198]=2` jump to end; if `=1`: approach, `R_W_P`, weld move at `R[175]` speed with schedule from R[170..174], (R_W_S — out of v1), retract.
5. End: `R_E` (label 101 target — also the abort path).

---

## 2. Key ABB/RAPID facts the design relies on

Verified against the RAPID Technical Reference (3HAC050917 / 3HAC16581) and ABB
forums; items marked ⚠ still to be confirmed in RobotStudio during Phase 1.

1. **Task-wide global namespace answers the "inheritance" question.** All non-`LOCAL`
   PROCs/FUNCs/PERS/VARs of every module loaded in a task share one global scope.
   A dynamically loaded .tgs module can directly call `TG_ReqWeldFrame` and read
   `nTG_WeldStatus` declared in another module. No include/inherit mechanism exists
   or is needed. This is exactly the property the pre-assessment hoped for.
2. **Socket API** (`socketdev`, `SocketCreate/Bind/Listen/Accept/Send/Receive \Str`,
   `SocketClose`) needs RobotWare option **616-1 PC Interface** (add it to the
   virtual controller system too). API is identical RW6 vs RW7 (see comparison doc).
3. **`SocketReceive` default timeout is 60 s** and raises `ERR_SOCK_TIMEOUT`; pass
   `\Time:=WAIT_MAX` on every receive that waits on HMI work (captures can take
   minutes). Confirmed by RAPID reference.
4. **Virtual controller binds to `127.0.0.1`**; a real IRC5 must bind the actual
   LAN interface IP. Keep `SERVER_IP` a single `CONST`/`PERS`. (RobotStudio forum
   threads confirm; the RoboDK driver README warns the reverse — localhost may not
   work on a *real* controller.)
5. **RAPID `string` max length is 80 chars.** Longest protocol message ≈ 60 chars. OK.
6. **`num` holds exact integers only to 8 388 608 (2^23).** The free-disk-space
   message (R_F_T) can exceed that → use `dnum` + `NumToStr`/`ValToStr` dnum variants
   for that one value. ⚠ verify `FSSize("HOME:" ...\Free)` is the right RW6 call
   for free space on `HOME:`; v1 may send a dummy constant with a comment.
7. **Late binding** `%stProgName%;` calls a PROC by string; unknown name raises
   `ERR_REFUNKPRC`, catchable in an `ERROR` handler. Works with procedures from
   dynamically loaded modules.
8. **`Load \Dynamic, "HOME:/TGS/" + name + ".mod"` / `UnLoad`**: dynamic modules are
   dropped automatically when PP is moved to `main` — acceptable, since `TG_Main`
   reloads on demand each cycle. Module must not redeclare global symbols that are
   already loaded (each .tgs module gets unloaded right after its run, so
   consecutive .tgs programs can't collide).
9. **No `GLOBAL` keyword and no `TRY/ENDTRY` in RAPID** — the pre-assessment's
   snippets are pseudo-code (see §3).
10. **Pose math**: `CRobT(\Tool:=... \WObj:=...)` for current pose — and its
    `\Tool`/`\WObj` are **PERS parameters**: passing a `VAR` or a FUNC result
    raises "Argument error(123): not a persistent reference" (hit on the VC
    2026-08-28; same rule as motion instructions). When the tool/wobj must be
    computed, copy it into a scratch `PERS` first. ABB orientations
    are **normalized quaternions** (`orient`: q1²+q2²+q3²+q4²=1; violating it raises
    error 50076 "Orientation not correct"; `NOrient()` re-normalizes defensively).
    Verified 2026-08-27.
11. **String → frame in one call**: `StrToVal` parses any RAPID value literal,
    including structured types: `StrToVal("[[600,500,225.3],[1,0,0,0]]", myPose)`
    fills a `pose`, and `wobjTG_Weld.uframe := myPose;` is a plain component
    assignment. Reverse direction via `NumToStr` concatenation (preferred over
    `ValToStr` to control digits). Verified 2026-08-27. **80-char string budget**:
    trans at 2 decimals + quats at 6 decimals → worst case ≈ 75 chars. Fits, with
    the constraint that frame translations stay < ±9999.99 mm (trivially true for
    an IRB 4600 cell).
12. **No modal UFRAME/UTOOL in RAPID** — tool and wobj are per-motion-instruction
    arguments. The FANUC "set PR then activate UFRAME" idiom becomes: request PROC
    updates a shared `PERS wobjdata`; the .tgs program simply uses that wobj in its
    move instructions. `PERS` updates take effect immediately for subsequent motions.
13. **File transfer to a real RW6 IRC5** is *not* plain FTP out of the box
    (classic FTP needs the FTP option; RobotStudio's file transfer uses ABB's own
    protocol). **Decision (§7): plan on purchasing/enabling the FTP option** for the
    real cell. **For the prototype this is moot**: the VC's `HOME:` is a plain
    Windows folder inside the RobotStudio solution, so the Python side (or the
    user) just copies the .mod file there.
14. **Optional `PERS` parameters + conditional argument propagation.** The style-b
    request signatures (§7.6) use `PROC TG_Req...(\PERS tooldata Tool,\PERS
    wobjdata WObj)` and forward with `tgSendPose \Tool?Tool \WObj?WObj;`. Both are
    standard RAPID (`CRobT`'s own `\Tool`/`\WObj` are optional PERS parameters;
    `\Par?Arg` is the conditional-argument form). A `PERS` parameter is a
    persistent *reference*: legal directly in `CRobT(\Tool:= \WObj:=)` — no
    scratch copy needed — and component writes through it (`WObj.uframe:=...`)
    land in the caller's persistent. **Verified on the VC 2026-08-28**: program
    check clean, and all three forms exercised in a numerically validated cycle
    (the write-through frame reached the capture-pose report to 0.01 mm).

---

## 3. Corrections to `abb_robot_architecture_guide.md`

The architecture direction (core module + dynamically loaded program modules +
late binding, socket shared via module-level data) is **sound and adopted**. But:

| Guide says | Reality |
|---|---|
| `GLOBAL VAR socketdev ...` | No `GLOBAL` keyword. Default visibility *is* global; write `VAR socketdev client_socket;` at module level (not `LOCAL`). |
| `TRY ... DEFAULT ... ENDTRY` | Doesn't exist. Use a RAPID `ERROR` handler and test `ERRNO = ERR_REFUNKPRC` (and `ERR_FILEACC`/`ERR_LOADED` for Load). |
| Socket created/bound inside the `WHILE TRUE` each pass, unconditionally | Must be guarded: re-`SocketCreate` on an already-created socket errors. Use `SocketGetStatus` + defensive `SocketClose` first (our `TG_SocketDisc` mirror). |
| .tgs module closes the sockets at its end | Keep lifecycle in the core module only (mirrors `TGMAINKL` calling `SOCKET_DISC`). .tgs programs never touch the socket directly — they only call request PROCs. Simpler and matches FANUC. |
| `SocketReceive client \Str:=...` bare | Always add `\Time:=WAIT_MAX` (60 s default timeout otherwise). |

---

## 4. Recommended ABB architecture

### 4.1 Modules (task `T_ROB1`, single task, no Multitasking needed for v1)

```
SystemModule  TG_Comms.sys      ← the "KAREL library" equivalent (resident)
SystemModule  TG_Cell.sys       ← cell-hardware macros: FANUC's tiny .ls utility
                                  programs (CAM_OPEN/CAM_CLOSE → TG_CamOpen/
                                  TG_CamClose via dummy DO doTG_Camera;
                                  WELD_PREP/CAM_PREP/DRY_RUN_* added 2026-08-28
                                  as empty placeholder PROCs). Separate from
                                  TG_Comms on purpose: protocol vs hardware.
NormalModule  TG_Main.mod       ← TGMAINKL equivalent (resident; owns main())
NormalModule  <TgsName>.mod     ← one per .tgs program (dynamically loaded from HOME:/TGS/)
```

- `TG_Comms.sys`: socket vars, all shared `PERS` state (§4.3), low-level send/recv
  helpers, and the request PROCs. Marked `SYSMODULE` so it survives "PP to main"
  and is hidden from the operator's program view. For the very first smoke test it
  can start life as a plain .mod; converting to .sys + auto-load (system parameter
  *Controller → Automatic Loading of Modules*) is a hardening step.
- `TG_Main.mod`: `PROC main()` = production entry. AUTO-start of the cell runs it
  (FANUC's "controller starts tgmain.ls on AUTO" maps to setting `main` as
  production entry and starting from the FlexPendant/System IO).
- `.tgs` module: `MODULE TD05Test_Mod` containing `PROC TD05Test()`. **File name
  = PROC name = the program name the HMI sends** (`Load "HOME:/TGS/"+name+".mod"`
  then `%name%`); the **module name carries a `_Mod` suffix** because RAPID module
  names and global routine names share one namespace — naming a module like its own
  PROC is a semantic error ("Name error(45): module name ambiguous"; hit in
  RobotStudio 2026-08-28). Constraints for the future exporter: RAPID identifiers
  ≤ 32 chars, start with a letter, `[A-Za-z0-9_]`, case-insensitive (current
  10-char base62 .tgs names fit), and module name ≠ any global symbol name.

### 4.2 Naming map (KAREL → RAPID)

| KAREL | RAPID PROC (in TG_Comms) | Notes |
|---|---|---|
| SOCKET_COM | `TG_SocketCom` | create+bind+listen once, accept per cycle |
| SOCKET_DISC | `TG_SocketDisc` | defensive close (safe when already closed) |
| REQ_PROG_SEL | `TG_ReqProgSel` | fills `nTG_ProgSel` |
| R_F_T | `TG_ReqFileTransfer` | fills `nTG_FtpStatus`, `stTG_ProgName` |
| R_P_C | `TG_ReqPassCheck` | fills `nTG_PassOK`, `nTG_DryRun` |
| R_C_F | `TG_ReqCamFrame` | updates `wobjTG_Cam`, fills `nTG_DoCapture`; frame arrives as ONE pose-literal message, not 6 prompted reals (§4.5) |
| R_C | `TG_ReqCapture` | fills `nTG_CaptureOK` |
| R_G_C_D | `TG_ReqGlobalCapDone` | fills `nTG_GlobalCapOK` |
| R_W_F | `TG_ReqWeldFrame` | updates `wobjTG_Weld`, fills `nTG_WeldStatus`; frame arrives as ONE pose-literal message (§4.5) |
| R_W_P | `TG_ReqWeldParams` | fills `nTG_UdwpFlag`, `nTG_WeldProc`, `nTG_WireFeed`, `nTG_ArcLength`, `nTG_ArcControl`, `nTG_TravelSpeed` |
| R_E | `TG_ReqEnd` | |
| SET_PASS_SR / SET_SUB_ROUTINE_SR / SET_ROB_S_SR | plain `PERS string` assignment in the .tgs program (`stTG_ProgPass := "...";`) — no PROC needed | |
| CAM_OPEN / CAM_CLOSE (.ls utilities) | `TG_CamOpen` / `TG_CamClose` in **TG_Cell.sys** — flip dummy DO `doTG_Camera` (must exist in EIO; setup doc §8). TODO: map to the real camera output | |
| TGMAINKL | `main()` in `TG_Main.mod` | |

Each PROC carries a header comment `! FANUC: R_W_F (HMI request id 4)`.

### 4.3 Shared state (`PERS`, all in TG_Comms — the "register map")

```
PERS string stTG_ProgPass    := "";      ! SR[24] program name == project password
PERS string stTG_SubName     := "";      ! SR[25] sub-routine token
PERS string stTG_RobStatus   := "";      ! SR[23]
PERS num    nTG_ProgSel      := 0;       ! R[188]
PERS num    nTG_FtpStatus    := 0;       ! R[189]
PERS num    nTG_PassOK       := 0;       ! R[200]
PERS num    nTG_DryRun       := 0;       ! R[199]
PERS num    nTG_DoCapture    := 0;       ! R[197]
PERS num    nTG_CaptureOK    := 0;       ! R[196]
PERS num    nTG_GlobalCapOK  := 0;       ! R[187]
PERS num    nTG_WeldStatus   := 0;       ! R[198]
PERS num    nTG_UdwpFlag     := 0;       ! R[170]
PERS num    nTG_WeldProc     := 0;       ! R[171]
PERS num    nTG_WireFeed     := 0;       ! R[172]
PERS num    nTG_ArcLength    := 0;       ! R[173]
PERS num    nTG_ArcControl   := 0;       ! R[174]
PERS num    nTG_TravelSpeed  := 0;       ! R[175]

PERS wobjdata wobjTG_Cam  := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]]; ! PR[5]/UFRAME[5]
PERS wobjdata wobjTG_Weld := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]]; ! PR[6]/UFRAME[6]
PERS tooldata tTG_Cam  := ...;  ! UT[2]  — DUMMY VALUES, replace with calibrated camera tool
PERS tooldata tTG_Weld := ...;  ! UT[8]  — DUMMY VALUES, replace with calibrated torch TCP

! DEPRECATED fallback (decision 7.6) — FANUC modal UFRAME_NUM/UTOOL_NUM
! emulation: the .tgs program selects the active tool/frame BY NUMBER;
! tgSendPose resolves the number to the LIVE tooldata/wobjdata at read time.
! Used only when a request is called without explicit \Tool/\WObj; kept
! deliberately as the back-pocket alternative. (Revised 2026-08-28 after VC
! testing: the original PERS-to-PERS "active copy" went stale when a request
! updated the frame — RAPID assignment copies by value. Numbers need no
! refresh.)
PERS num nTG_ActTool  := 8;   ! UTOOL_NUM:  2=camera, 8=torch
PERS num nTG_ActFrame := 0;   ! UFRAME_NUM: 5=camera, 6=weld, else base
```

The received frames land in `wobjTG_Cam.uframe` / `wobjTG_Weld.uframe`
(`StrToVal`-parsed `pose`, §4.5). Because they're `PERS` and wobj is an
argument of each move, "receiving the frame and updating it in the welding
program" needs **no activation step** — the next `MoveL ... \WObj:=wobjTG_Weld`
uses the new value. This is the RAPID answer to the `R_W_F → PR[6] → UFRAME[6]=PR[6]`
idiom. **Caveat learned on the VC (2026-08-28)**: this holds only for data
referenced *by name*. A PERS-to-PERS copy (the original `wobjTG_Act := wobjTG_Cam`
"active frame") is by value and goes stale when the request updates the source —
the exact hazard behind FANUC's "re-emit `UFRAME[n]=PR[n]` after every
frame-PR-writing routine" invariant. The exporter must therefore never emit
frame copies; the tool/frame is either passed explicitly as `\Tool`/`\WObj`
PERS parameters (primary since decision 7.6 — a PERS parameter is a live
persistent reference) or selected by number (`nTG_ActFrame`, deprecated
fallback) and resolved live inside `tgSendPose`. Both make the FANUC re-emit
ritual unnecessary on ABB by construction.

### 4.4 Wire-protocol helpers (the "figure it out once" layer)

The KAREL repetition collapses into a handful of helpers in `TG_Comms`:

```
PROC tgSendAck(string data)          ! WRITE + READ(cmd::1): SocketSend \Str, then SocketReceive 1-byte ack, \Time:=WAIT_MAX
FUNC string tgPromptRecv(string prompt)  ! WRITE prompt + READ payload
PROC tgSendPose(\PERS tooldata Tool,\PERS wobjdata WObj)   ! CRobT(\Tool \WObj) → pose literal (§4.5) → tgSendAck; both omitted → deprecated modal fallback (§7.6)
FUNC string tgPoseToStr(pose p)      ! "[[x,y,z],[q1,q2,q3,q4]]" — NumToStr(trans,2) / NumToStr(quat,6), stays < 80 chars
FUNC pose   tgStrToPose(string s)    ! StrToVal into pose + NOrient safety; error handling on parse failure
FUNC string tgFmtReal(num v)         ! %+09.3f zero-padded formatter for scalar fields (NumToStr doesn't zero-pad)
FUNC num    tgParseReal(string s)    ! StrToVal wrapper with error handling
```

Every request PROC is then 5–15 lines, mirroring §1.4 line by line — same
sequence, same prompts, same acks and scalar field widths as KAREL, so the HMI's
non-frame handling (`do_send`/`do_receive`, fixed-width scalar fields, `"0"` acks)
works unchanged when the C++ `ABBRobot` class is written later. Frame/pose
payloads are the one deliberate deviation — next section.

### 4.5 Frame payloads — the one deliberate deviation from FANUC (decided 2026-08-27)

FANUC carries frames as Euler angles (`x,y,z,w,p,r`), received as **6 individually
prompted 9-byte reals**. ABB natively uses `x,y,z` + a **normalized quaternion**,
and RAPID can parse a whole pose literal in one call (§2.11). Decision:

- **Wire format (both directions)**: one message containing the RAPID pose literal
  `[[x,y,z],[q1,q2,q3,q4]]` — translations 2 decimals, quaternions 6 decimals
  (≈ 75 chars worst case, inside the 80-char RAPID string limit). The quaternion
  representative is NOT canonical: `CRobT` may return q or −q for the same
  rotation (observed on the VC 2026-08-28), plus signed zeros. Consumers and
  transcript diffs must compare rotations with q ≡ −q equivalence, never as
  strings; the Euler conversion on the PC side is naturally sign-insensitive.
- **Robot → HMI current pose** (R_C_F, R_W_F, R_P_C, R_C, R_E): one `tgSendAck`
  of `tgPoseToStr(...)` built from `CRobT` on the request's explicit
  `\Tool`/`\WObj` PERS parameters (§7.6; deprecated modal fallback when omitted).
- **HMI → robot frame** (R_C_F, R_W_F): ONE prompt `"Give me the frame"` → one
  ≤ 80-char payload → `tgStrToPose` → assign to `wobjTG_*.uframe`. (Replaces the
  six `"Give me the frame x"…"r"` exchanges of KAREL.)
- **Conversion lives on the PC side**: the Python prototype (and later the C++
  `ABBRobot` class) exposes the same frame interface as the FANUC path
  (matrix / `x,y,z,Rx,Ry,Rz`) and converts to/from quaternions internally,
  **normalizing before sending** (RAPID rejects non-normalized orients,
  error 50076). Python: small `euler_zyx_to_quat` / `quat_to_euler_zyx` helpers
  using only `math` — no numpy dependency.
- **No diverging paths above the robot class in the HMI**: `Robot` subclasses
  already own their wire encoding (`FANUCRobot::sendKarelStrPose` etc.); the
  future `ABBRobot` overrides the same hooks with the quaternion codec. Callers
  keep passing matrices, exactly as today.

### 4.6 `TG_Main.main()` (TGMAINKL equivalent)

```
main():
  TG_SocketDisc;                        ! always start clean (KAREL line 20)
  WHILE TRUE DO
    TG_SocketCom;                       ! bind/listen (first pass) + SocketAccept (blocks for HMI)
    TG_ReqProgSel;                      ! → nTG_ProgSel
    IF nTG_ProgSel = 1 THEN
      TG_ReqFileTransfer;               ! → nTG_FtpStatus, stTG_ProgName
      IF nTG_FtpStatus = 1 THEN
        tgRunTgsProgram stTG_ProgName;  ! Load \Dynamic "HOME:/TGS/"+name+".mod" → %name%; → UnLoad
      ENDIF
    ELSEIF nTG_ProgSel = 2 THEN
      ! TG_CamCalProg placeholder (out of v1 scope)
    ENDIF
    TG_SocketDisc;
  ENDWHILE
ERROR handler:
  ERR_REFUNKPRC → log, UnLoad if loaded, TG_SocketDisc, retry main loop
  ERR_SOCK_CLOSED / ERR_SOCK_TIMEOUT → TG_SocketDisc, retry
  Load errors (file missing) → log, TG_SocketDisc, retry
```

### 4.7 Sample .tgs test module (replaces D05tRJYQd for the prototype)

Simple module exercising the priority requests in the real .tgs order (§1.5):
pass-check → one fake capture set (`TG_ReqCamFrame`/`TG_ReqCapture` with small
safe moves or zero-length moves) → `TG_ReqGlobalCapDone` → one fake weld
(`TG_ReqWeldFrame` → `TG_ReqWeldParams` → plain `MoveL` placeholder where
`ArcLStart/ArcL/ArcLEnd` will go once RobotWare Arc is in scope) → `TG_ReqEnd`.
All motion targets dummy/safe; welding schedule values only stored in `PERS` nums
(FANUC's `SET_VAR` into `AWE1WP*` schedules has no counterpart until we pick the
RobotWare Arc data mapping — explicitly deferred, commented in code).

### 4.8 Python prototype (`hmi_prototype/abb_server.py`)

Mirror of the C++ class but as the client (like `FANUCRobot`), in the style of
`resources/FANUC/SOCKET/Fanuc.py`:

- `class AbbTgsHmiClient`: `connect(host, port)`; `do_receive()` (recv → send `"0"`),
  `do_send(payload)` (recv prompt → send payload), `fmt_real(v)` (`f"{v:+09.3f}"`),
  pose codec per §4.5: `pose_to_rapid_literal(xyzwpr)` / `rapid_literal_to_xyzwpr(s)`
  with `math`-only Euler↔quaternion conversion (normalize before sending).
- Serve loop: first exchange = program selection (recv `Give me the program ID` →
  send `"1"`), then R_F_T service (send ftp status `"1"`, send prog name), then
  dispatch on received request IDs (`1,2,4,5,10,11,14,100`) with dummy frames/flags,
  printing everything. Configurable canned answers (e.g. `dry_run=1`,
  `do_capture=1`, frame values) so robot-side branches can be exercised.
- Pure `socket` stdlib, single-threaded, blocking — same simplicity as `Fanuc.py`.

### 4.9 Repo layout to add

```
abb/rapid/TG_Comms.sys        abb/rapid/TG_Main.mod        abb/rapid/TGS/TD05Test.mod
hmi_prototype/abb_server.py
docs/abb_request_protocol_v1.md   (wire spec, extracted from §1 once frozen)
docs/robotstudio_setup.md         (VC options: PC Interface 616-1; bind 127.0.0.1; HOME:/TGS)
```

---

## 5. Phases

**Phase 1 — comm core smoke test.** `TG_Comms` with socket lifecycle + helpers +
`TG_ReqProgSel` + `TG_ReqEnd`; minimal `abb_server.py`; RobotStudio VC
(IRB 4600-20/2.5, RW 6.15, PC Interface) ↔ Python on 127.0.0.1:2000. Exit
criterion: connect / prog-sel / end / disconnect loop runs twice in a row.
*Status 2026-08-27: code written (`abb/rapid/TG_Comms.sys`, `abb/rapid/TG_Main.mod`,
`hmi_prototype/abb_server.py`). Python side + pose codec + full choreography
verified by automated tests (`hmi_prototype/test_phase1.py`, 11/11 green,
including a fake-robot emulation of the RAPID message sequence). RAPID syntax
cross-checked against the known-good RoboDK RW6 driver.
**Validated on the VC 2026-08-27** (two clean cycles, transcripts matched the
expected choreography exactly).*

**Phase 2 — priority requests.** All PROCs of §4.2 + shared PERS; Python handlers
with dummy data; verify scalar field widths and the §4.5 pose codec round-trip
(known frame → Python → RAPID → `wobjTG_*.uframe` → sent back → matches, incl.
Euler↔quaternion conversion and normalization; check in RobotStudio watch window).
*Status 2026-08-28: implemented — all 7 remaining request PROCs in `TG_Comms.sys`,
sample .tgs module `abb/rapid/TGS/TD05Test.mod` (mirrors the TD05tRJYQd call
order, touch-sense omitted), Python handlers for ids 1/2/4/5/10/11/14. `TG_Main`
already late-binds the received program name (`%stTG_ProgName%` with
ERR_REFUNKPRC handling) — Phase 3 only adds Load/UnLoad + the file copy.
13 new automated tests (24 total green): full-cycle choreography vs a fake-robot
executable spec, plus every branch (ftp fail, wrong password → FANUC 'END'
semantics, capture fail → abort, weld skip/abort, predefined vs user-defined
schedule).
**Validated on the VC 2026-08-28** after two controller-only fixes (module-name
ambiguity §4.1; stale PERS frame copy → modal-number active frame §4.3; CRobT
PERS-parameter rule §2.10). Frame math verified numerically: the reported
capture pose matched a hand-computed transform of the base pose into the served
camera frame to 0.01 mm, and frames persist across cycles like FANUC UFRAME[n].*

**Phase 3 — dynamic program flow.** `TG_Main` loop + `Load \Dynamic`/late
binding/`UnLoad` + sample .tgs module; full end-to-end: Python pushes
`TD05Test.mod` into VC `HOME:/TGS/` (file copy), selects program 1, robot loads
and runs it, all requests served, `R_E`, disconnect, loop again.
*Status 2026-08-28: implemented — `tgRunTgsProgram` does Load \Dynamic → %name% →
UnLoad with graceful handling of ERR_LOADED (Phase-2 leftover module), ERR_UNLOAD,
ERR_REFUNKPRC and ERR_IOERROR (each ends the cycle with a clean R_E or a warn);
`abb_server.py` takes an optional `vc_home_dir` argument and copies
`abb/rapid/TGS/<prog>.mod` into `<HOME>/TGS/` during request 10, reporting a
failed copy as ftp status 0 (FANUC error-path parity). 3 new tests (27 total
green).
**Validated on the VC 2026-08-28**: two consecutive cycles with nothing
pre-loaded — transfer → `Load \Dynamic` → late-bound call → all requests →
`UnLoad` → `R_E` → disconnect → repeat. This completes the v1 prototype scope.*

**Phase 4 — hardening & scope growth (post-prototype).** Reconnect/error-recovery
matrix (→ [abb_error_recovery_matrix_v1.md](abb_error_recovery_matrix_v1.md):
5 findings; **I1 ResetRetryCount, I4 malformed-frame-forces-skip/abort, and
I2/I3 stale-module fix implemented 2026-08-28** — the last surfaced F-E:
RAPID error-handler fall-through is an implicit RETURN, so mid-.tgs errors
were silently swallowed until the explicit RAISE; I5 receive timeout deferred
for FANUC parity, I6 counters declined; VC validation of I1/I4 + I2/I3
pending, robotstudio_setup §11–§12); real file transfer via the **FTP option** (decided §7; verify option id and
server behavior on RW6.15 when quoting the real cell); remaining requests (R_W_S,
R_TS_*, camera calibration set — the cam-cal .tgs must set `nTG_ActFrame:=0`, §1.4.1
corollary b); RobotWare Arc mapping for R_W_P (wire level done Phase 2;
**application-layer research done 2026-08-28** →
[abb_weld_params_research_v1.md](abb_weld_params_research_v1.md): ABB weld
params are plain PERS welddata assignments, no FANUC schedule-file gymnastics;
the KAREL's wirefeed/arclength "swap" is compensation for ArcTool's mislabeled
$CMD_VOLTS/$CMD_WFS — map by intent on ABB; concrete welddata components are
cell-config-dependent → §5 of that doc lists what must come from the customer.
Cell power source confirmed 2026-08-28: **Fronius TPS 500i /600V/nc** — in
scope of ABB's RI-FB inside/i manual, needs RW ≥ 6.05 + [633-4] Arc + [637-1]
Production Screen + a fieldbus option, and the FANUC cell's Special-2-step
mode maps to ABB characteristics/synergic mode; implementation is a separate
future task); cell macros **done 2026-08-28** (TG_WeldPrep/TG_CamPrep/TG_DryRunOn/Off as empty
placeholder PROCs in `TG_Cell.sys`, called from the sample .tgs in the FANUC
order — sample lines 13–19/63; ⚠ empty PROC bodies to confirm at the next VC
program check); real `FSSize` free-space value in R_F_T
(dnum, §2.6); port of the Python prototype into a C++ `ABBRobot : Robot` class in
TGuideWeldingHMI (small: `do_receive`/`do_send` identical, plus the §4.5 quaternion
codec, no XML path). The tool/frame parameter style is **decided** (§7.6: explicit
`\Tool`/`\WObj` PERS parameters, style b, implemented 2026-08-28) — the exporter
emits explicit arguments on every request call; the modal-number fallback stays
deprecated in the back pocket.

---

## 6. Assumptions & recommended defaults (speak up if wrong)

1. **Robot = TCP server, port 2000** (same as FANUC). Port and server IP are
   config data, not literals: `PERS num nTG_Port := 2000;` /
   `PERS string stTG_ServerIP := "127.0.0.1";` — editable from
   FlexPendant/RobotStudio without touching code. VC binds `127.0.0.1`, real cell
   binds the controller LAN IP.
2. **Byte-compatible wire format for everything except frames**: keep FANUC scalar
   field widths, prompts, and acks so the eventual C++ ABB class reuses the FANUC
   send/receive helpers. Frame/pose payloads use the ABB-native quaternion pose
   literal instead (decided — see §4.5).
3. Frame updates are applied **immediately** inside the request PROC (no separate
   "activate" step — impossible/unneeded in RAPID, see §4.3).
4. `R_W_P` welding-schedule side effects (`SET_VAR AWE1WP*`) reduced to storing
   PERS values; RobotWare Arc mapping deferred (cell may not have the Arc option).
5. Touch-sense family, `R_W_S`, `R_C_C*`, `T_T_R_F` are out of v1 (per priority list).
6. `TGMAINKL`'s dead code (`status = 0` overwriting the `R_F_T` call status) is
   ported by intent, not literally.
7. Free-memory value in `R_F_T`: v1 sends a dummy constant (⚠ `FSSize` to verify);
   HMI only compares it against the program size, so a large constant is safe for
   the prototype.
8. Tool/frame numbers (UT2/UT8/UFRAME5/6/9) become the named PERS data of §4.3
   with **dummy values marked `! TODO replace with calibrated data`**.

## 7. Decisions log

1. **Wire compatibility**: byte-level FANUC compatibility for the message
   choreography (ids, prompts, acks, scalar widths), **except frames/poses**, which
   switch to ABB-native `x,y,z` + normalized quaternion carried as a RAPID pose
   literal. Euler↔quaternion conversion lives on the PC side (Python now, the C++
   `ABBRobot` class later), so the HMI's frame-facing interface (matrix /
   `x,y,z,Rx,Ry,Rz`) stays brand-agnostic and no diverging call paths appear above
   the robot class. Verified: RAPID `orient` must be normalized (error 50076;
   `NOrient` available) and `StrToVal` parses a pose literal in one call. → §4.5.
2. **Port 2000**, held in an easily changed config variable (`PERS`), not a literal.
3. **File transfer to the real cell: FTP option** on the controller. Prototype uses
   direct copy into the VC's `HOME:` folder either way.
4. **Abort semantics confirmed**: `nTG_WeldStatus = 2` / `nTG_CaptureOK = 0` →
   call `TG_ReqEnd` then `RETURN` from the .tgs PROC (mirrors `LBL[101] → R_E`).
5. **No Multitasking in v1** — socket lives in the motion task; request calls block,
   exactly like KAREL `CALL_PROG`.
6. **Tool/frame parameter style — DECIDED 2026-08-28: (b) explicit PERS
   parameters, with the modal path retained as a DEPRECATED fallback** (kept
   deliberately as the back-pocket alternative — do not delete). The pose-touching
   request PROCs take optional `\PERS tooldata Tool,\PERS wobjdata WObj`; passing
   both is the (b) style — the parameters are persistent references, fed straight
   to `CRobT` (no `tTG_Scratch` copies on this path), and for R_C_F/R_W_F the
   served frame is written through `WObj.uframe`, so the request no longer
   hardcodes data names. Omitting them falls back to the original modal-number
   resolution (with a TPWrite warning if exactly one is passed). Mechanically this
   is (c)'s optional-argument shape, but as policy it is (b): `TD05Test.mod`,
   `TG_Main`, and everything the exporter emits pass explicit arguments; the modal
   numbers are exercised by nothing. Wire format unchanged — transcripts stay
   byte-comparable. **Validated on the VC 2026-08-28**: two consecutive cycles,
   no fallback warning, every reported pose arithmetically consistent with the
   served frames — capture pose = cam-frame transform of the base pose
   (0.008 mm / 0.000°), R_E pose = weld-frame transform of the home pose
   (0.007 mm / 0.000°), R_W_F demo pose exact — and frames persist across cycles
   and program restarts through the parameter path (first `R_C_F` of a later
   cycle reported the predicted persisted-frame value exactly).

   Pre-decision context — v1 originally emulated FANUC's modal
   `UTOOL_NUM`/`UFRAME_NUM` with two global `PERS num`s (`nTG_ActTool`/
   `nTG_ActFrame`) that `tgSendPose` resolved to live `tooldata`/`wobjdata` at
   read time (§4.3), chosen for *translation parity*: one assignment per FANUC
   modal line, .tgs line-for-line comparable with the .ls it came from. Not the
   idiomatic RAPID form. The alternatives weighed:
   - **(a) Keep modal numbers.** 1:1 with FANUC output; requests are callable from
     anywhere without knowing data names. Cost: hidden global state — a request
     issued without setting the numbers first reports a pose in the wrong frame with
     no diagnostic (the F-2 defect class), and the number→data map lives hardcoded in
     `tgActTool`/`tgActWobj`, so every new tool/frame (cam-cal, extra weld frames)
     means editing `TG_Comms` rather than the generated program.
   - **(b) Explicit PERS parameters.** `PROC TG_ReqCamFrame(PERS tooldata tool,
     PERS wobjdata wobj)` — a `PERS` *parameter* is itself a persistent reference,
     so it can be passed straight to `CRobT(\Tool:= \WObj:=)`; this also deletes the
     `tTG_Scratch`/`wobjTG_Scratch` workaround of §2.10 and makes the active frame
     visible at each call site. Cost: signatures diverge from the KAREL call list,
     and the exporter must resolve FANUC numbers to RAPID data names at emit time.
   - **(c) Modal default + optional override.** `\Tool`/`\WObj` optional arguments
     that win when `Present()`, falling back to the numbers. Keeps parity, allows the
     exporter to be explicit where it knows the answer. Cost: two paths to test.

   Either way the §4.3 invariant is unconditional: **never copy** a `wobjdata`/
   `tooldata` a request PROC may update — reference it by name, by number resolved at
   point of use, or by `PERS` parameter. The ABB counterpart of FANUC's
   `UFRAME[n]=PR[n]` re-emit is *nothing*, not a translation of it
   ([rapid_validation_findings_v1.md](rapid_validation_findings_v1.md) F-2).
