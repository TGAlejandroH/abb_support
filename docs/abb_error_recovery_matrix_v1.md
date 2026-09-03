# ABB reconnect / error-recovery matrix — v1 (PLAN, no code yet)

Status: **implemented 2026-08-28** (everything green-lit) — Phase 4 item 1 of
[abb_port_plan_v1.md](abb_port_plan_v1.md) §5. **I1** (ResetRetryCount),
**I4** (frame-parse failure → skip/abort, abort flavor for R_W_F), **I2/I3**
(stale-module fix) and **I7** (wire-error recovery via `tgCycleAbort` /
`ExitCycle`, added after the §12 kill test **halted the program** and exposed
finding **F-F**: unhandled errors do not propagate through the late-binding
`%%` call) implemented, all with explicit FIX comments; **I5 deferred by
decision** (FANUC parity — keep documented for later); **I6 declined**.
Findings F-E and F-F (§3) both concern RAPID error-handler semantics.
VC validation: [robotstudio_setup.md](robotstudio_setup.md) §11 (I1/I4) and
§12 (I2/I3 checks 1+3 done; check 2 = F-F re-test pending).

Sources: current `TG_Comms.sys`/`TG_Main.mod` handlers (each row verified against
the code), the FANUC KAREL originals (parity reference), the RoboDK RW6 driver
(`resources/Sample RAPID program for socket communication/`, known-good recovery
idioms), RAPID Technical Reference. Items marked ⚠ are to be confirmed by the VC
experiments of §6.

## 1. Design attitude (inherited, not invented)

The FANUC system's recovery philosophy, which we keep:

- `TGMAINKL`/`SOCKET_COM` run under `%NOPAUSE = ERROR + COMMAND + TPENABLE` and
  **ignore almost every returned status** — the program never pauses on error;
  it drops the connection and loops back to accept.
- **The recovery unit is the cycle.** Any mid-cycle failure ends the cycle
  (drop socket, unload program, back to accept). No mid-cycle resume, no
  per-request retry — the HMI restarts the whole session, which it already does.
- The HMI side owns user-facing error reporting; the robot side logs to the
  Operator Window and keeps serving.

RAPID equivalent already in place: `tgMainCycle`'s `ERROR` handler catches
anything propagated from the request PROCs / .tgs program (RAPID errors
propagate up the call chain until a handler takes them), logs `ERRNO`, calls
`TG_SocketDisc`, `RETURN`s to the `main()` loop → next accept. Validated in
Phase 1 ("HMI vanished mid-cycle" row of
[robotstudio_setup.md](robotstudio_setup.md) §4).

## 2. The matrix

### A. Connection layer (`TG_SocketCom`: create/bind/listen/accept)

| # | Event | Detection | Current behavior | Planned |
|---|-------|-----------|------------------|---------|
| A1 | HMI not running (nobody connects) | `SocketAccept` blocks (`\Time:=WAIT_MAX`) | waits forever — correct (FANUC parity) | keep |
| A2 | Accept interrupted / socket closed under the accept | `ERR_SOCK_CLOSED` | close both + `WaitTime 2` + `RETRY` — **halts after the retry limit** (F-A) | add `ResetRetryCount` before `RETRY` |
| A3 | `ERR_SOCK_TIMEOUT` on accept (only possible if a finite time is ever configured) | `ERR_SOCK_TIMEOUT` | bare `RETRY` — same retry-limit halt (F-A) | `ResetRetryCount` + `RETRY` |
| A4 | Bind fails (port still held: crashed task, TIME_WAIT) | bind-time error, not in the handled list | propagates to `tgMainCycle` → disc (incl. `WaitTime 2`) → next cycle retries — likely self-heals | keep; add explicit log branch ⚠ verify ERRNO on VC |
| A5 | Second HMI client connects while one is being served | queued in listen backlog, robot unaware | after the cycle ends, the next accept may take a **stale** queued connection; first exchange with a dead peer raises `ERR_SOCK_CLOSED` → recovered as B1 | keep (self-healing, one wasted cycle); document |

### B. Mid-session socket failures (`tgSendAck` / `tgPromptRecv`, any request)

| # | Event | Detection | Current behavior | Planned |
|---|-------|-----------|------------------|---------|
| B1 | HMI process dies / closes socket (OS sends RST/FIN) | `ERR_SOCK_CLOSED` on the next send or receive | outside a .tgs run: propagates → `tgMainCycle` → disc → new cycle (Phase-1 validated). Inside a .tgs run: **the program HALTED at the failing instruction** (VC-observed 2026-08-28, events 41595/40228/10020/10126) — unhandled errors do not propagate through the late-binding `%%` call (F-F), so no handler up-chain ever saw the error; `UnLoad` never executed (F-B) | **implemented (I7)**: the wire helpers `tgSendAck`/`tgPromptRecv` recover at the source — catch-all handler → `tgCycleAbort`: log + close sockets + `ExitCycle` to main (which starts with `TG_SocketDisc`); works from any call depth, no propagation needed |
| B2 | HMI **machine** power-fails / cable pulled (half-open TCP, no RST) | nothing — `SocketReceive \Time:=WAIT_MAX` blocks forever | robot hangs until operator stops the program (FANUC KAREL `READ` behaved identically) | configurable receive timeout `nTG_RecvTimeout` (I5); default keeps today's behavior |
| B3 | HMI alive but hung (connected, sends nothing) | same as B2 | same as B2 | same as B2 — and this is why the default must stay "wait forever": legitimate captures/user interaction can take arbitrarily long |
| B4 | Send to a peer that just closed | TCP accepts the first send into the buffer; the **paired ack receive** then raises `ERR_SOCK_CLOSED` | recovered as B1 | keep — note: the FANUC ack-per-message design means a dead peer is always detected within one exchange; no silent desync is possible |
| B5 | E-stop / Motors Off mid-receive, then resume | none — execution suspends and resumes in place | if the HMI kept the socket open and kept waiting: session **continues** where it stopped. If the HMI gave up: B1 path | keep; confirm by VC experiment E4 ⚠ |
| B6 | Controller warm restart / power fail mid-session | sockets are system-closed; PP position kept | next socket op errors (⚠ exact ERRNO to observe) → `tgMainCycle` → disc → recovers *if* execution is restarted; whether it auto-restarts is cell config (System Input "Start at Main" etc.), out of RAPID's hands | verify E5 ⚠; document real-cell config note |

### C. Protocol / payload failures

| # | Event | Detection | Current behavior | Planned |
|---|-------|-----------|------------------|---------|
| C1 | Bad program ID (unparseable) | `StrToVal` false in `TG_ReqProgSel` | `nTG_ProgSel:=0` → "unknown program ID" → clean cycle end | keep (FANUC parity) |
| C2 | Unknown program ID (parses, not 1/2) | `IF/ELSE` in `tgMainCycle` | clean cycle end | keep |
| C3 | Bad scalar payload (ftp status, flags, weld params) | `tgParseReal` error path | value defaults to 0 + TPWrite, cycle continues — 0 is the "fail/skip" value for every branch flag, so the failure direction is safe | keep; document the invariant "0 = safe default" |
| C4 | **Bad frame payload** (R_C_F / R_W_F) | `tgTryStrToPose` false | TPWrite, frame **keeps its previous value**, `do_capture`/`weld_status` still read → a capture can proceed against a stale frame — the F-2 defect class arriving over the wire (F-C) | on parse failure ABORT the program (decision revised 2026-08-28, both requests): R_C_F → `nTG_DoCapture:=2` (robot-side sentinel, wire carries only 0/1); R_W_F → `nTG_WeldStatus:=2` (I4) |
| C5 | Bad pass-check payload (< 2 chars) | `StrLen` check | `nTG_PassOK:=0` → .tgs returns without R_E (FANUC 'END' parity) | keep |
| C6 | HMI sends a program name with no matching file/PROC | `ERR_IOERROR` / `ERR_REFUNKPRC` on Load/`%name%` | clean `R_E` + cycle end (Phase-3 validated) | keep |

### D. Dynamic-load lifecycle (`tgRunTgsProgram`)

| # | Event | Detection | Current behavior | Planned |
|---|-------|-----------|------------------|---------|
| D1 | Module already loaded (Phase-2 manual load) | `ERR_LOADED` | `TRYNEXT` → runs the in-memory module | replace with UnLoad-then-`RETRY` (I2); `TRYNEXT` only if the UnLoad itself fails. **VC-validated 2026-08-28** (fallback variant: RobotStudio-loaded modules cannot be `UnLoad`-ed → warn cascade + bounded `using it`, cycle completes; end-of-run UnLoad warns too — expected) |
| D2 | **Module left loaded by an aborted previous cycle** (B1 during .tgs run) | `ERR_LOADED` on next cycle's Load | `TRYNEXT` runs the **old in-memory version** even though a new file was just transferred — version-skew hazard (F-B; the HMI's FTP compare-before-send keeps the *file* fresh but cannot unload the *module* — see F-B note) | fixed by the same I2 + defensive UnLoad in the cycle handler (I3) |
| D3 | UnLoad fails | `ERR_UNLOAD` | warn + `TRYNEXT` | keep. **VC-validated 2026-08-28** (end-of-run UnLoad on a RobotStudio-loaded module) |
| D4 | PP moved to main by operator | n/a | `\Dynamic` modules are auto-dropped (plan §2.8) — clean by construction. **Nuance (VC 2026-08-28): ExitCycle's PP move does NOT drop them** — an I7 recovery leaves the module loaded and the I2 reload path takes it next cycle (validated) | keep |

### E. Not recoverable at RAPID level (documented, not handled)

Motion supervision / collision, joint limits, singularity stops, hardware
faults: these suspend execution at system level; `ERROR` handlers never see
them. Recovery is the operator's (or a cell-level supervisor's): resume →
behaves as B5; abandon → restart at main → clean start (D4 clears modules,
`main()` starts with `TG_SocketDisc`). The HMI sees a dead/hung session either
way — its existing timeout/reconnect covers it.

## 3. Findings (gaps in the current code)

- **F-A — bounded RETRY can halt the server loop.** RAPID's retry counter is
  limited by the system parameter *No Of Retry* (default 4); when exhausted,
  the error is no longer recovered and execution **stops**. `TG_SocketCom`
  RETRYs `ERR_SOCK_CLOSED`/`ERR_SOCK_TIMEOUT` without `ResetRetryCount`, so 4-5
  consecutive failures kill the 24/7 loop. The RoboDK driver calls
  `ResetRetryCount` before exactly this kind of indefinite `RETRY`. ⚠ default
  value to confirm in system parameters on the VC.
- **F-B — aborted cycle leaves the old .tgs module loaded.** Any error during
  the .tgs run skips `UnLoad`. The next cycle transfers a (possibly newer)
  file, `Load` raises `ERR_LOADED`, and the current `TRYNEXT` silently runs the
  **stale in-memory module** instead of the file just transferred.
  *HMI-side mechanism, verified 2026-08-28 in TGuideWeldingHMI:* while serving
  R_F_T the HMI FTP-downloads the controller's copy and byte-compares it with
  the fresh program (`RobotCell.cpp` ~1853 →
  `WeldingProject::CompareCurrentWithProgramInTheController`, ~2848); equal →
  skip the send (ftp status 1), differ/absent → `TransferRobotProgram()` (plain
  FTP PUT overwrite; the only FTP delete is the separate `FreeUpTPPMemory`
  housekeeping). On FANUC this is sufficient because the file in `md:` *is*
  the program `CALL_PROG` runs. On ABB it keeps the **file** fresh but cannot
  reach the **loaded module** in task memory: identical files → stale module
  has identical content, benign; *different* files → HMI uploads the new file,
  `Load` still hits `ERR_LOADED`, and the old in-memory version runs while the
  HMI believes the new one does. The fix is therefore robot-side (I2/I3) and
  VC-testable (the prototype file copy stands in for FTP; module lifecycle is
  identical).
  *Performance addendum (question answered 2026-08-28):* I2 adds **no** work
  to normal cycles — the Phase-3 lifecycle already does `Load` → run →
  `UnLoad` every cycle, so `ERR_LOADED` (and the UnLoad-then-RETRY) only fires
  on abnormal leftovers. The real per-start cost is the unconditional `Load`
  itself, which FANUC never paid (the transferred file *is* the resident
  program there). If E7 measures it as significant for realistic .tgs sizes,
  the candidate optimization is an ftp-status extension "2 = unchanged, not
  re-sent" (the HMI knows this in its compare branch): status 2 → skip
  `UnLoad`/reuse the loaded module, status 1 → always UnLoad + Load fresh.
  Design change — only if E7 justifies it.
- **F-C — malformed frame payload proceeds against a stale frame.** The
  request keeps the previous `oframe` and still reads the go/no-go flag; if the
  HMI says "capture", the capture pose is reported in a frame the HMI doesn't
  know it has. Same defect class as findings F-2 — wrong-frame data with no
  diagnostic — only triggered from the wire side.
- **F-D — half-open TCP hangs the robot forever** (B2). Acceptable on the VC;
  on a real cell an HMI-PC power loss would freeze the robot in-cycle until an
  operator intervenes. FANUC had the same behavior, so this is parity — but a
  cheap `PERS` timeout makes it tunable per cell.
- **F-E — error-handler fall-through is an implicit RETURN, not a RAISE**
  (found 2026-08-28 while implementing I2/I3). A RAPID `ERROR` handler that
  runs to its end completes the routine as if it had RETURNed; only an
  explicit `RAISE` (or having no handler at all) propagates. The pre-fix
  `tgRunTgsProgram` handler ended with a comment claiming unmatched errors
  "propagate to tgMainCycle" — fixed by an explicit `tgTryUnload` + `RAISE`
  tail. *Scope correction after the F-F discovery*: this swallow analysis
  applies only to errors arising in `tgRunTgsProgram`'s **own frame**
  (Load/UnLoad/the `%%` call itself); wire errors inside the .tgs never
  reached this handler in any version (see F-F). Rule for all future
  handlers stands: **never rely on fall-through; end every handler branch
  with RETRY, TRYNEXT, RETURN, or RAISE.**
- **F-F — unhandled errors do not propagate through a late-binding (`%%`)
  call; the program stops at the failing instruction** (VC-observed
  2026-08-28 during the §12 kill test, RW 6.15). Evidence: HMI killed
  mid-.tgs → PP halted AT `SocketReceive` inside `tgSendAck` with event-log
  sequence 41595 Socket error → 40228 Execution error → 10020/10126, and
  **zero** handler output (no unload warn, no cycle-error line) — the error
  reached neither `tgRunTgsProgram` nor `tgMainCycle` across the
  `%stTG_ProgName%` frame, while the identical kill outside the .tgs
  (mid-prog-sel, normal frames only) has recovered since Phase 1. ⚠ the
  exact documented rule (RAPID reference wording for `%%` + error recovery)
  still to be located; the fix does not depend on it. **Fix (I7)**: recover
  at the bottom — `tgSendAck`/`tgPromptRecv` catch-all handlers call
  `tgCycleAbort` (log `TG: cycle error, ERRNO` + `TG: wire lost - restarting
  cycle`, close sockets, `ExitCycle`); main() is the recovery target and
  re-enters the accept loop. **VC-validated 2026-08-28**: ExitCycle works
  from an error handler at depth through the `%%` chain, the program kept
  running (served the next session untouched), observed `ERRNO = 1095`
  (runtime number of the peer-closed socket error). Companion fact:
  **ExitCycle's PP move does NOT drop `\Dynamic` modules** (unlike a manual
  PP-to-main) — the next cycle took the I2 reload path (`module already
  loaded - reloading from file` → silent unload → fresh Load → clean run),
  which validated I2 in its primary stale-module role. Consequence for the
  exporter: .tgs programs need no error handlers of their own — the wire
  layer self-recovers below them. Remaining ⚠: locate the documented RAPID
  rule for `%%` + error propagation (behavior itself is now pinned by VC
  evidence).

## 4. Non-findings (verified sound)

- Ack-per-message protocol bounds failure detection to one exchange (B4).
- Errors propagate correctly from any request PROC depth to `tgMainCycle`
  (Phase-1 validated), and `main()`'s handler-free loop is safe because the
  cycle handler only executes `SocketClose`/`WaitTime`/`TPWrite`/`RETURN` —
  none of which raise recoverable errors.
- `PERS` state needs no cleanup between cycles: every .tgs program re-seeds
  its own tokens (Phase-2 lesson, `stTG_SubName` reset), frames are *supposed*
  to persist, and C3's zero-defaults fail safe.

## 5. What I plan to implement (pending your go)

| # | Change | Where | Fixes |
|---|--------|-------|-------|
| I1 | `ResetRetryCount` before each `RETRY` — **implemented 2026-08-28** | `TG_SocketCom` `ERROR` handler | F-A |
| I2 | **implemented 2026-08-28** (explicit FIX comments in `TG_Main.mod`) — `ERR_LOADED` → best-effort `tgTryUnload` + `RETRY` (re-Load the fresh file), bounded by a `bRetriedLoad` flag; if the module still cannot be unloaded (e.g. RobotStudio-loaded, not `Load`-ed), second `ERR_LOADED` → warn + `TRYNEXT` (pre-fix behavior as fallback, cannot loop). No happy-path cost (see F-B addendum) | `tgRunTgsProgram` `ERROR` handler | F-B, D1, D2 |
| I3 | **implemented 2026-08-28** — new `tgTryUnload` helper (own handler: warn + `TRYNEXT`, a failed unload never kills a cycle) + `tg_module_loaded` flag. Primary cleanup: `tgRunTgsProgram`'s handler tail (`tgTryUnload` + explicit `RAISE` — required per F-E; also in the `ERR_REFUNKPRC` branch, where the module IS loaded). Belt-and-braces: flag-guarded unload in `tgMainCycle`'s handler | `tgRunTgsProgram` + `tgMainCycle` handlers | F-B, F-E |
| I4 | Frame parse failure ABORTS the program (decision revised 2026-08-28: cam skip → abort, matching the weld case): `nTG_DoCapture:=2` (R_C_F, robot-side sentinel checked by the .tgs) / `nTG_WeldStatus:=2` (R_W_F), set after completing the flag exchange (the HMI serves the full request sequence; cutting it short would desync) — HMI sees a normal abort choreography ending in R_E. **Implemented 2026-08-28**; fault-injection knobs `corrupt_cam_frame`/`corrupt_weld_frame` + CLI arg 5 (`corrupt-cam`/`corrupt-weld`) in `abb_server.py`, 2 fake-robot tests (29 total green). **Both sides VC-validated 2026-08-28**: corrupt-weld (no request 14, R_E pose exactly the predicted `[[1000,0,600],…]`) and corrupt-cam on the revised abort build (order `10,5,1,100`, `do capture = 2`, no captures; R_E pose verified arithmetically as the torch TCP at the capture-1 joints — 0.007 mm) | `TG_ReqCamFrame`, `TG_ReqWeldFrame` | F-C |
| I5 | **deferred by decision 2026-08-28** (leave as-is for FANUC parity; keep documented for later) — `PERS num nTG_RecvTimeout` (0 = wait forever, the default → behavior unchanged; >0 = seconds) resolved by a `tgRecvTime()` helper used as `\Time` on the two receive sites (`tgSendAck` ack, `tgPromptRecv` payload); `SocketAccept` keeps `WAIT_MAX` (waiting for a connection forever is correct). `ERR_SOCK_TIMEOUT` → cycle abort via the existing handler chain. The 0-sentinel avoids `WAIT_MAX` as a `PERS` initializer entirely. Sketch in §5.1 | helpers in `TG_Comms` | F-D (tunable) |

| I7 | **implemented + VC-validated 2026-08-28** (added after the §12 kill test exposed F-F) — `tgCycleAbort` in `TG_Comms.sys`: wire helpers `tgSendAck`/`tgPromptRecv` get catch-all `ERROR` handlers → log + `SocketClose` both + `ExitCycle`; `main()` resets `tg_module_loaded`. Kill test: recovery lines + `TG: main started` + next session served with no intervention. Mid-prog-sel kills now also recover through this path (regression check optional, same mechanism at shallower depth) | `TG_Comms.sys` helpers + `TG_Main.mod` `main()` | F-F, B1 |

~~I6 — cycle/error counters~~ **declined 2026-08-28** ("no cycle diagnostics
for now"); the Operator Window log is the soak-run record. E6's pass criterion
changes to: 50/50 clean cycles by transcript count + no Operator Window errors.

### 5.1 I5 sketch (agreed shape, not yet implemented)

```rapid
! Receive timeout for every HMI exchange, in seconds. 0 (default) = wait
! forever (FANUC parity; captures/operator interaction may take arbitrarily
! long). Finite value on the real cell unblocks the robot when the HMI PC
! dies without closing TCP (no RST). FlexPendant-editable.
PERS num nTG_RecvTimeout:=0;

LOCAL FUNC num tgRecvTime()
    IF nTG_RecvTimeout>0 RETURN nTG_RecvTimeout;
    RETURN WAIT_MAX;
ENDFUNC

! in tgSendAck / tgPromptRecv:
SocketReceive tg_client_socket,\Str:=...,\Time:=tgRecvTime();  ! was WAIT_MAX
```

Explicitly **not** planned: mid-cycle resume, per-request retry, robot-initiated
reconnect (robot is the server; FANUC parity), handling of system-level stops
(§2.E), and any HMI-side changes.

## 6. Validation plan

Python side first (fake-robot executable spec can't inject RAPID errors, so
this needs the VC):

1. `abb_server.py --die-at <point>` flag (drop the TCP connection after a named
   exchange: `after-accept`, `mid-prog-sel`, `mid-frame`, `mid-capture-wait`,
   `after-r-e`) to make VC fault injection reproducible, plus `--cycles N`
   soak mode. (Python tests cover the flag's own behavior.)
2. VC experiment table (each row: injection point → expected Operator Window
   sequence → expected next-cycle behavior):
   - E1 kill at `mid-prog-sel` → `TG: cycle error, ERRNO=…` → disc → accept; next cycle clean.
   - E2 kill at `mid-frame` (inside .tgs) → same, **plus** next cycle must log the I2/I3 UnLoad path, run the *new* file, and cycle 2 transcripts must match a clean run.
   - E3 kill 5× in a row at `after-accept` → loop must survive past the old retry limit (proves I1).
   - E4 e-stop mid-`R_C` wait, resume with HMI still connected → session completes; transcript identical to a clean run.
   - E5 controller warm restart mid-session → observe the ERRNO, confirm recovery after restart-at-main; document the observed behavior.
   - E6 soak: `--cycles 50` unattended; pass = 50/50 clean cycles, counters (I6) consistent, no Operator Window errors.
3. Frame-abort check (I4): serve a garbage frame payload once → robot must
   answer the R_C_F choreography with `do_capture` treated as 0 and reach `R_E`
   cleanly; HMI transcript shows a normal skip. *(Implemented as
   `abb_server.py` arg 5 `corrupt-cam`/`corrupt-weld` — VC procedure in
   [robotstudio_setup.md](robotstudio_setup.md) §11.)*
4. **E7 — `Load \Dynamic` timing** (feeds the F-B addendum decision): generate
   a synthetic .tgs module of realistic worst-case size (thousands of move
   lines), time `Load`/`UnLoad` on the VC with `ClkStart`/`ClkRead` around the
   `Load` in a scratch PROC. If worst-case load ≲ 1 s → keep the
   always-reload lifecycle, close the topic; if multiple seconds → spec the
   ftp-status-2 optimization.

## 7. Open decisions (your call before I code)

1. ~~**B2/B3 timeout default**~~ — **decided 2026-08-28: don't implement I5
   for now** (leave as-is for FANUC parity); this section stays as the design
   record for when a real cell needs it.
2. ~~**I4 abort flavor for R_W_F**~~ — **decided 2026-08-28: abort**
   (`nTG_WeldStatus:=2`); implemented. **Revised same day (team decision):
   the cam frame aborts too** — `nTG_DoCapture:=2` robot-side sentinel (wire
   still carries 0/1), `.tgs` checks it before the capture branch;
   implemented + fake-robot tested.
4. ~~**I2/I3 go/no-go**~~ — **decided 2026-08-28: go**; implemented (see I2/I3
   rows). E7 (always-reload `Load` timing) remains open as a separate
   measurement.
3. ~~**I6 counters**~~ — **decided 2026-08-28: no** (Operator Window log is
   enough).
