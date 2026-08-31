# abb_support — FANUC → ABB port prototype

Client-server prototype adding **ABB (IRC5, RobotWare 6.15, IRB 4600-20/2.5)**
support to the TetraGen welding system: the RAPID equivalent of the FANUC
KAREL request programs (`TGMainKL`, `R_C_F`, `R_W_F`, …) plus a Python
stand-in for the HMI's socket side.

**Status: v1 prototype complete** — priority request set ported and validated
end-to-end against a RobotStudio virtual controller (2026-08-28).

## Layout

| Path | What |
|---|---|
| `abb/rapid/TG_Comms.sys` | Request library: socket lifecycle, protocol helpers, all TG_Req* PROCs, shared PERS state (the FANUC register-map equivalent) |
| `abb/rapid/TG_Cell.sys` | Cell-hardware macros (FANUC's utility .ls programs): `TG_CamOpen`/`TG_CamClose` drive the camera flap on a dummy DO `doTG_Camera` |
| `abb/rapid/TG_Main.mod` | Main loop (TGMainKL equivalent): accept HMI → prog-sel → file transfer → `Load \Dynamic` + late-bound call of the .tgs program + edit-preserving `UnLoad` (operator touch-ups are staged in `HOME:/TGS/edited/` for retrieval) |
| `abb/rapid/TGS/TD05Test.mod` | Sample .tgs program (mirrors the FANUC TD05tRJYQd call order) |
| `hmi_prototype/abb_server.py` | Python HMI prototype: serves all robot-initiated requests with dummy data; Euler↔quaternion pose codec; delivers the .tgs module to `HOME:/TGS/` by direct copy (VC fallback, kept) or RWS upload |
| `hmi_prototype/rws_client.py` | Minimal Robot Web Services client (stdlib, digest auth): fileservice GET/PUT/DELETE, module save, PERS read/write |
| `hmi_prototype/tg_retrieve.py` | "Retrieve robot program from controller" stand-in: fetch the staged touch-up → validate → backup → adopt as master → cleanup |
| `hmi_prototype/test_phase*.py` | 63 automated tests incl. fake-robot and fake-RWS executable specs of the RAPID choreography and retrieval flow |
| `docs/abb_port_plan_v1.md` | **The** design doc: extracted FANUC protocol, ABB architecture, decisions log, RAPID gotchas learned on the controller |
| `docs/rapid_validation_findings_v1.md` | The three defects found during VC validation — root causes and the rules they imply for the production port |
| `docs/robotstudio_setup.md` | How to build the VC and run each phase's smoke test |
| `docs/fanuc_hmi_request_program_calls_v1.md` | FANUC request-number table (reference) |
| `docs/abb_program_touchup_and_retrieval_v1.md` | Pendant position touch-ups and pulling the edited program back to the HMI — why FANUC's `MD:` FTP trick has no ABB equivalent, and the RWS-based replacement |
| `resources/FANUC/` | Source material: KAREL/LS programs, sample .tgs export, Python socket sample |

## Quick start

1. RobotStudio VC per [docs/robotstudio_setup.md](docs/robotstudio_setup.md)
   (RW 6.15.08, IRB 4600-20/2.50, option **616-1 PC Interface**); load
   `TG_Comms.sys` + `TG_Main.mod` into `T_ROB1`, PP to main, Start.
2. `python hmi_prototype/abb_server.py 127.0.0.1 2000 2 "<solution>\Virtual Controllers\<name>\HOME"`
3. Tests: `python -m unittest discover -s hmi_prototype`

## Key design points (details in the plan doc)

- Robot is the TCP **server** (port 2000, a PERS setting); the HMI connects and
  the robot initiates every request — same choreography, prompts, acks and
  scalar formats as the FANUC wire protocol.
- **Frames deviate deliberately**: one message carrying a RAPID pose literal
  `[[x,y,z],[q1,q2,q3,q4]]` (normalized quaternion). FANUC-convention Euler
  conversion lives on the PC side (Python now, C++ `ABBRobot` later).
- .tgs modules are delivered to `HOME:/TGS/` (FTP on the real cell), loaded
  dynamically, called by name via late binding, and unloaded — they see the
  request PROCs and shared PERS data through RAPID's task-wide global scope.
- The report tool/frame is passed **explicitly** on each request call
  (`TG_ReqCamFrame \Tool:=tTG_Cam \WObj:=wobjTG_Cam`, plan §7.6 style b); a
  PERS parameter is a live persistent reference, so a received frame takes
  effect on the very next pose report. The FANUC-style modal numbers
  (`nTG_ActTool`/`nTG_ActFrame`, the UTOOL_NUM/UFRAME_NUM equivalent) remain
  as a deprecated fallback for argument-less calls. Never copy wobjdata —
  it goes stale (plan §4.3).

## Out of scope for v1 (Phase 4 backlog)

Real FTP to a physical IRC5 (FTP option), `R_W_S`, touch-sense and
camera-calibration request families, RobotWare Arc welddata mapping, and the
C++ `ABBRobot : Robot` class in TGuideWeldingHMI. (`WaitRob \InPos` before
pose reports was promoted out of the backlog and is in `tgSendPose`.)
