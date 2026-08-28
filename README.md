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
| `abb/rapid/TG_Main.mod` | Main loop (TGMainKL equivalent): accept HMI → prog-sel → file transfer → `Load \Dynamic` + late-bound call of the .tgs program + `UnLoad` |
| `abb/rapid/TGS/TD05Test.mod` | Sample .tgs program (mirrors the FANUC TD05tRJYQd call order) |
| `hmi_prototype/abb_server.py` | Python HMI prototype: serves all robot-initiated requests with dummy data; Euler↔quaternion pose codec; copies the .tgs module into the VC's `HOME:/TGS/` (FTP stand-in) |
| `hmi_prototype/test_phase*.py` | 27 automated tests incl. fake-robot executable specs of the RAPID choreography |
| `docs/abb_port_plan_v1.md` | **The** design doc: extracted FANUC protocol, ABB architecture, decisions log, RAPID gotchas learned on the controller |
| `docs/robotstudio_setup.md` | How to build the VC and run each phase's smoke test |
| `docs/fanuc_hmi_request_program_calls_v1.md` | FANUC request-number table (reference) |
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
- Active tool/frame is selected **by number** (`nTG_ActTool`/`nTG_ActFrame`,
  the UTOOL_NUM/UFRAME_NUM equivalent) and resolved live — never by copying
  wobjdata, which goes stale (see plan §4.3).

## Out of scope for v1 (Phase 4 backlog)

Real FTP to a physical IRC5 (FTP option), `R_W_S`, touch-sense and
camera-calibration request families, RobotWare Arc welddata mapping,
`WaitRob \InPos` before pose reports, and the C++ `ABBRobot : Robot` class in
TGuideWeldingHMI.
