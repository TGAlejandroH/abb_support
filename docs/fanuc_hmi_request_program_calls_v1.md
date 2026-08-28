# FANUC HMI Request Program Calls (v1)

Reference table for the TetraGen HMI request protocol used by FANUC program export.
When the HMI mode is active in the FANUC Output Profile window (export mode
`standalone_hmi`), exported programs invoke small controller-side request routines via
`CALL` instructions. Each routine signals a numbered request to the TetraGen HMI, which
performs the corresponding action (camera work, touch-sense bookkeeping, weld parameter
lookup, etc.) and hands control back to the robot program.

This table is the authoritative mapping between HMI request numbers and the program
names called from exported FANUC programs. The request numbers themselves live on the
HMI side of the protocol and are not stored anywhere else in this repo.

## Request table

| Request Number | Program Name (CALL) | Description |
|---|---|---|
| 1 | `R_C_F` | Request Camera Frame | Frame-PR-writing routine
| 2 | `R_C` | Request Capture |
| 3 | `R_C_D` | Request Camera Done (for weld local captures) |
| 4 | `R_W_F` | Request Weld Frame | Frame-PR-writing routine
| 5 | `R_P_C` | Request Password Check |
| 6 | *FILE NOT IMPLEMENTED* | RequestTCPCalibration |
| 7 | `R_C_C` | Request Camera Calibration |
| 8 | `R_C_C_F` | Request Camera Calibration Frame | Frame-PR-writing routine
| 9 | `T_T_R_F` | Touch Test Frame Request | Frame-PR-writing routine
| 10 | `R_F_T` | Request File Transfer |
| 11 | `R_G_C_D` | Request Global Captures Done |
| 12 | *FILE NOT IMPLEMENTED* | RequestCheckerboardDetection |
| 13 | `R_W_S` | Request Welding Stats |
| 14 | `R_W_P` | Request Welding Parameters |
| 15 | *FILE NOT IMPLEMENTED* | RequestNextWeldRegistration |
| 16 | `R_TS_D` | Request Touch Sense Do Per Weld | Frame-PR-writing routine
| 17 | `R_TS_F` | Request Touch Sense Search Frame | Frame-PR-writing routine
| 18 | `R_TS_P` | Request Touch Sense Point Record | Frame-PR-writing routine
| 19 | `R_TS_END` | Request Touch Sense End | Frame-PR-writing routine
| 100 | `R_E` | Request End |

Entries marked *FILE NOT IMPLEMENTED* have a reserved request number on the HMI side
but no controller program file exists for them yet;

> **⚠ The "Frame-PR-writing routine" markers above are load-bearing.** They define the set of
> routines whose `CALL` invalidates the program shortener's UFRAME PR block dedup cache
> (`FanucTranslator.FRAME_PR_PREP_ROUTINE_NAMES_DEFAULT`; the profile-configurable subset also in
> `FanucRoutineSet.FRAME_PR_PREP_ROUTINE_FIELDS`). These routines rewrite the user-frame PR on the
> controller — never `UFRAME_NUM` — so the exporter must re-emit the next `UFRAME[n]=PR[n]` copy
> after calling one, even when byte-identical. If a routine gains or loses frame-PR writes, update
> the marker here AND the translator set in the same change; see the maintenance invariant in
> [product/fanuc_export.md](product/fanuc_export.md) (§ Redundant Frame/Tool Line Shortener).

## Where the program names live in code

- Routine-name defaults are defined on the `FanucRoutineNames` dataclass in
  [../tguide_lib/workflows/fanuc_output_profiles.py](../tguide_lib/workflows/fanuc_output_profiles.py)
  (e.g. `capture_focus_routine_name="R_C_F"`, `touch_finish_routine_name="R_TS_END"`).
  Some touch-sense names can be overridden through weld-planner config keys such as
  `FANUC_TOUCHSENSE_PREP_ROUTINE_NAME`.
- The `CALL` instructions are emitted by the FANUC program blocks in
  [../tguide_lib/robot_brand_program_blocks/fanuc_program_blocks.py](../tguide_lib/robot_brand_program_blocks/fanuc_program_blocks.py).
- Persisted per-profile routine names are stored in
  [../ProgramData/Workflows/weld_planner/fanuc_output_profiles.json](../ProgramData/Workflows/weld_planner/fanuc_output_profiles.json).
- In the `no_hmi` export modes, the profile blanks these routine names and no request
  `CALL`s are emitted; see
  [fanuc_output_profile_coordinated_motion_v1.md](fanuc_output_profile_coordinated_motion_v1.md)
  and [product/fanuc_export.md](product/fanuc_export.md) for the broader export
  contract.
