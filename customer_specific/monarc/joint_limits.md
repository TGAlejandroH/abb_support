# Monarc cell — joint limits (read-only survey, 2026-08-31)

Source of truth: `SYSPAR/MOC.cfg` `ARM` section of the customer backup
`D:\ABB\Monarc_RS\Project\Controller Data\4600-803651_Virtual\` (RobotWare
6.16.0025). Cross-checked against the two other backups in the project and
against the RobotWare media-pool base configs — **all identical, so no joint
limit in this cell has been tightened from factory.**

`upper/lower_joint_bound` = the active working range. `upper/lower_joint_bound_max`
= the largest value the range may be widened to (rev-counter limit).

## Robot — `ROB_1`, IRB 4600-20/2.50 Type D (s/n 4600-803651)

| Axis | Arm | Working range (rad) | Working range (deg) | Max settable |
|---|---|---|---|---|
| 1 | `rob1_1` | −3.1416 … +3.1416 | −180 … +180 | same |
| 2 | `rob1_2` | −1.5708 … +2.618 | −90 … +150 | same |
| 3 | `rob1_3` | −3.1416 … +1.309 | −180 … +75 | same |
| 4 | `rob1_4` | −6.98132 … +6.98132 | −400 … +400 | ±1267 rad (±72 594°) |
| 5 | `rob1_5` | −2.0944 … +2.0944 | −120 … +120 | same |
| 6 | `rob1_6` | −6.98132 … +6.98132 | −400 … +400 | ±1152 rad (±66 005°) |

## Positioner — IRBP D600 L2000 D1200 Type A

| Mech unit | Joint | Logical axis | `extax` slot | Meas. node | Working range (rad) | Working range (deg) | Max settable |
|---|---|---|---|---|---|---|---|
| `STN1` | `ARM1` | 8 | `eax_b` | `pos1_2` | −3.159 … +3.159 | −181 … +181 | same |
| `STN1` | `PLATE1` | 9 | `eax_c` | `pos1_3` | −20 … +20 | −1145.92 … +1145.92 | ±1.25664E+06 rad (±200 000 rev) |
| `STN2` | `ARM2` | 8 | `eax_b` | `pos1_4` | −3.159 … +3.159 | −181 … +181 | same |
| `STN2` | `PLATE2` | 9 | `eax_c` | `pos1_5` | −20 … +20 | −1145.92 … +1145.92 | ±1.25664E+06 rad |
| `INTERCH` | `INTERCH_PLATE1` | 8 | `eax_b` | `pos1_3` | internal — see §Notes | | |
| `INTERCH` | `INTERCH_PLATE2` | 9 | `eax_c` | `pos1_5` | internal | | |
| `INTERCH` | `INTERCH` (index) | 10 | `eax_d` | `pos1_1` | internal | | |

## Notes

- **Logical axes are reused.** 5 positioner motors share 3 ADU-790A drives and
  only 3 logical axes. The same `extax` slot is a *different physical motor*
  depending on which mech unit is active: `eax_b`(8) = `ARM1` **or** `ARM2`
  **or** `INTERCH_PLATE1`; `eax_c`(9) = `PLATE1` **or** `PLATE2` **or**
  `INTERCH_PLATE2`; `eax_d`(10) = the index axis only. `STN1` and `STN2` can
  never be active simultaneously (contactors K1–K5).
- **`eax_a`, `eax_e`, `eax_f` are unused** — always `9E9` in this cell's data.
- **`INTERCH` bounds are not obtainable from the project.** They live in
  `SEC_D600_L2000_D1200_TYPEA_STN1.cfg.enc` (encrypted, loaded `-internal`);
  backups do not save it and the config editor does not expose it. Get them
  from ABB if ever needed.
- **`PLATE1`/`PLATE2` are `-independent_joint_on`** — hence the ±1145.92°
  range and the six-figure max; the plate is meant to spin freely and the
  angular-search correction depends on it.
- **All four positioner arms have `-deactivate_cyclic_brake_check_arm`** — CBC
  is disabled on the positioner.
- **SafeMove Pro adds no axis-range supervision.** Its configuration
  (seal 2E540591) carries only 3 Cartesian zones: `SafetyPerimeter` (keep-in,
  0.25 m/s), `ProductionZone` (4 m/s), `TipChange` (keep-out). So the only
  envelope restrictions beyond the table above are those 3 zones plus the 4
  stationary RAPID world zones armed from `POWER_ON` in `Utility.sys`.
