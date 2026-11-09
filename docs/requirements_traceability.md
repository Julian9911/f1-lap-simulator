# Requirements and traceability

This table links each requirement to the way it is verified and to its current status (model version **v7**). Requirements marked **automated** are checked by `f1_regression_test.m` after every change; the others are verified by analysis, demonstration or inspection.

The baseline is the lap time of the closed-loop model (94.9 s); the quasi-steady-state profile (about 91.2 s) is the ideal reference it follows and is not a requirement.

The verification methods follow the usual systems-engineering vocabulary: **Test** (automated check), **Analysis** (comparison against reference data), **Demonstration** (run it and observe), **Inspection** (read the artifact).

## Functional and behavioural requirements

| ID | Requirement | Rationale | Method | Evidence / check | Status (v7) |
|---|---|---|---|---|---|
| R-01 | The model shall complete at least two laps of Silverstone, unattended, in a 260 s simulation. | A second lap is the first one that starts at speed and is the comparable lap. | Test (automated) | Regression criterion "Complete laps >= 2" | PASS |
| R-02 | The evaluated lap time shall stay within 1.5 s of the accepted baseline (94.9 s). | Detects any change that alters the behaviour of the car. | Test (automated) | Regression criterion "Lap time" | PASS: 94.93 s |
| R-03 | Between consecutive complete laps, lap times shall differ by less than 1.5 s. | Repeatability. Only evaluated when three or more laps are simulated. | Test (automated) | Regression criterion "Repeatability" | Not evaluated with 2 laps |
| R-04 | The maximum lateral distance to the reference line shall be below 3.0 m, and its 95th percentile below 1.8 m. | The car must follow the planned line (the line has a 4 m half-width). | Test (automated) | Regression criteria "Lateral error max" and "p95" | PASS: 2.02 m / 1.54 m |
| R-05 | The distance travelled in a lap shall be within +/-3 % of the circuit length. | Detects shortcuts and false laps. | Test (automated) | Regression criterion "Distance / circuit" | PASS |
| R-06 | All logged signals of the evaluated lap shall be finite. | Numerical health. | Test (automated) | Regression criterion "No NaN/Inf" | PASS |
| R-07 | Maximum speed shall stay below 342 km/h, and the car shall not stop during a complete lap (minimum speed above 36 km/h). | Physical plausibility. | Test (automated) | Regression criteria "Maximum speed" and "No stops" | PASS: 316 km/h |

## Validation requirements (against the real reference lap)

These are project-level targets for the documentation. They are verified by analysis with `f1_telemetry.m` and `f1_sector_map.m`; they are not part of the automated regression.

| ID | Requirement | Method | Evidence | Status (v7) |
|---|---|---|---|---|
| V-1 | The simulated lap time shall be within 10 % of the reference lap (88.09 s). | Analysis | Lap 94.93 s, +6.84 s (+7.8 %) (regression run v7: 94.93 s) | Met |
| V-2 | The simulated top speed shall be within 2 % of the reference (317 km/h). | Analysis | 316 km/h | Met |
| V-3 | Per-sector times shall be reported, with the loss by corner speed class. | Analysis | `f1_sector_map` and `f1_telemetry` | Met (sector bounds approximate) |
| V-4 | Differences with the reference shall be explained or declared. | Inspection | Technical report, section 7 and `decision_log.md` | Partly explained: 3.7 s tracking gap (profile 91.2 s vs closed loop 94.9 s) is measured; the rest is parameter uncertainty (see limitations) |

## Process and documentation requirements

| ID | Requirement | Method | Evidence | Status (v7) |
|---|---|---|---|---|
| R-08 | A single entry point shall initialize the workspace (`f1_vehicle_params`), including the speed profile. | Demonstration | Clear workspace, run `f1_vehicle_params`, simulate, run regression | Met (speed profile added to the entry point) |
| R-09 | Every parameter shall have a source or an explicit status. | Inspection | `parameter_register.md` (45 entries, 8 status classes) | Met |
| R-10 | Model limitations shall be declared. | Inspection | README and technical report, "Limitations" | Met |
| R-11 | Results shall be reproducible from public data only. | Inspection | Data sources in README and report; no team data | Met |
| R-12 | Analysis tools shall run without modifying the model. | Demonstration | `f1_telemetry`, `f1_sector_map`, `f1_track_animation`, `f1_internal_states` read `out` only; `f1_sensitivity_sweep` restores the workspace on exit | Met |
| R-13 | Every physical parameter shall be defined in one place only. | Inspection + Test | `f1_vehicle_params.m` (structs `veh`, `aero`, `tire`, `pwr`, `brk`); regression 8/8 unchanged after the refactor (D-14) | Met |
| R-14 | The influence of the main parameters on lap time shall be quantified. | Analysis | `f1_sensitivity_sweep` (+5 % on mu, ClA, mass, engine, CdA); README table (D-16) | Met (CdA effect below the resolution of the model) |
| R-15 | Quantities that cannot be measured on a real car shall be separated from those that can. | Inspection | `f1_internal_states` (per-wheel Fz, Fx, Fy, utilization, g-g) and README section "Internal states" (D-17) | Met (signals come from the Vehicle Body Info bus, not Combined Slip outputs) |

## Traceability: requirement to regression criterion

| Regression criterion (`f1_regression_test.m`) | Requirement |
|---|---|
| Complete laps >= 2 | R-01 |
| Lap time (94.9 +/- 1.5 s) | R-02 |
| Repeatability between laps (< 1.5 s) | R-03 |
| Lateral error max (< 3.0 m) and p95 (< 1.8 m) | R-04 |
| Distance travelled (+/-3 %) | R-05 |
| No NaN/Inf | R-06 |
| Maximum speed (< 342 km/h) and no stops (> 36 km/h) | R-07 |

When the model is improved on purpose (for example more grip), update the baseline in `f1_regression_test.m` and record the change in `CHANGELOG.md`.
