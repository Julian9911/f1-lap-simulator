# Parameter register

Every parameter of the model (version v6), where it is used, and how trustworthy it is. The **Status** column is the important one: it separates what comes from the regulations or public data from what is calibrated or simply estimated.

## Status legend

| Status | Meaning |
|---|---|
| REGULATION | Taken from the 2026 technical regulations |
| SOURCED | Value or range from a cited public source (mostly secondary) |
| TELEMETRY | Derived from the public reference telemetry |
| CALIBRATED | Tuned against a reference measurement; not a published value |
| DERIVED | Computed from other parameters |
| ESTIMATED | Engineering judgement; no source |
| BLOCK | Simulink block setting or block default |
| DESIGN | Algorithm design choice |

## Summary

| Status | Parameters |
|---|---|
| REGULATION | 2 |
| SOURCED | 3 |
| TELEMETRY | 2 |
| CALIBRATED | 3 |
| DERIVED | 4 |
| ESTIMATED | 20 |
| BLOCK | 4 |
| DESIGN | 5 |

Total: 43 entries.

## Register


### Vehicle

| ID | Parameter | Value | Unit | Status | Where used | Source / rationale |
|---|---|---|---|---|---|---|
| V-01 | Vehicle mass | 770 | kg | REGULATION | veh.m | 2026 minimum mass, driver included, fuel excluded. |
| V-02 | Wheelbase | 3.4 | m | REGULATION | veh.wheelbase | 2026 maximum wheelbase. |
| V-03 | Track width, front / rear | 1.6 / 1.6 | m | ESTIMATED | veh.track_f, veh.track_r; friction-circle functions | Not officially published. |
| V-04 | CG height (= load-transfer height h) | 0.35 | m | ESTIMATED | veh.cg_height; Vehicle Body block; h_cg in f1_engine_cmd and f1_pedals_to_cmd | Higher than the 0.25-0.30 m typical of vehicle-dynamics literature; kept because every result of the project was obtained with it. The block and the friction-circle estimates must use the same value (same in all three places). Changing only the control functions to 0.275 m moved the v3 lap by 0.5 s. |
| V-06 | Rear weight fraction | 0.55 | - | ESTIMATED | veh.weight_dist_r | About 45/55 is typical for single-seaters. |
| V-07 | CG to front / rear axle (a / b) | 1.87 / 1.53 | m | DERIVED | veh.a, veh.b | a = wheelbase x 0.55, b = wheelbase x 0.45. |
| V-08 | Yaw moment of inertia Izz | 1187 | kg m2 | DERIVED | veh.Izz | Izz = m k_yaw^2 with k_yaw = wheelbase / sqrt(7.5): a rule of thumb for F1 cars, whose mass is concentrated near the CG (a sedan is about 3). |

### Aerodynamics

| ID | Parameter | Value | Unit | Status | Where used | Source / rationale |
|---|---|---|---|---|---|---|
| A-01 | Downforce coefficient Cl | 3.2 | - | SOURCED | Vehicle Body block | With Af 1.5 m2 gives ClA 4.8 m2, inside the 3.2-6.4 m2 range of secondary sources (see References). |
| A-02 | Frontal area Af | 1.5 | m2 | SOURCED | Vehicle Body block | Secondary sources, about 1.5 m2. |
| A-03 | Drag coefficient Cd, corner mode | 0.73 | - | ESTIMATED | Vehicle Body block | CdA 1.095 m2. Secondary sources range 0.7-0.85. |
| A-04 | CdA, straight mode | 0.85 | m2 | CALIBRATED | f1_aero_active (CdA_recta), f1_speed_profile (CdA) | Tuned to the real 90th-percentile acceleration at 70-90 m/s (range 0.8-0.9). Not a published value. |
| A-05 | Mode switch thresholds | straight if /ay/ < 3 m/s2 and vx > 40 m/s; corner again if /ay/ > 6 m/s2 or vx < 40 m/s |  | ESTIMATED | f1_aero_active | Hysteresis; ay approximated by |r vx|. |
| A-06 | Air density | 1.225 (Tair 288.15 K) | kg/m3 | DERIVED | Vehicle Body environment; f1_speed_profile; f1_aero_active | ISA sea level, 15 C. |
| A-07 | Aero balance Cpm | 0 | - | ESTIMATED | Vehicle Body block | Not modelled. |

### Tires

| ID | Parameter | Value | Unit | Status | Where used | Source / rationale |
|---|---|---|---|---|---|---|
| T-01 | Peak lateral grip Fy/Fz, front / rear | 1.7 / 1.8 | - | ESTIMATED | f1_lateral_tire_table | Secondary sources quote mu of about 1.5-1.8; the rear peak is higher so the front saturates first (stable understeer). |
| T-02 | Initial slope of Fy/Fz, front / rear | 20 / 24 | 1/rad | ESTIMATED | f1_lateral_tire_table | F1 tires are about 20-25. |
| T-03 | Curve shape C / E | 1.3 / -2 | - | ESTIMATED | f1_lateral_tire_table | Early peak with a soft fall-off (no cliff). |
| T-04 | Nominal wheel load Fznom | 4000 | N | BLOCK | Vehicle Body block; f1_lateral_tire_table | Must match in both places. |
| T-05 | Longitudinal Magic Formula peak PDX1 | 1.7 | - | ESTIMATED | 4 wheel blocks | Same order as the lateral peak. |
| T-06 | Wheel FZMAX | 15000 | N | ESTIMATED | 4 wheel blocks | Raised from 8600 N: with ClA 4.8 the wheel loads reach about 10 kN and wheel lock-ups appeared. |
| T-07 | Effective rolling radius Re | 0.3515 | m | ESTIMATED | Wheel blocks; control functions | Gear ratios were derived from telemetry at this radius. |
| T-08 | Rolling resistance QSY1 / QSY3 / QSY7 (Fz0) | 0.00699 / 0.00154 / 0.894 (4300 N) | - | BLOCK | Wheel blocks; f1_speed_profile | QSY4 term kept as in the block; the speed profile omits it. |
| T-09 | Brake system | max 14 000 N m total; front bias 0.6; disc mu 0.4, bore 0.05 m, Rm 0.177 m, 2 pads |  | ESTIMATED | f1_pedals_to_cmd; wheel blocks | Disc parameters must equal those of the 4 wheel blocks. |
| T-10 | Grip limits used by the controllers | traction 1.4; braking 1.4; tire peak 1.7; floor 0.3 | - | ESTIMATED | f1_engine_cmd, f1_pedals_to_cmd | Friction circle: mu_x = min(mu, sqrt(mu_pk^2 - mu_y^2)). |

### Powertrain

| ID | Parameter | Value | Unit | Status | Where used | Source / rationale |
|---|---|---|---|---|---|---|
| P-01 | Engine torque map scale | 0.91 | - | CALIBRATED | Mapped SI Engine block; f1_speed_profile (eng_scale) | About 740 kW at 12 000 rpm; regulation total is 750 kW (400 kW ICE + 350 kW MGU-K). Must match in both places. |
| P-02 | Peak engine torque | 616 | N m | DERIVED | Mapped SI Engine | From the scaled map. |
| P-03 | Rev limiter taper | 14 000 -> 15 000 | rpm | ESTIMATED | f1_engine_cmd; f1_speed_profile | 15 000 rpm is the 2026 maximum. |
| P-04 | Gear ratios, gears 2-8 | 2.893, 2.407, 1.965, 1.680, 1.473, 1.276, 1.139 | - | TELEMETRY | Transmission block; f1_engine_cmd; f1_shift_logic; f1_speed_profile | Median rpm/speed per gear at Re 0.3515 m. First gear is unused (flying lap). Must match in four places. |
| P-05 | Final drive / transmission efficiency | 4.0 / 0.9 | - | ESTIMATED | Transmission block; control functions | Overall ratio is set by P-04 together with this value. |
| P-06 | Shift thresholds | up 11 000, down 8 500 | rpm | TELEMETRY | f1_shift_logic; f1_speed_profile | Real up-shifts: median about 10 900 rpm (90% between 10 300 and 11 400). |
| P-07 | LSD clutch preload | 2500 | N | ESTIMATED | Limited Slip Differential block | 4 discs, Reff 0.08 m, tauC 0.1 s. Lap time is insensitive from 500 to 12 500 N. |
| P-08 | Transmission dynamics | Jout 0.1 kg m2, bout 0.0025, tau_s 0.01 s |  | BLOCK | Transmission block | tau_s causes torque spikes at each shift (known defect). |

### Driver

| ID | Parameter | Value | Unit | Status | Where used | Source / rationale |
|---|---|---|---|---|---|---|
| D-01 | Stanley look-ahead / position gain | 40 m / 2.5 |  | BLOCK | Predictive Driver block | Configuration frozen in v1. |
| D-02 | Reference-point search window | +/- 50 points (100 m) |  | DESIGN | f1_ref_pose | Avoids locking onto another part of the circuit that passes nearby. |

### Track

| ID | Parameter | Value | Unit | Status | Where used | Source / rationale |
|---|---|---|---|---|---|---|
| K-01 | Circuit centerline | OpenStreetMap, about 5857 m, resampled every 2 m |  | SOURCED | f1_track_curvature | OpenStreetMap contributors, ODbL. |
| K-02 | Reference (race) line | minimum curvature, +/- 4 m half-width, about 5788 m |  | ESTIMATED | f1_make_raceline.py; f1_load_raceline | OSM has no track width; 4 m is a reasonable, conservative value. |
| K-03 | Curvature smoothing for the speed profile | moving max then moving mean, 5 points (10 m) |  | DESIGN | f1_speed_profile | Removes OSM noise while keeping each corner's peak. |

### Speed profile

| ID | Parameter | Value | Unit | Status | Where used | Source / rationale |
|---|---|---|---|---|---|---|
| S-01 | Cornering grip used by the profile | 1.7 | - | ESTIMATED | f1_speed_profile (mu_v) | Equals the tire peak. Limit speed: v = sqrt(mu g / (kappa - mu k)), k = 0.5 rho ClA / m. |
| S-02 | Braking deceleration | 20 | m/s2 | CALIBRATED | f1_speed_profile (a_brake) | Set to what the simulated car achieves (measured about 20 m/s2); the real car brakes at about 22 m/s2. |
| S-03 | Speed cap | 90 | m/s | DESIGN | f1_speed_profile (v_cap) | About the maximum speed of the car. |
| S-04 | Traction limit used for acceleration | mu 1.4, rear load 55% |  | ESTIMATED | f1_speed_profile | Acceleration table is computed, not measured: engine map x gearbox - aero - rolling, capped by traction. |

### Timing

| ID | Parameter | Value | Unit | Status | Where used | Source / rationale |
|---|---|---|---|---|---|---|
| L-01 | Finish-line detection | lateral tolerance 30 m; grace period 2 s |  | DESIGN | f1_lap_timer | Avoids false crossings at the start and from other track sections. |

### Simulation

| ID | Parameter | Value | Unit | Status | Where used | Source / rationale |
|---|---|---|---|---|---|---|
| X-01 | Stop time | 260 | s | DESIGN | Model configuration | At least two full laps; the second (flying) lap is evaluated. |

## Open items

- **OI-1** Values repeated by hand in several places (CdA, gear ratios, torque scale, final drive, Re). A single parameter file would remove the risk of silent drift.

The same data is available as [parameter_register.csv](parameter_register.csv).
