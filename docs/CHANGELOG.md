# Changelog

Lap time is the second (flying) lap. Reference: real qualifying lap, 88.09 s.

| Version | Date | Lap time | Gap to real | Main change |
|---|---|---|---|---|
| v1 | 2026-10-28 | 126.58 s | +38.5 s | First validated closed-loop lap. Centerline following; Cl 1.8 (ClA 2.7 m2); open differential; single gear. |
| v2 | 2026-10-29 | 109.29 s | +21.2 s | Minimum-curvature race line as the driver's reference; finer curvature smoothing; braking 20 m/s2; regression test; comparison with real telemetry aligned by position. |
| v3 | 2026-10-30 | 102.96 s | +14.9 s | Friction circle in the torque and brake limiters; speed-profile grip 1.5 to 1.7. |
| v4 | 2026-10-31 | 94.71 s | +6.6 s | Aerodynamics ClA 4.8 m2; wheel FZMAX 15 000 N; limited slip differential; air temperature 288.15 K. |
| v5 | 2026-11-01 | 95.28 s | +7.2 s | Engine map scaled to about 750 kW; 8-speed gearbox with shift logic; acceleration table calculated from the model instead of measured. |
| v6 | 2026-11-01 | 94.91 s | +6.8 s | Active aero straight mode (CdA 0.85 m2) through the external force port; top speed 302 to 316 km/h. |
| v7 | 2026-11-08 | **94.93 s** | **+6.84 s** | No change in behavior. One source of parameters (`f1_vehicle_params`) read by the speed profile, the block dialogs and the control functions; lap-time sensitivity sweep of the full model; internal-states view (per-wheel load and forces, utilization, load transfer, body slip).; interactive corner map (`f1_corner_map`). |

## Post-v6 (no change to results)

- Sector map and three-figure telemetry dashboard (`f1_sector_map.m`, `f1_telemetry.m`).
- `f1_speed_profile` called from the entry point `f1_vehicle_params`.
- Prototype files retired; folders reorganised; model top level, subsystem layout and port names cleaned up.
- Documentation package: README, technical report, architecture diagrams, parameter register, requirements, decision log.

Notes
- v3 was 102.45 s with the friction-circle functions using h = 0.275 m and 102.96 s with 0.35 m. 0.35 m is the value used by the Vehicle Body block and by all the results, so 102.96 s is the consistent one. The parameter script had 0.275 m, which did not affect the results; it was set to 0.35 m for consistency.
- The final run of the repository gave 94.93 s (+6.84 s, +7.8 %) for the evaluated lap, against 94.91 s in the v6 regression run; the difference is inside the test tolerance and was not investigated. Figures and tables in the README and the report use the final run.
- Between v3 and v4 two changes were made together (aero and wheel FZMAX), so their individual effects are not separated.
