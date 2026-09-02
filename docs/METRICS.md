# Metrics

Every key `eval/metrics.py` produces.  These names are the contract with
`config/criteria.yaml`; renaming one breaks a gate, so add rather than rename.

"Final" means the mean over the last 2 seconds of the run.

## Sanity

| metric | meaning |
|---|---|
| `nan_count` | non-finite values anywhere in control.csv |
| `diverged` | NaN, tilt > 60 deg, \|z\| > 1 m, or any current > 20 A |
| `fell_over` | peak tilt magnitude > 45 deg |
| `current_max_A`, `current_mean_A`, `power_mean_W` | servo load |

## Posture

| metric | meaning |
|---|---|
| `final_abs_roll_deg`, `final_abs_pitch_deg`, `final_abs_tilt_deg` | estimated attitude at the end |
| `max_abs_tilt_deg`, `peak_early_tilt_deg` | worst tilt, and worst in the first 30% |
| `settling_time_s` | last time the tilt was outside a band around its final value |
| `roll_improvement_ratio`, `tilt_improvement_ratio` | early peak / final value |

## Load

| metric | meaning |
|---|---|
| `total_force_mean_N` | sum of the four FSRs, final.  Should be the robot's weight |
| `final_force_cv` | std/mean across the four feet, final |
| `min_foot_force_final_N` | least loaded foot -- below the contact threshold means a foot is off the ground |
| `final_forces_N` | the four values, for the report |

## Objective (memo section 23, 28)

| metric | meaning |
|---|---|
| `J_first_quintile`, `J_last_quintile`, `J_improvement_ratio` | first fifth / last fifth, whole run |
| `J_trend_tau`, `J_trend_p_value` | Mann-Kendall trend, series subsampled to 200 points first (consecutive 100 Hz samples are not independent).  For a dither run, restricted to the search phase (see `gradient_sign_consistency` below) -- the whole run tests for a trend across a long, correctly-flat hunting tail too and reads a working search as "J never decreased" (finding #16).  Non-dither runs get the whole series, unchanged |
| `J_final` | final value, whole run |

## Dither (memo section 27-30)

| metric | meaning |
|---|---|
| `dither_update_count` | gradient updates completed |
| `dither_search_phase_updates` | how many of those updates `gradient_sign_consistency` (and `J_trend_p_value` above) actually scored |
| `gradient_sign_consistency` | over the **search phase**: fraction agreeing with the per-joint modal sign.  A converged dither alternates on purpose, so scoring the whole run punishes success.  Search phase = per joint, up to and including the first update where that joint's own RPROP step reaches its own empirical floor (the point the search itself gave up on a reliable sign and shrank to minimum) -- falls back to the first 60% of that joint's updates if the step never bottoms out.  Replaced a flat "first 60% of updates" in finding #14; that fraction was implicitly calibrated against mock's slower convergence and mis-scored isaaclab's faster one (finding #16: mock's search on 03_dither ran ~10 of 33 updates before hunting, isaaclab's ~2-3, because the real optimum needed was ~10x smaller) |
| `gradient_sign_consistency_full` | same over the whole run, reported not gated |
| `dither_offset_norm_final` | how far the search moved the command |

## State machine (memo section 32-34, 39)

| metric | meaning |
|---|---|
| `sm_min_target_leg_force_N` | how close the unload got to zero |
| `sm_final_target_leg_force_N` | load after reloading |
| `sm_motion_verified` | the section 33 foot-IMU check passed |
| `sm_returned_to_stand`, `sm_cycles`, `sm_aborted`, `sm_states_visited` | sequencing |
| `max_abs_tilt_during_lift_deg` | worst tilt while the leg was up |
| `sm_max_foot_lift_m` | actual lift, from truth.csv -- offline only |
| `sm_foot_forward_displacement_m` | commanded forward offset at the end |

## Faults (memo section 35-37)

| metric | meaning |
|---|---|
| `fault_planned`, `fault_detection_count`, `fault_classes_detected` | bookkeeping |
| `fault_detected` | anything flagged at all |
| `fault_false_alarm_count` | flags raised **before** the injection time |
| `fault_class_correct` | a flag after injection matches the expected class |
| `fault_joint_correct` | ...and named the right joint |
| `fault_detection_latency_s` | first correct flag minus injection time.  `inf` if never |

## Ground truth comparison (memo section 43) -- offline only

| metric | meaning |
|---|---|
| `roll_estimate_rmse_deg`, `pitch_estimate_rmse_deg` | IMU-only estimate vs simulator truth |
| `roll_estimate_bias_deg`, `pitch_estimate_bias_deg` | mean error |
| `foot_force_rmse_N` | FSR reading vs true contact force |

This block is the quantitative answer to "how far can we get without measuring
joints?", and it is the one thing that must never be fed back into the
controller.

## Cross-run

| metric | meaning |
|---|---|
| `isolation_violations` | source-scan hits in the controller-side modules (`eval/isolation.py`).  Part of the phase 1 gate |
