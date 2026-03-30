/**
 * Enable fixed-wing MPC avoidance prototype.
 *
 * When disabled the module republishes incoming setpoints unchanged.
 *
 * @value 0 Disabled
 * @value 1 Enabled
 * @min 0
 * @max 1
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_INT32(FW_MPC_AVOID_EN, 0);

/**
 * Enable direct throttle override from MPC.
 *
 * When disabled, fw_mpc_avoidance leaves throttle control to TECS and publishes
 * only direct pitch. When enabled, the module also publishes throttle_direct
 * from the MPC thrust command.
 *
 * @value 0 TECS throttle
 * @value 1 MPC direct throttle
 * @min 0
 * @max 1
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_INT32(FW_MPC_THR_EN, 0);

/**
 * Active MPC prediction horizon length.
 *
 * This sets the number of prediction steps used by the MPC QP at runtime.
 * The compile-time solver/storage limit remains fixed in the controller.
 *
 * @min 2
 * @max 64
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_INT32(FW_MPC_HORIZON, 64);

/**
 * Internal MPC model integration step [s].
 *
 * Sets the integration step used by the internal legacy MPC model during rollouts.
 *
 * @unit s
 * @min 0.005
 * @max 0.08
 * @decimal 3
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_AVOID_DT, 0.05f);

/**
 * Hold last valid MPC setpoints after QP failure.
 *
 * While obstacle-triggered is active, publish the last valid MPC setpoints for this duration
 * when the current QP solve fails. This avoids immediate handover to nominal FW setpoints.
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_FAIL_HOLD, 1.20f);

/**
 * Distance hysteresis for MPC deactivation.
 *
 * When MPC is already active, it remains active until the nearest obstacle distance
 * exceeds (trigger distance + FW_MPC_ACT_HYS), unless timeout hold is still active.
 *
 * @unit m
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_ACT_HYS, 2.0f);

/**
 * Deactivation hold time after last trigger.
 *
 * Additional hold time for MPC activity after obstacle trigger becomes false.
 * Helps avoid rapid on/off flicker near the trigger boundary.
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_DEACT_T, 0.60f);

/**
 * Obstacle message freshness timeout.
 *
 * If no obstacle update arrives within this time, MPC obstacle activation is disabled.
 *
 * @unit s
 * @min 0.05
 * @max 5.0
 * @decimal 2
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_OBS_TO, 0.50f);

/**
 * Minimum activation distance to obstacle surface.
 *
 * MPC obstacle avoidance activates when distance to obstacle surface is below trigger distance.
 * Trigger distance is min(FW_MPC_OBS_TMAX, max(FW_MPC_OBS_DMIN, speed * FW_MPC_OBS_LKHD + FW_MPC_OBS_BIAS)).
 *
 * @unit m
 * @min 1.0
 * @max 200.0
 * @decimal 1
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_OBS_DMIN, 35.0f);

/**
 * Speed lookahead time for obstacle activation.
 *
 * Used in trigger distance formula: min(FW_MPC_OBS_TMAX, max(FW_MPC_OBS_DMIN, speed * FW_MPC_OBS_LKHD + FW_MPC_OBS_BIAS)).
 *
 * @unit s
 * @min 0.0
 * @max 15.0
 * @decimal 2
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_OBS_LKHD, 5.0f);

/**
 * Additional distance bias for obstacle activation.
 *
 * Used in trigger distance formula: min(FW_MPC_OBS_TMAX, max(FW_MPC_OBS_DMIN, speed * FW_MPC_OBS_LKHD + FW_MPC_OBS_BIAS)).
 *
 * @unit m
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_OBS_BIAS, 8.0f);

/**
 * Maximum activation distance to obstacle surface.
 *
 * Caps the speed-based trigger distance, so distant obstacles do not activate MPC
 * too early at high airspeed.
 *
 * @unit m
 * @min 5.0
 * @max 200.0
 * @decimal 1
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_OBS_TMAX, 80.0f);

/**
 * Additional horizontal planning margin for earlier turns.
 *
 * This value is added to obstacle radius/margin inside MPC constraints and activation logic.
 * Increasing this makes the aircraft start avoidance earlier.
 *
 * @unit m
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_OBS_PLAN, 3.0f);

/**
 * Obstacle proximity cost weight.
 *
 * Adds a soft repulsive cost around obstacles in MPC objective.
 * Higher values make the controller start turning away earlier.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_OBS_CW, 6.0f);

/**
 * Obstacle proximity cost distance from obstacle surface.
 *
 * Repulsive cost is active within this horizontal distance from obstacle buffered surface.
 *
 * @unit m
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_OBS_CD, 20.0f);

/**
 * Minimum waypoint tracking weight scale during strong avoidance.
 *
 * As obstacle urgency rises, the stage tracking cost is reduced down to this scale.
 * Lower values let the optimizer prioritize obstacle avoidance over direct mission tracking.
 *
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_AV_TRK, 0.40f);

/**
 * Minimum terminal waypoint tracking scale during strong avoidance.
 *
 * As obstacle urgency rises anywhere in the horizon, the terminal waypoint cost is reduced
 * down to this scale to avoid fighting the avoidance maneuver.
 *
 * @min 0.02
 * @max 1.0
 * @decimal 2
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_AV_TERM, 0.25f);

/**
 * Minimum control penalty scale during strong avoidance.
 *
 * As obstacle urgency rises, control smoothness and absolute control penalties are reduced
 * down to this scale, allowing stronger steering action near obstacles.
 *
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_AV_CTL, 0.25f);

/**
 * Minimum local altitude for aggressive MPC pitch authority.
 *
 * Below this local Up altitude (relative to the local origin), fw_mpc_avoidance
 * limits nose-down direct pitch to reduce ground-impact risk during avoidance.
 *
 * @unit m
 * @min 0.0
 * @max 200.0
 * @decimal 1
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_MIN_ALT, 10.0f);

/**
 * Legacy MPC model mass.
 *
 * Mass used by the internal full-state MPC model. This is intentionally
 * separate from SIH_* so the avoidance model can be tuned independently of
 * simulator_sih.
 *
 * Defaults are aligned with the Gazebo advanced_plane model.
 *
 * @unit kg
 * @min 0.1
 * @max 50.0
 * @decimal 3
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_MASS, 1.0f);

/**
 * Legacy MPC model inertia Ixx.
 *
 * Roll-axis inertia used by the internal full-state MPC model.
 *
 * Defaults are aligned with the Gazebo advanced_plane model.
 *
 * @unit kg m^2
 * @min 0.0001
 * @max 10.0
 * @decimal 6
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_IXX, 0.197563f);

/**
 * Legacy MPC model inertia Iyy.
 *
 * Pitch-axis inertia used by the internal full-state MPC model.
 *
 * Defaults are aligned with the Gazebo advanced_plane model.
 *
 * @unit kg m^2
 * @min 0.0001
 * @max 10.0
 * @decimal 6
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_IYY, 0.1458929f);

/**
 * Legacy MPC model inertia Izz.
 *
 * Yaw-axis inertia used by the internal full-state MPC model.
 *
 * Defaults are aligned with the Gazebo advanced_plane model.
 *
 * @unit kg m^2
 * @min 0.0001
 * @max 10.0
 * @decimal 6
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_IZZ, 0.1477f);

/**
 * Legacy MPC model linear damping gain.
 *
 * Damping gain passed to the internal aerodynamic model for lateral/vertical
 * velocity damping.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 3
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_KDV, 1.0f);

/**
 * Legacy MPC model angular damping gain.
 *
 * Damping gain passed to the internal aerodynamic model for angular-rate
 * damping.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 4
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_KDW, 0.025f);

/**
 * Base robust obstacle inflation margin.
 *
 * Additional margin applied to both hard and soft obstacle constraints to
 * compensate for model mismatch and tracking uncertainty.
 *
 * @unit m
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_RB_BASE, 0.25f);

/**
 * Robust margin gain on airspeed.
 *
 * Additional obstacle inflation proportional to current true airspeed.
 *
 * @unit s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_RB_VSCL, 0.02f);

/**
 * Robust margin gain on model position error.
 *
 * Additional obstacle inflation proportional to the observed model prediction
 * position error.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_RB_PERR, 0.25f);

/**
 * Robust margin gain on prediction age / fallback age.
 *
 * Additional obstacle inflation proportional to the age of the last accepted
 * model prediction.
 *
 * @unit m/s
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_RB_FAGE, 0.75f);

/**
 * Robust margin gain on guidance quality loss.
 *
 * Additional obstacle inflation proportional to loss of downstream guidance
 * authority, expressed as (1 - can_run_factor).
 *
 * @unit m
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_RB_QFAC, 1.50f);
