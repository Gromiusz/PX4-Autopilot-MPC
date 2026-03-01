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
 * Enable emergency fallback turn when MPC QP solve fails near obstacle.
 *
 * When enabled, the module can generate a direct avoidance turn setpoint if the QP
 * fails while obstacle-triggered mode is active.
 *
 * @value 0 Disabled
 * @value 1 Enabled
 * @min 0
 * @max 1
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_INT32(FW_MPC_EMERG_EN, 0);

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
 * @max 48
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_INT32(FW_MPC_HORIZON, 48);

/**
 * Internal MPC model integration step [s].
 *
 * Sets the integration step used by the internal SIH-like model during rollouts.
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
PARAM_DEFINE_FLOAT(FW_MPC_ACT_HYS, 8.0f);

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
 * Trigger distance is max(FW_MPC_OBS_DMIN, speed * FW_MPC_OBS_LKHD + FW_MPC_OBS_BIAS).
 *
 * @unit m
 * @min 1.0
 * @max 200.0
 * @decimal 1
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_OBS_DMIN, 25.0f);

/**
 * Speed lookahead time for obstacle activation.
 *
 * Used in trigger distance formula: speed * FW_MPC_OBS_LKHD + FW_MPC_OBS_BIAS.
 *
 * @unit s
 * @min 0.0
 * @max 15.0
 * @decimal 2
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_OBS_LKHD, 4.0f);

/**
 * Additional distance bias for obstacle activation.
 *
 * Used in trigger distance formula: speed * FW_MPC_OBS_LKHD + FW_MPC_OBS_BIAS.
 *
 * @unit m
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_OBS_BIAS, 8.0f);

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
PARAM_DEFINE_FLOAT(FW_MPC_OBS_CW, 3.0f);

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
PARAM_DEFINE_FLOAT(FW_MPC_OBS_CD, 12.0f);

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
PARAM_DEFINE_FLOAT(FW_MPC_AV_TRK, 0.25f);

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
PARAM_DEFINE_FLOAT(FW_MPC_AV_TERM, 0.10f);

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
PARAM_DEFINE_FLOAT(FW_MPC_AV_CTL, 0.35f);
