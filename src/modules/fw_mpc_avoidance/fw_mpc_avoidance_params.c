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
 * Internal MPC model integration step [s].
 *
 * Sets the integration step used by the internal SIH-like model during rollouts.
 *
 * @unit s
 * @min 0.005
 * @max 0.05
 * @decimal 3
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_AVOID_DT, 0.02f);

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
PARAM_DEFINE_FLOAT(FW_MPC_OBS_LKHD, 3.0f);

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
PARAM_DEFINE_FLOAT(FW_MPC_OBS_BIAS, 5.0f);

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
PARAM_DEFINE_FLOAT(FW_MPC_OBS_PLAN, 8.0f);
