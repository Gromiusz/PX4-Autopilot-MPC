/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team.
 *
 ****************************************************************************/

#include "FwMpcAvoidance.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>

#include <cmath>
#include <vector>

using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;

const matrix::Vector3f FwMpcDynamics::_I_diag{0.02f, 0.02f, 0.04f};
const matrix::SquareMatrix<float, 3> FwMpcDynamics::_I = matrix::diag(FwMpcDynamics::_I_diag);
const matrix::SquareMatrix<float, 3> FwMpcDynamics::_I_inv = matrix::diag(matrix::Vector3f{1.f / 0.02f, 1.f / 0.02f, 1.f / 0.04f});

FwMpcAvoidance::FwMpcAvoidance() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

bool FwMpcAvoidance::init()
{
	_controller.configure(_param_fw_mpc_avoid_dt.get(), FwMpcController::kMaxHorizon);
	const matrix::Vector3f I_diag{_param_sih_ixx.get(), _param_sih_iyy.get(), _param_sih_izz.get()};
	_controller.set_vehicle_params(_param_sih_mass.get(), I_diag, _param_sih_kdv.get(), _param_sih_kdw.get());
	_controller.weights().obstacle_proximity_weight = math::max(_param_fw_mpc_obs_cw.get(), 0.f);
	_controller.weights().obstacle_proximity_distance = math::max(_param_fw_mpc_obs_cd.get(), 0.f);

	if (!_lpos_sub.registerCallback()) {
		PX4_ERR("vehicle_local_position callback registration failed");
		return false;
	}

	_last_run = hrt_absolute_time();
	ScheduleNow();
	return true;
}

void FwMpcAvoidance::parameters_update()
{
	if (_param_update_sub.updated()) {
		parameter_update_s p{};
		_param_update_sub.copy(&p);
		updateParams();

		if (_controller.configure(_param_fw_mpc_avoid_dt.get(), FwMpcController::kMaxHorizon)) {
			_mpc_ready = false;

		} else {
			PX4_ERR("fw mpc config failed");
		}

		const matrix::Vector3f I_diag{_param_sih_ixx.get(), _param_sih_iyy.get(), _param_sih_izz.get()};
		_controller.set_vehicle_params(_param_sih_mass.get(), I_diag, _param_sih_kdv.get(), _param_sih_kdw.get());
		_controller.weights().obstacle_proximity_weight = math::max(_param_fw_mpc_obs_cw.get(), 0.f);
		_controller.weights().obstacle_proximity_distance = math::max(_param_fw_mpc_obs_cd.get(), 0.f);
	}
}

void FwMpcAvoidance::step_internal_model(const float dt)
{
	vehicle_attitude_s att{};
	vehicle_angular_velocity_s rates{};
	vehicle_local_position_s lpos{};
	wind_s wind{};

	if (_att_sub.copy(&att) && _rates_sub.copy(&rates) && _lpos_sub.copy(&lpos)) {
		FwMpcDynamics::State s{};
		s.q_nb = Quatf(att.q);
		s.omega_B = Vector3f{rates.xyz[0], rates.xyz[1], rates.xyz[2]};
		s.velocity_N = Vector3f{lpos.vx, lpos.vy, lpos.vz};
		s.position_N = Vector3f{lpos.x, lpos.y, lpos.z};
		_dynamics.reset(s);
	}

	_wind_sub.copy(&wind);
	Vector3f wind_B{wind.windspeed_north, wind.windspeed_east, 0.0f};
	// In absence of MPC solution, use zero moments and zero thrust; this keeps state integration bounded.
	_dynamics.propagate(Vector3f{}, Vector3f{}, wind_B, dt);
}

bool FwMpcAvoidance::should_activate_mpc(const vehicle_local_position_s &lpos, const matrix::Vector3f &vel_ned,
		float &nearest_distance, float &trigger_distance) const
{
	nearest_distance = NAN;
	trigger_distance = NAN;

	const hrt_abstime obstacle_timeout_us =
		static_cast<hrt_abstime>(math::max(_param_fw_mpc_obs_timeout.get(), 0.05f) * 1e6f);

	if (_obstacle_count <= 0 || hrt_elapsed_time(&_time_obstacle_last_update) > obstacle_timeout_us) {
		return false;
	}

	const Vector2f pos_xy{lpos.x, lpos.y};
	const float pos_z_up = -lpos.z;
	const float speed = vel_ned.norm();
	const float dmin = math::max(_param_fw_mpc_obs_dmin.get(), 1.f);
	const float lookahead_s = math::max(_param_fw_mpc_obs_lkhd.get(), 0.f);
	const float bias = math::max(_param_fw_mpc_obs_bias.get(), 0.f);
	trigger_distance = math::max(dmin, speed * lookahead_s + bias);
	nearest_distance = INFINITY;

	for (int i = 0; i < _obstacle_count; i++) {
		const FwMpcController::Obstacle &obs = _obstacles[i];
		const float Rbuf = obs.R + obs.margin + obs.planning_margin;
		const Vector2f obs_xy{obs.c(0), obs.c(1)};
		const float horizontal_distance_to_surface = (obs_xy - pos_xy).norm() - Rbuf;

		if (PX4_ISFINITE(obs.height) && obs.height > 0.f) {
			const float half_height_buffered = 0.5f * obs.height + obs.margin;
			const float vertical_distance_to_surface = fabsf(pos_z_up - obs.c(2)) - half_height_buffered;
			nearest_distance = math::min(nearest_distance, math::max(horizontal_distance_to_surface, vertical_distance_to_surface));

			// Finite-height obstacle only applies in its vertical span.
			if (vertical_distance_to_surface > 0.f) {
				continue;
			}

		} else {
			nearest_distance = math::min(nearest_distance, horizontal_distance_to_surface);
		}

		if (horizontal_distance_to_surface < trigger_distance) {
			return true;
		}
	}

	return false;
}

bool FwMpcAvoidance::build_emergency_avoidance_setpoint(const vehicle_local_position_s &lpos,
		const matrix::Vector3f &vel_ned, float yaw, float pitch_now, float nearest_distance,
		float trigger_distance, hrt_abstime now, fixed_wing_lateral_setpoint_s &lat_sp,
		fixed_wing_longitudinal_setpoint_s &lon_sp) const
{
	if (_obstacle_count <= 0) {
		return false;
	}

	const Vector2f pos_xy{lpos.x, lpos.y};
	const float pos_z_up = -lpos.z;
	float best_horizontal_distance = INFINITY;
	Vector2f best_obs_xy{};
	bool found = false;

	for (int i = 0; i < _obstacle_count; i++) {
		const FwMpcController::Obstacle &obs = _obstacles[i];
		const float Rbuf = obs.R + obs.margin + obs.planning_margin;
		const Vector2f obs_xy{obs.c(0), obs.c(1)};
		const float horizontal_distance_to_surface = (obs_xy - pos_xy).norm() - Rbuf;

		if (PX4_ISFINITE(obs.height) && obs.height > 0.f) {
			const float half_height_buffered = 0.5f * obs.height + obs.margin;
			const float vertical_distance_to_surface = fabsf(pos_z_up - obs.c(2)) - half_height_buffered;

			if (vertical_distance_to_surface > 0.f) {
				continue;
			}
		}

		if (!found || horizontal_distance_to_surface < best_horizontal_distance) {
			best_horizontal_distance = horizontal_distance_to_surface;
			best_obs_xy = obs_xy;
			found = true;
		}
	}

	if (!found) {
		return false;
	}

	Vector2f away_xy = pos_xy - best_obs_xy;

	if (away_xy.norm() < 1e-3f) {
		away_xy = Vector2f{cosf(yaw + M_PI_2_F), sinf(yaw + M_PI_2_F)};
	}

	const float psi_away = atan2f(away_xy(1), away_xy(0));
	const Vector2f vel_xy{vel_ned(0), vel_ned(1)};
	const float psi_now = (vel_xy.norm() > 4.f) ? atan2f(vel_xy(1), vel_xy(0)) : yaw;
	const float dpsi = matrix::wrap_pi(psi_away - psi_now);

	float turn_sign = (dpsi >= 0.f) ? 1.f : -1.f;

	if (fabsf(dpsi) < math::radians(8.f)
	    && _have_last_valid_mpc_setpoint
	    && PX4_ISFINITE(_last_valid_lat_sp.lateral_acceleration)
	    && fabsf(_last_valid_lat_sp.lateral_acceleration) > 0.2f) {
		turn_sign = (_last_valid_lat_sp.lateral_acceleration > 0.f) ? 1.f : -1.f;
	}

	float urgency = 0.75f;

	if (PX4_ISFINITE(nearest_distance) && PX4_ISFINITE(trigger_distance) && trigger_distance > 1e-3f) {
		urgency = math::constrain(1.f - nearest_distance / trigger_distance, 0.45f, 1.f);
	}

	if (PX4_ISFINITE(nearest_distance) && nearest_distance <= 0.f) {
		urgency = 1.f;
	}

	const float roll_lim_rad = math::radians(math::max(_param_fw_r_lim.get(), 5.f));
	const float max_lateral_accel = CONSTANTS_ONE_G * tanf(roll_lim_rad);
	const float lateral_accel_cmd = turn_sign * urgency * max_lateral_accel;

	const float pitch_min_rad = math::radians(_param_fw_p_lim_min.get());
	const float pitch_max_rad = math::radians(_param_fw_p_lim_max.get());
	float pitch_cmd = PX4_ISFINITE(pitch_now) ? pitch_now : math::radians(2.f);
	pitch_cmd = math::constrain(pitch_cmd, pitch_min_rad, pitch_max_rad);

	const float throttle_min = math::constrain(_param_fw_thr_min.get(), 0.f, 1.f);
	float throttle_cmd = math::max(throttle_min, 0.35f);

	if (_have_last_valid_mpc_setpoint && PX4_ISFINITE(_last_valid_lon_sp.throttle_direct)) {
		throttle_cmd = math::max(throttle_cmd, _last_valid_lon_sp.throttle_direct);
	}

	throttle_cmd = math::constrain(throttle_cmd, throttle_min, 1.f);

	lat_sp.timestamp = now;
	lat_sp.course = NAN;
	lat_sp.airspeed_direction = NAN;
	lat_sp.lateral_acceleration = lateral_accel_cmd;

	lon_sp.timestamp = now;
	lon_sp.altitude = NAN;
	lon_sp.height_rate = NAN;
	lon_sp.equivalent_airspeed = NAN;
	lon_sp.pitch_direct = pitch_cmd;
	lon_sp.throttle_direct = throttle_cmd;

	return true;
}

void FwMpcAvoidance::publish_mpc_status(bool mpc_allowed, bool mpc_active, bool obstacle_data_fresh,
					bool obstacle_triggered, bool emergency_turn_active, float nearest_distance,
					float trigger_distance, float vehicle_speed, int qp_status, float model_pred_pos_error,
					float model_pred_vel_error, float model_pred_att_error, float model_pred_age_s,
					bool solve_success, float objective_value, int qp_iterations, float qp_solve_time_us)
{
	mpc_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.mpc_allowed = mpc_allowed;
	status.mpc_active = mpc_active;
	status.obstacle_data_fresh = obstacle_data_fresh;
	status.obstacle_triggered = obstacle_triggered;
	status.emergency_turn_active = emergency_turn_active;
	status.obstacle_count = math::max(_obstacle_count, 0);
	status.nearest_obstacle_distance = nearest_distance;
	status.trigger_distance = trigger_distance;
	status.vehicle_speed = vehicle_speed;
	status.model_pred_pos_error = model_pred_pos_error;
	status.model_pred_vel_error = model_pred_vel_error;
	status.model_pred_att_error = model_pred_att_error;
	status.model_pred_age_s = model_pred_age_s;
	status.solve_success = solve_success;
	status.objective_value = objective_value;
	status.qp_iterations = qp_iterations;
	status.qp_solve_time_us = qp_solve_time_us;
	status.last_qp_status = qp_status;
	_mpc_status_pub.publish(status);
}

bool FwMpcAvoidance::should_allow_mpc(const vehicle_status_s &status, const vehicle_control_mode_s &control_mode) const
{
	const bool armed = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	const bool fixed_wing = (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING) && !status.in_transition_mode;
	const bool auto_position_control = control_mode.flag_control_position_enabled && !control_mode.flag_control_manual_enabled;

	return armed && fixed_wing && auto_position_control;
}

void FwMpcAvoidance::Run()
{
	if (should_exit()) {
		_lpos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	parameters_update();

	fw_mpc_obstacles_s obstacles_msg{};

	if (_fw_mpc_obstacles_sub.update(&obstacles_msg)) {
		if (obstacles_msg.count == 0) {
			_controller.clear_obstacles();
			_obstacle_count = 0;
			_time_obstacle_last_update = hrt_absolute_time();

		} else if (obstacles_msg.frame == fw_mpc_obstacles_s::FRAME_LOCAL_ENU
			   || obstacles_msg.frame == fw_mpc_obstacles_s::FRAME_LOCAL_NED) {
			const int count = math::min((int)obstacles_msg.count, (int)fw_mpc_obstacles_s::MAX_OBSTACLES);
			std::vector<FwMpcController::Obstacle> obs;
			obs.reserve(count);

			for (int i = 0; i < count; i++) {
				float n = 0.f;
				float e = 0.f;
				float u = 0.f;

				if (obstacles_msg.frame == fw_mpc_obstacles_s::FRAME_LOCAL_ENU) {
					n = obstacles_msg.y[i];
					e = obstacles_msg.x[i];
					u = obstacles_msg.z[i];

				} else { // FRAME_LOCAL_NED
					n = obstacles_msg.x[i];
					e = obstacles_msg.y[i];
					u = -obstacles_msg.z[i];
				}

				FwMpcController::Obstacle o{};
				o.c = Vector3f{n, e, u};
				o.R = obstacles_msg.radius[i];
				o.height = obstacles_msg.height[i];
				o.margin = obstacles_msg.margin[i];
				o.planning_margin = math::max(_param_fw_mpc_obs_plan.get(), 0.f);
				obs.push_back(o);
				_obstacles[i] = o;
			}

			_controller.set_obstacles(obs);
			_obstacle_count = count;
			_time_obstacle_last_update = hrt_absolute_time();

		} else {
			_controller.clear_obstacles();
			_obstacle_count = 0;
		}
	}

	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _last_run) * 1e-6f, 0.001f, 0.1f);
	_last_run = now;

	fixed_wing_lateral_setpoint_s lat_sp{};
	fixed_wing_longitudinal_setpoint_s lon_sp{};
	vehicle_local_position_setpoint_s lpos_sp{};

	bool have_lat = false;
	bool have_lon = false;
	const bool have_goal = _lpos_sp_sub.copy(&lpos_sp);
	float nearest_obstacle_distance = NAN;
	float trigger_distance = NAN;
	float vehicle_speed = NAN;
	float model_pred_pos_error = NAN;
	float model_pred_vel_error = NAN;
	float model_pred_att_error = NAN;
	float model_pred_age_s = NAN;
	bool obstacle_triggered = false;
	bool emergency_turn_active = false;
	vehicle_status_s status{};
	vehicle_control_mode_s control_mode{};
	const bool have_status = _status_sub.copy(&status);
	const bool have_control_mode = _control_mode_sub.copy(&control_mode);
	const bool mpc_allowed = have_status && have_control_mode && should_allow_mpc(status, control_mode);
	const hrt_abstime obstacle_timeout_us =
		static_cast<hrt_abstime>(math::max(_param_fw_mpc_obs_timeout.get(), 0.05f) * 1e6f);
	const bool obstacle_data_fresh = (_obstacle_count > 0) && (hrt_elapsed_time(&_time_obstacle_last_update) <= obstacle_timeout_us);

	bool mpc_active_now = false;

	if (_param_fw_mpc_avoid_en.get() && mpc_allowed) {
		vehicle_attitude_s att{};
		vehicle_angular_velocity_s rates{};
		vehicle_local_position_s lpos{};

		const bool have_state = _att_sub.copy(&att) && _rates_sub.copy(&rates) && _lpos_sub.copy(&lpos);
		const matrix::Vector3f vel_N{lpos.vx, lpos.vy, lpos.vz};
		vehicle_speed = vel_N.norm();
		const bool should_activate_now = have_state && have_goal
						&& should_activate_mpc(lpos, vel_N, nearest_obstacle_distance, trigger_distance);
		obstacle_triggered = should_activate_now;
		mpc_active_now = should_activate_now;

		if (should_activate_now && !_mpc_active_last) {
			// Reinitialize trim whenever MPC takes over again.
			_mpc_ready = false;
			_have_last_valid_mpc_setpoint = false;
			_have_last_model_prediction = false;
		}

		if (should_activate_now) {
			const matrix::Quatf q(att.q);
			const matrix::Dcmf R_nb{q};
			const matrix::Vector3f vel_B = R_nb.transpose() * vel_N;
			const matrix::Eulerf euler(q);

			FwMpcController::StateVec x_now{};
			x_now(0) = vel_B(0);
			x_now(1) = vel_B(1);
			x_now(2) = vel_B(2);
			x_now(3) = rates.xyz[0];
			x_now(4) = rates.xyz[1];
			x_now(5) = rates.xyz[2];
			x_now(6) = euler.phi();
			x_now(7) = euler.theta();
			x_now(8) = euler.psi();
			x_now(9) = lpos.x;
			x_now(10) = lpos.y;
			x_now(11) = -lpos.z; // up

			if (_have_last_model_prediction) {
				const float pred_age_s = (now - _time_last_model_prediction) * 1e-6f;
				const float model_dt_s = math::max(_param_fw_mpc_avoid_dt.get(), 1e-3f);

				if (pred_age_s >= 0.5f * model_dt_s && pred_age_s <= 2.f * model_dt_s) {
					model_pred_age_s = pred_age_s;

					const Vector3f pos_now{x_now(9), x_now(10), x_now(11)};
					const Vector3f pos_pred{_last_model_prediction(9), _last_model_prediction(10), _last_model_prediction(11)};
					model_pred_pos_error = (pos_now - pos_pred).norm();

					const Vector3f vel_now{x_now(0), x_now(1), x_now(2)};
					const Vector3f vel_pred{_last_model_prediction(0), _last_model_prediction(1), _last_model_prediction(2)};
					model_pred_vel_error = (vel_now - vel_pred).norm();

					const float dphi = matrix::wrap_pi(x_now(6) - _last_model_prediction(6));
					const float dtheta = matrix::wrap_pi(x_now(7) - _last_model_prediction(7));
					const float dpsi = matrix::wrap_pi(x_now(8) - _last_model_prediction(8));
					model_pred_att_error = sqrtf(dphi * dphi + dtheta * dtheta + dpsi * dpsi);
				}
			}

			const matrix::Vector3f goal_up{lpos_sp.x, lpos_sp.y, -lpos_sp.z};

			if (!_mpc_ready) {
				_controller.initTrim(13.f, x_now(11), goal_up);
				_mpc_ready = true;
			}

			FwMpcController::ControlVec u_cmd{};
			FwMpcController::StateVec x_pred{};
			const float V_cruise = math::max(vel_N.norm(), 8.f);

			if (_controller.step(x_now, goal_up, V_cruise, false, u_cmd, x_pred)) {
				const float roll_lim_rad = math::radians(math::max(_param_fw_r_lim.get(), 5.f));
				const float pitch_min_rad = math::radians(_param_fw_p_lim_min.get());
				const float pitch_max_rad = math::radians(_param_fw_p_lim_max.get());
				const float phi_cmd = math::constrain(x_pred(6), -roll_lim_rad, roll_lim_rad);
				const float theta_cmd = math::constrain(x_pred(7), pitch_min_rad, pitch_max_rad);
				const float throttle_min = math::constrain(_param_fw_thr_min.get(), 0.f, 1.f);
				float throttle_norm = u_cmd(3) / math::max(_controller.limits().u_max(3), 0.1f);
				throttle_norm = PX4_ISFINITE(throttle_norm) ? math::constrain(throttle_norm, throttle_min, 1.f) : throttle_min;
				float lateral_accel_cmd = CONSTANTS_ONE_G * tanf(phi_cmd);
				lateral_accel_cmd = PX4_ISFINITE(lateral_accel_cmd) ? lateral_accel_cmd : 0.f;

				lat_sp.timestamp = now;
				lat_sp.course = NAN;
				lat_sp.airspeed_direction = NAN;
				lat_sp.lateral_acceleration = lateral_accel_cmd;

				lon_sp.timestamp = now;
				lon_sp.altitude = NAN;
				lon_sp.height_rate = NAN;
				lon_sp.equivalent_airspeed = NAN;
				lon_sp.pitch_direct = theta_cmd;
				lon_sp.throttle_direct = throttle_norm;
				have_lat = true;
				have_lon = true;
				_last_valid_lat_sp = lat_sp;
				_last_valid_lon_sp = lon_sp;
				_time_last_valid_mpc_setpoint = now;
				_have_last_valid_mpc_setpoint = true;
				_last_model_prediction = x_pred;
				_time_last_model_prediction = now;
				_have_last_model_prediction = true;

			} else {
				bool have_emergency_setpoint = false;

				if (_param_fw_mpc_emerg_en.get() && obstacle_data_fresh && obstacle_triggered) {
					have_emergency_setpoint = build_emergency_avoidance_setpoint(lpos, vel_N, euler.psi(), euler.theta(),
								      nearest_obstacle_distance, trigger_distance, now, lat_sp, lon_sp);
				}

				if (have_emergency_setpoint) {
					emergency_turn_active = true;
					have_lat = true;
					have_lon = true;
					_last_valid_lat_sp = lat_sp;
					_last_valid_lon_sp = lon_sp;
					_time_last_valid_mpc_setpoint = now;
					_have_last_valid_mpc_setpoint = true;

				} else {
					const hrt_abstime hold_timeout_us =
						static_cast<hrt_abstime>(math::max(_param_fw_mpc_fail_hold.get(), 0.f) * 1e6f);

					if (_have_last_valid_mpc_setpoint
					    && hrt_elapsed_time(&_time_last_valid_mpc_setpoint) <= hold_timeout_us) {
						lat_sp = _last_valid_lat_sp;
						lon_sp = _last_valid_lon_sp;
						lat_sp.timestamp = now;
						lon_sp.timestamp = now;
						have_lat = true;
						have_lon = true;
					}
				}
			}

		} else if (have_state) {
			// Fallback: integrate internal model to keep nominal state bounded.
			step_internal_model(math::max(dt, _param_fw_mpc_avoid_dt.get()));
		}
	}

	if (!mpc_active_now) {
		_have_last_model_prediction = false;
	}

	_mpc_active_last = mpc_active_now;
	const FwMpcController::QpDebug &qp_debug = _controller.last_qp_debug();
	publish_mpc_status(mpc_allowed, mpc_active_now, obstacle_data_fresh, obstacle_triggered, emergency_turn_active,
			   nearest_obstacle_distance,
			   trigger_distance, vehicle_speed, _controller.last_qp_status(), model_pred_pos_error,
			   model_pred_vel_error, model_pred_att_error, model_pred_age_s, qp_debug.solve_success,
			   qp_debug.objective_value, qp_debug.iterations, qp_debug.solve_time_us);

	if (have_lat) {
		_lat_sp_pub.publish(lat_sp);
	}

	if (have_lon) {
		_lon_sp_pub.publish(lon_sp);
	}

	ScheduleDelayed(20000); // 20 ms
}

int FwMpcAvoidance::task_spawn(int argc, char *argv[])
{
	FwMpcAvoidance *instance = new FwMpcAvoidance();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("allocation failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FwMpcAvoidance::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FwMpcAvoidance::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_USAGE_NAME("fw_mpc_avoidance", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	return 0;
}

extern "C" __EXPORT int fw_mpc_avoidance_main(int argc, char *argv[])
{
	return FwMpcAvoidance::main(argc, argv);
}
