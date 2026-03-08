/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team.
 *
 ****************************************************************************/

#include "FwMpcAvoidance.hpp"

#include <modules/navigator/navigation.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>

#include <cmath>
#include <vector>

using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;

namespace
{
bool obstacle_is_ahead(const Vector2f &rel_xy, const Vector2f &vel_xy, float clearance)
{
	const float speed_xy = vel_xy.norm();

	if (speed_xy <= 3.f) {
		return true;
	}

	return rel_xy.dot(vel_xy / speed_xy) > -math::max(clearance, 0.f);
}
} // namespace

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
	const int horizon = math::constrain(_param_fw_mpc_horizon.get(), 2, FwMpcController::kMaxHorizon);
	_controller.configure(_param_fw_mpc_avoid_dt.get(), horizon);
	const matrix::Vector3f I_diag{_param_sih_ixx.get(), _param_sih_iyy.get(), _param_sih_izz.get()};
	_controller.set_vehicle_params(_param_sih_mass.get(), I_diag, _param_sih_kdv.get(), _param_sih_kdw.get());
	_controller.weights().obstacle_proximity_weight = math::max(_param_fw_mpc_obs_cw.get(), 0.f);
	_controller.weights().obstacle_proximity_distance = math::max(_param_fw_mpc_obs_cd.get(), 0.f);
	_controller.weights().avoidance_tracking_scale_min = math::constrain(_param_fw_mpc_av_trk.get(), 0.05f, 1.f);
	_controller.weights().avoidance_terminal_scale_min = math::constrain(_param_fw_mpc_av_term.get(), 0.02f, 1.f);
	_controller.weights().avoidance_control_scale_min = math::constrain(_param_fw_mpc_av_ctl.get(), 0.05f, 1.f);

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

		const int horizon = math::constrain(_param_fw_mpc_horizon.get(), 2, FwMpcController::kMaxHorizon);

		if (_controller.configure(_param_fw_mpc_avoid_dt.get(), horizon)) {
			_mpc_ready = false;

		} else {
			PX4_ERR("fw mpc config failed");
		}

		const matrix::Vector3f I_diag{_param_sih_ixx.get(), _param_sih_iyy.get(), _param_sih_izz.get()};
		_controller.set_vehicle_params(_param_sih_mass.get(), I_diag, _param_sih_kdv.get(), _param_sih_kdw.get());
		_controller.weights().obstacle_proximity_weight = math::max(_param_fw_mpc_obs_cw.get(), 0.f);
		_controller.weights().obstacle_proximity_distance = math::max(_param_fw_mpc_obs_cd.get(), 0.f);
		_controller.weights().avoidance_tracking_scale_min = math::constrain(_param_fw_mpc_av_trk.get(), 0.05f, 1.f);
		_controller.weights().avoidance_terminal_scale_min = math::constrain(_param_fw_mpc_av_term.get(), 0.02f, 1.f);
		_controller.weights().avoidance_control_scale_min = math::constrain(_param_fw_mpc_av_ctl.get(), 0.05f, 1.f);
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

float FwMpcAvoidance::thrust_to_direct_throttle(float thrust_cmd_N) const
{
	const float thrust_min = _controller.limits().u_min(3);
	const float thrust_max = _controller.limits().u_max(3);

	if (!PX4_ISFINITE(thrust_cmd_N) || thrust_max <= thrust_min + 1e-3f) {
		return math::constrain(_param_fw_thr_min.get(), 0.f, 1.f);
	}

	const float throttle_norm = (thrust_cmd_N - thrust_min) / (thrust_max - thrust_min);
	return math::constrain(throttle_norm, math::constrain(_param_fw_thr_min.get(), 0.f, 1.f), 1.f);
}

float FwMpcAvoidance::constrain_pitch_safety(float pitch_cmd, float vehicle_speed, float altitude_up,
		float pitch_min_rad, float pitch_max_rad) const
{
	float safe_pitch_min = pitch_min_rad;
	float safe_pitch_max = pitch_max_rad;

	// Near local-origin ground, prevent aggressive nose-down commands.
	const float min_altitude = math::max(_param_fw_mpc_min_alt.get(), 0.f);

	if (PX4_ISFINITE(altitude_up) && altitude_up < min_altitude) {
		safe_pitch_min = math::max(safe_pitch_min, math::radians(-2.f));
	}

	// At low airspeed, prevent additional strong nose-up commands.
	const float min_airspeed = math::max(_param_fw_airspd_min.get(), 5.f);

	if (PX4_ISFINITE(vehicle_speed) && vehicle_speed < min_airspeed) {
		safe_pitch_max = math::min(safe_pitch_max, math::radians(6.f));
	}

	if (safe_pitch_min > safe_pitch_max) {
		safe_pitch_min = safe_pitch_max;
	}

	return math::constrain(pitch_cmd, safe_pitch_min, safe_pitch_max);
}

void FwMpcAvoidance::publish_obstacle_position()
{
	obstacle_position_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.obstacle_count = 0;

	if (_have_latest_obstacles_msg
	    && (_latest_obstacles_msg.frame == fw_mpc_obstacles_s::FRAME_LOCAL_NED
		|| _latest_obstacles_msg.frame == fw_mpc_obstacles_s::FRAME_LOCAL_ENU)) {
		const uint8_t count = math::min(_latest_obstacles_msg.count, obstacle_position_s::MAX_OBSTACLES);
		msg.obstacle_count = count;

		for (uint8_t i = 0; i < count; i++) {
			if (_latest_obstacles_msg.frame == fw_mpc_obstacles_s::FRAME_LOCAL_ENU) {
				msg.obstacle_north[i] = _latest_obstacles_msg.y[i];
				msg.obstacle_east[i] = _latest_obstacles_msg.x[i];
				msg.obstacle_down[i] = -_latest_obstacles_msg.z[i];

			} else {
				msg.obstacle_north[i] = _latest_obstacles_msg.x[i];
				msg.obstacle_east[i] = _latest_obstacles_msg.y[i];
				msg.obstacle_down[i] = _latest_obstacles_msg.z[i];
			}

			msg.obstacle_size_x[i] = _latest_obstacles_msg.size_x[i];
			msg.obstacle_size_y[i] = _latest_obstacles_msg.size_y[i];
			msg.obstacle_size_z[i] = _latest_obstacles_msg.size_z[i];
			msg.obstacle_radius[i] = _latest_obstacles_msg.radius[i];
			msg.obstacle_height[i] = _latest_obstacles_msg.height[i];
			msg.obstacle_margin[i] = _latest_obstacles_msg.margin[i];
		}
	}

	auto equal_or_both_nan = [](float a, float b) {
		return (PX4_ISFINITE(a) && PX4_ISFINITE(b) && fabsf(a - b) < 1e-4f)
		       || (!PX4_ISFINITE(a) && !PX4_ISFINITE(b));
	};

	bool changed = !_have_last_obstacle_position_publish
		       || msg.obstacle_count != _last_obstacle_position_msg.obstacle_count;

	for (uint8_t i = 0; i < obstacle_position_s::MAX_OBSTACLES && !changed; i++) {
		changed = !equal_or_both_nan(msg.obstacle_north[i], _last_obstacle_position_msg.obstacle_north[i])
			  || !equal_or_both_nan(msg.obstacle_east[i], _last_obstacle_position_msg.obstacle_east[i])
			  || !equal_or_both_nan(msg.obstacle_down[i], _last_obstacle_position_msg.obstacle_down[i])
			  || !equal_or_both_nan(msg.obstacle_size_x[i], _last_obstacle_position_msg.obstacle_size_x[i])
			  || !equal_or_both_nan(msg.obstacle_size_y[i], _last_obstacle_position_msg.obstacle_size_y[i])
			  || !equal_or_both_nan(msg.obstacle_size_z[i], _last_obstacle_position_msg.obstacle_size_z[i])
			  || !equal_or_both_nan(msg.obstacle_radius[i], _last_obstacle_position_msg.obstacle_radius[i])
			  || !equal_or_both_nan(msg.obstacle_height[i], _last_obstacle_position_msg.obstacle_height[i])
			  || !equal_or_both_nan(msg.obstacle_margin[i], _last_obstacle_position_msg.obstacle_margin[i]);
	}

	if (!changed) {
		return;
	}

	_obstacle_position_pub.publish(msg);
	_last_obstacle_position_msg = msg;
	_have_last_obstacle_position_publish = true;
}

void FwMpcAvoidance::publish_mission_setpoint_position(const vehicle_local_position_s *lpos,
		const mission_s *mission,
		const home_position_s *home_pos)
{
	mission_setpoint_position_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.mission_id = mission != nullptr ? mission->mission_id : 0;
	msg.current_seq = mission != nullptr ? mission->current_seq : -1;
	msg.mission_item_count = mission != nullptr ? mission->count : 0;
	msg.total_setpoint_count = 0;
	msg.setpoint_count = 0;
	msg.chunk_index = 0;
	msg.chunk_count = 1;
	msg.reference_valid = false;
	msg.home_alt_valid = home_pos != nullptr && home_pos->valid_alt && PX4_ISFINITE(home_pos->alt);
	msg.truncated = false;

	const bool have_global_ref = lpos != nullptr
				     && lpos->xy_global
				     && lpos->z_global
				     && PX4_ISFINITE(lpos->ref_lat)
				     && PX4_ISFINITE(lpos->ref_lon)
				     && PX4_ISFINITE(lpos->ref_alt);

	struct MissionSetpointEntry {
		uint16_t sequence;
		float north;
		float east;
		float down;
	};

	std::vector<MissionSetpointEntry> setpoints;
	setpoints.reserve(mission != nullptr ? mission->count : 0);

	if (!have_global_ref || mission == nullptr) {
		msg.reference_valid = false;

	} else {
		msg.reference_valid = true;
		MapProjection projection{lpos->ref_lat, lpos->ref_lon, lpos->ref_timestamp};
		const dm_item_t mission_dataman_id = static_cast<dm_item_t>(mission->mission_dataman_id);

		auto mission_item_contains_position = [](const mission_item_s &item) {
			return item.nav_cmd == NAV_CMD_WAYPOINT
			       || item.nav_cmd == NAV_CMD_LOITER_UNLIMITED
			       || item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT
			       || item.nav_cmd == NAV_CMD_LAND
			       || item.nav_cmd == NAV_CMD_TAKEOFF
			       || item.nav_cmd == NAV_CMD_LOITER_TO_ALT
			       || item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
			       || item.nav_cmd == NAV_CMD_VTOL_LAND;
		};

		for (uint16_t i = 0; i < mission->count; i++) {
			mission_item_s item{};

			if (!_dataman_client.readSync(mission_dataman_id, i, reinterpret_cast<uint8_t *>(&item), sizeof(item))) {
				continue;
			}

			if (!mission_item_contains_position(item)) {
				continue;
			}

			if (!PX4_ISFINITE(item.lat) || !PX4_ISFINITE(item.lon)) {
				continue;
			}

			float altitude_amsl = item.altitude;

			if (item.altitude_is_relative) {
				if (!msg.home_alt_valid) {
					continue;
				}

				altitude_amsl += home_pos->alt;
			}

			if (!PX4_ISFINITE(altitude_amsl)) {
				continue;
			}

				float north = NAN;
				float east = NAN;
				projection.project(item.lat, item.lon, north, east);
				const float down = -(altitude_amsl - lpos->ref_alt);

				setpoints.push_back(MissionSetpointEntry{i, north, east, down});
			}
		}

	msg.total_setpoint_count = static_cast<uint16_t>(setpoints.size());
	const uint8_t chunk_count = math::max<uint8_t>(1,
				 static_cast<uint8_t>((setpoints.size() + mission_setpoint_position_s::MAX_SETPOINTS - 1)
				 / mission_setpoint_position_s::MAX_SETPOINTS));
	msg.chunk_count = math::min(chunk_count, mission_setpoint_position_s::MAX_CHUNKS);
	msg.truncated = setpoints.size() > (mission_setpoint_position_s::MAX_SETPOINTS * mission_setpoint_position_s::MAX_CHUNKS);

	for (uint8_t chunk = 0; chunk < msg.chunk_count; chunk++) {
		mission_setpoint_position_s chunk_msg = msg;
		chunk_msg.chunk_index = chunk;
		chunk_msg.setpoint_count = 0;

		for (uint16_t i = 0; i < mission_setpoint_position_s::MAX_SETPOINTS; i++) {
			chunk_msg.sequence[i] = UINT16_MAX;
			chunk_msg.north[i] = NAN;
			chunk_msg.east[i] = NAN;
			chunk_msg.down[i] = NAN;
		}

		const size_t start = chunk * mission_setpoint_position_s::MAX_SETPOINTS;
		const size_t remaining = setpoints.size() > start ? setpoints.size() - start : 0;
		const uint16_t copy_count = math::min<uint16_t>(remaining, mission_setpoint_position_s::MAX_SETPOINTS);
		chunk_msg.setpoint_count = copy_count;

		for (uint16_t i = 0; i < copy_count; i++) {
			const MissionSetpointEntry &entry = setpoints[start + i];
			chunk_msg.sequence[i] = entry.sequence;
			chunk_msg.north[i] = entry.north;
			chunk_msg.east[i] = entry.east;
			chunk_msg.down[i] = entry.down;
		}

		if (_mission_setpoint_position_pub_handles[chunk] == nullptr) {
			int instance = chunk;
			_mission_setpoint_position_pub_handles[chunk] =
				orb_advertise_multi(ORB_ID(mission_setpoint_position), &chunk_msg, &instance);

		} else {
			orb_publish(ORB_ID(mission_setpoint_position), _mission_setpoint_position_pub_handles[chunk], &chunk_msg);
		}
	}

	for (uint8_t chunk = msg.chunk_count; chunk < _mission_setpoint_position_pub_count; chunk++) {
		if (_mission_setpoint_position_pub_handles[chunk] != nullptr) {
			orb_unadvertise(_mission_setpoint_position_pub_handles[chunk]);
			_mission_setpoint_position_pub_handles[chunk] = nullptr;
		}
	}

	_mission_setpoint_position_pub_count = msg.chunk_count;
	_have_last_mission_setpoint_position_publish = true;
	_last_mission_setpoint_position_msg = msg;
}

void FwMpcAvoidance::maybe_log_active_console_status(hrt_abstime now, bool mpc_active, int nearest_obstacle_index,
		float nearest_distance, float trigger_distance, float vehicle_speed, bool solve_success, int qp_tier_used,
		int qp_status, float qp_primal_residual, float qp_dual_residual, float qp_active_slack_max,
		float model_pred_pos_error, float model_pred_vel_error, float model_pred_att_error, float model_pred_age_s)
{
	const bool state_changed = !_have_last_active_console_state
			       || mpc_active != _last_console_active
			       || solve_success != _last_console_solve_success
			       || qp_tier_used != _last_console_qp_tier
			       || qp_status != _last_console_qp_status;

	if (!mpc_active) {
		_last_console_active = false;
		_last_console_solve_success = solve_success;
		_last_console_qp_tier = qp_tier_used;
		_last_console_qp_status = qp_status;
		_have_last_active_console_state = true;
		return;
	}

	static constexpr hrt_abstime log_interval_us = 200000;
	const bool due = (now - _time_last_active_console_log) >= log_interval_us;

	if (!state_changed && !due) {
		return;
	}

	PX4_INFO("active obs=%d near=%.1f v=%.1f tier=%d solve=%d qp=%d pred_p=%.2f dual=%.1f slack=%.2f",
		 nearest_obstacle_index,
		 (double)nearest_distance,
		 (double)vehicle_speed,
		 qp_tier_used,
		 solve_success ? 1 : 0,
		 qp_status,
		 (double)model_pred_pos_error,
		 (double)qp_dual_residual,
		 (double)qp_active_slack_max);

	_time_last_active_console_log = now;
	_last_console_active = mpc_active;
	_last_console_solve_success = solve_success;
	_last_console_qp_tier = qp_tier_used;
	_last_console_qp_status = qp_status;
	_have_last_active_console_state = true;
}

bool FwMpcAvoidance::should_activate_mpc(const vehicle_local_position_s &lpos, const matrix::Vector3f &vel_ned,
		float &nearest_distance, float &trigger_distance, int &nearest_obstacle_index) const
{
	nearest_distance = NAN;
	trigger_distance = NAN;
	nearest_obstacle_index = -1;

	const hrt_abstime obstacle_timeout_us =
		static_cast<hrt_abstime>(math::max(_param_fw_mpc_obs_timeout.get(), 0.05f) * 1e6f);

	if (_obstacle_count <= 0 || hrt_elapsed_time(&_time_obstacle_last_update) > obstacle_timeout_us) {
		return false;
	}

	const Vector2f pos_xy{lpos.x, lpos.y};
	const float pos_z_up = -lpos.z;
	const float speed = vel_ned.norm();
	const Vector2f vel_xy{vel_ned(0), vel_ned(1)};
	const float dmin = math::max(_param_fw_mpc_obs_dmin.get(), 1.f);
	const float lookahead_s = math::max(_param_fw_mpc_obs_lkhd.get(), 0.f);
	const float bias = math::max(_param_fw_mpc_obs_bias.get(), 0.f);
	const float tmax = math::max(_param_fw_mpc_obs_tmax.get(), dmin);
	trigger_distance = math::min(math::max(dmin, speed * lookahead_s + bias), tmax);
	nearest_distance = INFINITY;
	bool obstacle_triggered = false;

	for (int i = 0; i < _obstacle_count; i++) {
		const FwMpcController::Obstacle &obs = _obstacles[i];
		const float Rbuf = obs.R + obs.margin + obs.planning_margin;
		const Vector2f obs_xy{obs.c(0), obs.c(1)};
		const Vector2f rel_xy = obs_xy - pos_xy;
		const float horizontal_distance_to_surface = (obs_xy - pos_xy).norm() - Rbuf;

		if (!obstacle_is_ahead(rel_xy, vel_xy, Rbuf)) {
			continue;
		}

		float obstacle_distance = horizontal_distance_to_surface;

		if (PX4_ISFINITE(obs.height) && obs.height > 0.f) {
			const float half_height_buffered = 0.5f * obs.height + obs.margin;
			const float vertical_distance_to_surface = fabsf(pos_z_up - obs.c(2)) - half_height_buffered;
			obstacle_distance = math::max(horizontal_distance_to_surface, vertical_distance_to_surface);

			// Finite-height obstacle only applies in its vertical span.
			if (vertical_distance_to_surface > 0.f) {
				continue;
			}
		}

		if (!PX4_ISFINITE(nearest_distance) || obstacle_distance < nearest_distance) {
			nearest_distance = obstacle_distance;
			nearest_obstacle_index = i;
		}

		obstacle_triggered |= horizontal_distance_to_surface < trigger_distance;
	}

	if (nearest_obstacle_index < 0) {
		nearest_distance = NAN;
	}

	return obstacle_triggered;
}

void FwMpcAvoidance::publish_mpc_status(bool mpc_allowed, bool mpc_active, bool obstacle_data_fresh,
					bool obstacle_triggered, bool emergency_turn_active, int nearest_obstacle_index, float nearest_distance,
					float trigger_distance, float vehicle_speed, int qp_status, float model_pred_pos_error,
					float model_pred_vel_error, float model_pred_att_error, float model_pred_age_s,
					bool solve_success, int qp_tier_used, int qp_status_polish, float objective_value, float qp_primal_residual,
					float qp_dual_residual, float qp_active_slack_max, float qp_active_slack_sum,
					int qp_iterations, float qp_solve_time_us)
{
	mpc_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.mpc_allowed = mpc_allowed;
	status.mpc_active = mpc_active;
	status.obstacle_data_fresh = obstacle_data_fresh;
	status.obstacle_triggered = obstacle_triggered;
	status.emergency_turn_active = emergency_turn_active;
	status.obstacle_count = math::max(_obstacle_count, 0);
	status.nearest_obstacle_index = nearest_obstacle_index;
	status.nearest_obstacle_distance = nearest_distance;
	status.trigger_distance = trigger_distance;
	status.vehicle_speed = vehicle_speed;
	status.model_pred_pos_error = model_pred_pos_error;
	status.model_pred_vel_error = model_pred_vel_error;
	status.model_pred_att_error = model_pred_att_error;
	status.model_pred_age_s = model_pred_age_s;
	status.solve_success = solve_success;
	status.qp_tier_used = qp_tier_used;
	status.qp_status_polish = qp_status_polish;
	status.objective_value = objective_value;
	status.qp_primal_residual = qp_primal_residual;
	status.qp_dual_residual = qp_dual_residual;
	status.qp_active_slack_max = qp_active_slack_max;
	status.qp_active_slack_sum = qp_active_slack_sum;
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
	const bool auto_mission = (status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);

	return armed && fixed_wing && auto_position_control && auto_mission;
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
	bool obstacle_publish_requested = false;

	if (_fw_mpc_obstacles_sub.update(&obstacles_msg)) {
		_latest_obstacles_msg = obstacles_msg;
		_have_latest_obstacles_msg = true;
		obstacle_publish_requested = true;

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
	fixed_wing_longitudinal_setpoint_s nominal_lon_sp{};
	vehicle_local_position_setpoint_s lpos_sp{};
	vehicle_local_position_s lpos_for_debug{};
	mission_s mission_msg{};
	home_position_s home_pos{};

	bool have_lat = false;
	bool have_lon = false;
	const bool have_goal = _lpos_sp_sub.copy(&lpos_sp);
	const bool mission_updated = _mission_sub.updated();
	const bool have_mission = _mission_sub.copy(&mission_msg);
	const bool have_lpos_for_debug = _lpos_sub.copy(&lpos_for_debug);
	const bool have_home_pos = _home_pos_sub.copy(&home_pos);
	bool mission_publish_requested = mission_updated || !_have_last_mission_setpoint_position_publish;

	if (have_lpos_for_debug) {
		const bool mission_ref_valid = lpos_for_debug.xy_global && lpos_for_debug.z_global;

		if (!_have_last_mission_ref_state
		    || mission_ref_valid != _last_mission_ref_valid
		    || lpos_for_debug.ref_timestamp != _last_mission_ref_timestamp) {
			mission_publish_requested = true;
		}
	}

	if (have_home_pos
	    && (!_have_last_mission_ref_state || home_pos.update_count != _last_home_update_count)) {
		mission_publish_requested = true;
	}

	if (obstacle_publish_requested) {
		publish_obstacle_position();
	}

	if (mission_publish_requested) {
		publish_mission_setpoint_position(have_lpos_for_debug ? &lpos_for_debug : nullptr,
						 have_mission ? &mission_msg : nullptr,
						 have_home_pos ? &home_pos : nullptr);
		_have_last_mission_ref_state = true;
		_last_mission_ref_valid = have_lpos_for_debug && lpos_for_debug.xy_global && lpos_for_debug.z_global;
		_last_mission_ref_timestamp = have_lpos_for_debug ? lpos_for_debug.ref_timestamp : 0;
		_last_home_update_count = have_home_pos ? home_pos.update_count : 0;
	}

	const bool have_nominal_lon = _fw_nominal_lon_sp_sub.copy(&nominal_lon_sp);
	float nearest_obstacle_distance = NAN;
	float trigger_distance = NAN;
	int nearest_obstacle_index = -1;
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
		const bool trigger_now = have_state && have_goal
				 && should_activate_mpc(lpos, vel_N, nearest_obstacle_distance, trigger_distance, nearest_obstacle_index);
		obstacle_triggered = trigger_now;

		if (trigger_now) {
			_time_last_obstacle_trigger = now;
		}

		if (trigger_now) {
			mpc_active_now = true;

		} else if (_mpc_active_last && obstacle_data_fresh && have_state && have_goal) {
			const float exit_hysteresis = math::max(_param_fw_mpc_act_hys.get(), 0.f);
			const hrt_abstime deact_hold_us = static_cast<hrt_abstime>(math::max(_param_fw_mpc_deact_t.get(), 0.f) * 1e6f);
			const bool keep_by_distance = PX4_ISFINITE(nearest_obstacle_distance) && PX4_ISFINITE(trigger_distance)
						      && (nearest_obstacle_distance < (trigger_distance + exit_hysteresis));
			const bool keep_by_time = hrt_elapsed_time(&_time_last_obstacle_trigger) <= deact_hold_us;
			mpc_active_now = keep_by_distance || keep_by_time;
		}

		if (mpc_active_now && !_mpc_active_last) {
			// Reinitialize trim whenever MPC takes over again.
			_mpc_ready = false;
			_have_last_valid_mpc_setpoint = false;
			_have_last_model_prediction = false;
		}

		if (mpc_active_now && have_state && have_goal) {
			const matrix::Quatf q(att.q);
			const matrix::Dcmf R_nb{q};
			const matrix::Vector3f vel_B = R_nb.transpose() * vel_N;
			const matrix::Eulerf euler(q);

			if (lpos.z_global && PX4_ISFINITE(lpos.ref_alt)) {
				_controller.set_altitude_origin_amsl(lpos.ref_alt);
			}

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
			const float obstacle_attention_distance = PX4_ISFINITE(trigger_distance) ? trigger_distance : 0.f;
			const bool have_mpc_command = _controller.step(x_now, goal_up, V_cruise, false, obstacle_attention_distance, u_cmd, x_pred);
			const FwMpcController::QpDebug &step_qp_debug = _controller.last_qp_debug();

			if (have_mpc_command) {
				const float roll_lim_rad = math::radians(math::max(_param_fw_r_lim.get(), 5.f));
				const float pitch_min_rad = math::radians(_param_fw_p_lim_min.get());
				const float pitch_max_rad = math::radians(_param_fw_p_lim_max.get());
				const float phi_cmd = math::constrain(x_pred(6), -roll_lim_rad, roll_lim_rad);
				const float theta_cmd_raw = math::constrain(x_pred(7), pitch_min_rad, pitch_max_rad);
				const float theta_cmd = constrain_pitch_safety(theta_cmd_raw, vehicle_speed, x_now(11),
						      pitch_min_rad, pitch_max_rad);
				float lateral_accel_cmd = CONSTANTS_ONE_G * tanf(phi_cmd);
				lateral_accel_cmd = PX4_ISFINITE(lateral_accel_cmd) ? lateral_accel_cmd : 0.f;

				lat_sp.timestamp = now;
				lat_sp.course = NAN;
				lat_sp.airspeed_direction = NAN;
				lat_sp.lateral_acceleration = lateral_accel_cmd;

				if (have_nominal_lon) {
					lon_sp.altitude = nominal_lon_sp.altitude;
					lon_sp.height_rate = nominal_lon_sp.height_rate;
					lon_sp.equivalent_airspeed = nominal_lon_sp.equivalent_airspeed;

				} else {
					lon_sp.altitude = NAN;
					lon_sp.height_rate = NAN;
					lon_sp.equivalent_airspeed = NAN;
				}

				lon_sp.timestamp = now;
				lon_sp.pitch_direct = theta_cmd;
				lon_sp.throttle_direct = _param_fw_mpc_thr_en.get() ? thrust_to_direct_throttle(u_cmd(3)) : NAN;
				have_lat = true;
				have_lon = true;

				if (step_qp_debug.solve_success) {
					_last_valid_lat_sp = lat_sp;
					_last_valid_lon_sp = lon_sp;
					_time_last_valid_mpc_setpoint = now;
					_have_last_valid_mpc_setpoint = true;
					_last_model_prediction = x_pred;
					_time_last_model_prediction = now;
					_have_last_model_prediction = true;
				}

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
	maybe_log_active_console_status(now, mpc_active_now, nearest_obstacle_index, nearest_obstacle_distance,
				       trigger_distance, vehicle_speed, qp_debug.solve_success, qp_debug.solve_tier_used,
				       _controller.last_qp_status(), qp_debug.primal_residual, qp_debug.dual_residual,
				       qp_debug.active_slack_max, model_pred_pos_error, model_pred_vel_error,
				       model_pred_att_error, model_pred_age_s);
	publish_mpc_status(mpc_allowed, mpc_active_now, obstacle_data_fresh, obstacle_triggered, emergency_turn_active,
			   nearest_obstacle_index, nearest_obstacle_distance,
			   trigger_distance, vehicle_speed, _controller.last_qp_status(), model_pred_pos_error,
			   model_pred_vel_error, model_pred_att_error, model_pred_age_s, qp_debug.solve_success,
			   qp_debug.solve_tier_used,
			   qp_debug.status_polish, qp_debug.objective_value, qp_debug.primal_residual,
			   qp_debug.dual_residual, qp_debug.active_slack_max, qp_debug.active_slack_sum,
			   qp_debug.iterations, qp_debug.solve_time_us);

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
