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


static bool obstacle_is_ahead(const Vector2f &rel_xy, const Vector2f &vel_xy, float clearance)
{
	const float speed_xy = vel_xy.norm();

	if (speed_xy <= 3.f) {
		return true;
	}

	return rel_xy.dot(vel_xy / speed_xy) > -math::max(clearance, 0.f);
}

FwMpcAvoidance::FwMpcAvoidance() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

void FwMpcAvoidance::configure_controller_runtime()
{
	const matrix::Vector3f I_diag{_param_fw_mpc_ixx.get(), _param_fw_mpc_iyy.get(), _param_fw_mpc_izz.get()};
	_controller.set_vehicle_params(_param_fw_mpc_mass.get(), I_diag, _param_fw_mpc_kdv.get(), _param_fw_mpc_kdw.get());
	_controller.weights().obstacle_proximity_weight = math::max(_param_fw_mpc_obs_cw.get(), 0.f);
	_controller.weights().obstacle_proximity_distance = math::max(_param_fw_mpc_obs_cd.get(), 0.f);
	_controller.weights().avoidance_tracking_scale_min = math::constrain(_param_fw_mpc_av_trk.get(), 0.05f, 1.f);
	_controller.weights().avoidance_terminal_scale_min = math::constrain(_param_fw_mpc_av_term.get(), 0.02f, 1.f);
	_controller.weights().avoidance_control_scale_min = math::constrain(_param_fw_mpc_av_ctl.get(), 0.05f, 1.f);
	// Do not impose extra slew on the internal legacy MPC control space here.
	// The only hard limits we trust from downstream are applied on published
	// attitude/lateral setpoints via FW_R_LIM, FW_P_LIM_MIN/MAX and can_run_factor.
	_controller.limits().use_rate_limits = false;
}

bool FwMpcAvoidance::configure_controller_from_params()
{
	if (!_controller.configure(_param_fw_mpc_avoid_dt.get(), _param_fw_mpc_horizon.get())) {
		PX4_ERR("fw mpc config failed");
		return false;
	}

	return true;
}

float FwMpcAvoidance::limit_roll_setpoint_for_downstream(float desired_roll_sp, float dt_s) const
{
	const float roll_lim_rad = math::radians(math::max(_param_fw_r_lim.get(), 5.f));
	float limited_roll_sp = math::constrain(desired_roll_sp, -roll_lim_rad, roll_lim_rad);

	if (!_have_last_published_roll_sp || dt_s <= 1e-4f) {
		return limited_roll_sp;
	}

	const float max_slew_rad_s = math::radians(math::max(_param_fw_pn_r_slew_max.get(), 0.f));

	if (max_slew_rad_s <= 1e-4f) {
		return limited_roll_sp;
	}

	const float max_delta = max_slew_rad_s * dt_s;
	const float delta = math::constrain(limited_roll_sp - _last_published_roll_sp_rad, -max_delta, max_delta);
	return math::constrain(_last_published_roll_sp_rad + delta, -roll_lim_rad, roll_lim_rad);
}

float FwMpcAvoidance::limit_attitude_error_for_downstream(float desired_att_sp, float current_att,
		float time_constant_s, float max_rate_neg_rad_s, float max_rate_pos_rad_s) const
{
	if (!PX4_ISFINITE(desired_att_sp) || !PX4_ISFINITE(current_att)) {
		return desired_att_sp;
	}

	time_constant_s = math::max(time_constant_s, 0.f);
	max_rate_neg_rad_s = math::max(max_rate_neg_rad_s, 0.f);
	max_rate_pos_rad_s = math::max(max_rate_pos_rad_s, 0.f);

	if (time_constant_s <= 1e-4f || (max_rate_neg_rad_s <= 1e-4f && max_rate_pos_rad_s <= 1e-4f)) {
		return desired_att_sp;
	}

	const float max_neg_error = max_rate_neg_rad_s * time_constant_s;
	const float max_pos_error = max_rate_pos_rad_s * time_constant_s;
	return math::constrain(desired_att_sp, current_att - max_neg_error, current_att + max_pos_error);
}

bool FwMpcAvoidance::init()
{
	if (!configure_controller_from_params()) {
		return false;
	}

	configure_controller_runtime();

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

		if (configure_controller_from_params()) {
			_mpc_ready = false;
		}

		configure_controller_runtime();
	}
}

bool FwMpcAvoidance::poll_obstacle_updates()
{
	fw_mpc_obstacles_s obstacles_msg{};

	if (!_fw_mpc_obstacles_sub.update(&obstacles_msg)) {
		return false;
	}

	_latest_obstacles_msg = obstacles_msg;
	_have_latest_obstacles_msg = true;

	if (obstacles_msg.count == 0) {
		_controller.clear_obstacles();
		_obstacle_count = 0;
		_time_obstacle_last_update = hrt_absolute_time();
		return true;
	}

	if (obstacles_msg.frame != fw_mpc_obstacles_s::FRAME_LOCAL_ENU
	    && obstacles_msg.frame != fw_mpc_obstacles_s::FRAME_LOCAL_NED) {
		_controller.clear_obstacles();
		_obstacle_count = 0;
		return true;
	}

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
	return true;
}

void FwMpcAvoidance::refresh_auxiliary_publications()
{
	// Keep side topics used by debugging and visualization aligned with the latest mission and obstacle state.
	const bool obstacle_publish_requested = poll_obstacle_updates();

	mission_s mission_msg{};
	home_position_s home_pos{};
	vehicle_local_position_s lpos_for_debug{};

	const bool mission_updated = _mission_sub.updated();
	const bool have_mission = _mission_sub.copy(&mission_msg);
	const bool have_lpos_for_debug = _lpos_sub.copy(&lpos_for_debug);
	const bool have_home_pos = _home_pos_sub.copy(&home_pos);
	const vehicle_local_position_s *lpos_for_publish = have_lpos_for_debug ? &lpos_for_debug : nullptr;
	const mission_s *mission_for_publish = have_mission ? &mission_msg : nullptr;
	const home_position_s *home_pos_for_publish = have_home_pos ? &home_pos : nullptr;
	bool mission_publish_requested = mission_updated || !_have_last_mission_setpoint_position_publish;
	mission_publish_requested |= should_publish_mission_setpoint_on_local_ref_change(lpos_for_publish);
	mission_publish_requested |= should_publish_mission_setpoint_on_home_update(home_pos_for_publish);

	publish_obstacle_position_if_requested(obstacle_publish_requested);
	publish_mission_setpoint_position_if_requested(mission_publish_requested, lpos_for_publish,
			mission_for_publish, home_pos_for_publish);
}

void FwMpcAvoidance::update_run_timing(hrt_abstime &now, float &dt)
{
	now = hrt_absolute_time();
	dt = math::constrain((now - _last_run) * 1e-6f, 0.0005f, 0.2f);
	_last_run = now;
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

bool FwMpcAvoidance::sample_model_prediction(float pred_age_s, float model_dt_s,
		FwMpcController::StateVec &x_pred) const
{
	if (!_have_last_model_prediction || _last_model_prediction_horizon_steps < 1 || model_dt_s <= 1e-3f) {
		return false;
	}

	const float max_age_s = (_last_model_prediction_horizon_steps + 0.5f) * model_dt_s;

	if (pred_age_s < 0.5f * model_dt_s || pred_age_s > max_age_s) {
		return false;
	}

	const float step_pos = math::constrain(pred_age_s / model_dt_s, 0.f, static_cast<float>(_last_model_prediction_horizon_steps));
	const int idx0 = math::constrain(static_cast<int>(floorf(step_pos)), 0, _last_model_prediction_horizon_steps);
	const int idx1 = math::min(idx0 + 1, _last_model_prediction_horizon_steps);
	const float alpha = math::constrain(step_pos - static_cast<float>(idx0), 0.f, 1.f);
	const FwMpcController::StateVec x0 = _last_model_prediction_horizon.col(idx0);
	const FwMpcController::StateVec x1 = _last_model_prediction_horizon.col(idx1);

	for (int i = 0; i < FwMpcController::kStateSize; i++) {
		x_pred(i) = x0(i) + alpha * (x1(i) - x0(i));
	}

	for (int i = 6; i <= 8; i++) {
		x_pred(i) = matrix::wrap_pi(x0(i) + alpha * matrix::wrap_pi(x1(i) - x0(i)));
	}

	return true;
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

bool FwMpcAvoidance::should_publish_mission_setpoint_on_local_ref_change(const vehicle_local_position_s *lpos) const
{
	if (lpos == nullptr) {
		return false;
	}

	const bool mission_ref_valid = lpos->xy_global && lpos->z_global;
	return !_have_last_mission_ref_state
	       || mission_ref_valid != _last_mission_ref_valid
	       || lpos->ref_timestamp != _last_mission_ref_timestamp;
}

bool FwMpcAvoidance::should_publish_mission_setpoint_on_home_update(const home_position_s *home_pos) const
{
	if (home_pos == nullptr) {
		return false;
	}

	return !_have_last_mission_ref_state || home_pos->update_count != _last_home_update_count;
}

void FwMpcAvoidance::publish_obstacle_position_if_requested(bool publish_requested)
{
	if (!publish_requested) {
		return;
	}

	publish_obstacle_position();
}

void FwMpcAvoidance::publish_mission_setpoint_position_if_requested(bool publish_requested,
		const vehicle_local_position_s *lpos, const mission_s *mission, const home_position_s *home_pos)
{
	if (!publish_requested) {
		return;
	}

	publish_mission_setpoint_position(lpos, mission, home_pos);
	_have_last_mission_ref_state = true;
	_last_mission_ref_valid = lpos != nullptr && lpos->xy_global && lpos->z_global;
	_last_mission_ref_timestamp = lpos != nullptr ? lpos->ref_timestamp : 0;
	_last_home_update_count = home_pos != nullptr ? home_pos->update_count : 0;
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
					int qp_iterations, float qp_solve_time_us, float qp_nonlinear_min_clearance,
					float qp_accepted_step_scale, bool qp_full_step_rejected)
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
	status.qp_nonlinear_min_clearance = qp_nonlinear_min_clearance;
	status.qp_accepted_step_scale = qp_accepted_step_scale;
	status.qp_full_step_rejected = qp_full_step_rejected;
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

bool FwMpcAvoidance::compute_mpc_allowed(bool &obstacle_data_fresh)
{
	vehicle_status_s status{};
	vehicle_control_mode_s control_mode{};
	const bool have_status = _status_sub.copy(&status);
	const bool have_control_mode = _control_mode_sub.copy(&control_mode);
	const hrt_abstime obstacle_timeout_us =
		static_cast<hrt_abstime>(math::max(_param_fw_mpc_obs_timeout.get(), 0.05f) * 1e6f);
	obstacle_data_fresh = (_obstacle_count > 0) && (hrt_elapsed_time(&_time_obstacle_last_update) <= obstacle_timeout_us);

	return have_status && have_control_mode && should_allow_mpc(status, control_mode);
}

FwMpcAvoidance::MpcStateInputs FwMpcAvoidance::collect_mpc_state_inputs()
{
	MpcStateInputs inputs{};
	inputs.have_state = _att_sub.copy(&inputs.att) && _rates_sub.copy(&inputs.rates) && _lpos_sub.copy(&inputs.lpos);

	if (inputs.have_state) {
		inputs.vel_ned = matrix::Vector3f{inputs.lpos.vx, inputs.lpos.vy, inputs.lpos.vz};
		inputs.vehicle_speed = inputs.vel_ned.norm();

	} else {
		inputs.vehicle_speed = NAN;
	}

	fixed_wing_lateral_status_s fw_lat_status{};
	const hrt_abstime lateral_status_timeout_us = 500000;
	const bool lateral_status_valid = _fw_lat_status_sub.copy(&fw_lat_status)
					 && PX4_ISFINITE(fw_lat_status.can_run_factor)
					 && hrt_elapsed_time(&fw_lat_status.timestamp) <= lateral_status_timeout_us;
	inputs.can_run_factor = lateral_status_valid
				? math::constrain(fw_lat_status.can_run_factor, 0.05f, 1.f)
				: 1.f;
	return inputs;
}

void FwMpcAvoidance::update_mpc_activation_state(const MpcStateInputs &inputs, bool have_goal,
		bool obstacle_data_fresh, hrt_abstime now, MpcRunTelemetry &telemetry)
{
	telemetry.vehicle_speed = inputs.vehicle_speed;
	const bool trigger_now = inputs.have_state && have_goal
				 && should_activate_mpc(inputs.lpos, inputs.vel_ned, telemetry.nearest_obstacle_distance,
							telemetry.trigger_distance, telemetry.nearest_obstacle_index);
	telemetry.obstacle_triggered = trigger_now;

	if (trigger_now) {
		telemetry.mpc_active_now = true;
		_time_last_obstacle_trigger = now;

	} else if (_mpc_active_last && obstacle_data_fresh && inputs.have_state && have_goal) {
		const float exit_hysteresis = math::max(_param_fw_mpc_act_hys.get(), 0.f);
		const hrt_abstime deact_hold_us = static_cast<hrt_abstime>(math::max(_param_fw_mpc_deact_t.get(), 0.f) * 1e6f);
		const bool keep_by_distance = PX4_ISFINITE(telemetry.nearest_obstacle_distance) && PX4_ISFINITE(telemetry.trigger_distance)
					      && (telemetry.nearest_obstacle_distance < (telemetry.trigger_distance + exit_hysteresis));
		const bool keep_by_time = hrt_elapsed_time(&_time_last_obstacle_trigger) <= deact_hold_us;
		telemetry.mpc_active_now = keep_by_distance || keep_by_time;
	}

	if (telemetry.mpc_active_now && !_mpc_active_last) {
		_mpc_ready = false;
		_have_last_valid_mpc_setpoint = false;
		_have_last_model_prediction = false;
		_last_model_prediction_horizon_steps = 0;
	}
}

void FwMpcAvoidance::update_prediction_error_metrics(const FwMpcController::StateVec &x_now, hrt_abstime now,
		MpcRunTelemetry &telemetry) const
{
	if (!_have_last_model_prediction) {
		return;
	}

	const float pred_age_s = (now - _time_last_model_prediction) * 1e-6f;
	const float model_dt_s = math::max(_param_fw_mpc_avoid_dt.get(), 1e-3f);
	FwMpcController::StateVec x_pred_ref{};

	if (!sample_model_prediction(pred_age_s, model_dt_s, x_pred_ref)) {
		return;
	}

	telemetry.model_pred_age_s = pred_age_s;

	const Vector3f pos_now{x_now(9), x_now(10), x_now(11)};
	const Vector3f pos_pred{x_pred_ref(9), x_pred_ref(10), x_pred_ref(11)};
	telemetry.model_pred_pos_error = (pos_now - pos_pred).norm();

	const Vector3f vel_now{x_now(0), x_now(1), x_now(2)};
	const Vector3f vel_pred{x_pred_ref(0), x_pred_ref(1), x_pred_ref(2)};
	telemetry.model_pred_vel_error = (vel_now - vel_pred).norm();

	const float dphi = matrix::wrap_pi(x_now(6) - x_pred_ref(6));
	const float dtheta = matrix::wrap_pi(x_now(7) - x_pred_ref(7));
	const float dpsi = matrix::wrap_pi(x_now(8) - x_pred_ref(8));
	telemetry.model_pred_att_error = sqrtf(dphi * dphi + dtheta * dtheta + dpsi * dpsi);
}

void FwMpcAvoidance::build_mpc_step_context(const MpcStateInputs &inputs,
		const vehicle_local_position_setpoint_s &lpos_sp, const fixed_wing_longitudinal_setpoint_s &nominal_lon_sp,
		bool have_nominal_lon, hrt_abstime now, MpcRunTelemetry &telemetry, MpcStepContext &context)
{
	// Translate estimator and guidance outputs into the state/reference quantities used by the MPC model.
	const matrix::Quatf q(inputs.att.q);
	const matrix::Dcmf R_nb{q};
	const matrix::Eulerf euler(q);
	airspeed_validated_s airspeed_validated{};
	const bool airspeed_valid = _airspeed_validated_sub.copy(&airspeed_validated)
				   && PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s)
				   && PX4_ISFINITE(airspeed_validated.true_airspeed_m_s)
				   && airspeed_validated.airspeed_source != airspeed_validated_s::SOURCE_SYNTHETIC
				   && hrt_elapsed_time(&airspeed_validated.timestamp) <= 1_s;
	wind_s wind{};
	const bool wind_valid = _wind_sub.copy(&wind)
				&& PX4_ISFINITE(wind.windspeed_north)
				&& PX4_ISFINITE(wind.windspeed_east)
				&& hrt_elapsed_time(&wind.timestamp) <= 1_s;
	const matrix::Vector3f wind_N = wind_valid ? matrix::Vector3f{wind.windspeed_north, wind.windspeed_east, 0.f}
				 : matrix::Vector3f{};
	const matrix::Vector3f vel_air_N = inputs.vel_ned - wind_N;
	const matrix::Vector3f vel_B = R_nb.transpose() * vel_air_N;
	const float vehicle_airspeed_tas = vel_air_N.norm();
	context.vehicle_airspeed_cas = airspeed_valid ? math::max(0.5f, airspeed_validated.calibrated_airspeed_m_s)
				      : vehicle_airspeed_tas;
	context.vehicle_airspeed_ref = airspeed_valid ? math::max(0.5f, airspeed_validated.true_airspeed_m_s)
				      : vehicle_airspeed_tas;
	const float eas2tas = (airspeed_valid && context.vehicle_airspeed_cas > 0.5f)
			      ? math::constrain(airspeed_validated.true_airspeed_m_s / context.vehicle_airspeed_cas, 0.9f, 2.0f)
			      : 1.f;

	if (inputs.lpos.z_global && PX4_ISFINITE(inputs.lpos.ref_alt)) {
		_controller.set_altitude_origin_amsl(inputs.lpos.ref_alt);
	}

	_controller.set_wind_ned(wind_N);

	context.x_now(0) = vel_B(0);
	context.x_now(1) = vel_B(1);
	context.x_now(2) = vel_B(2);
	context.x_now(3) = inputs.rates.xyz[0];
	context.x_now(4) = inputs.rates.xyz[1];
	context.x_now(5) = inputs.rates.xyz[2];
	context.x_now(6) = euler.phi();
	context.x_now(7) = euler.theta();
	context.x_now(8) = euler.psi();
	context.x_now(9) = inputs.lpos.x;
	context.x_now(10) = inputs.lpos.y;
	context.x_now(11) = -inputs.lpos.z;

	update_prediction_error_metrics(context.x_now, now, telemetry);

	context.goal_up = matrix::Vector3f{lpos_sp.x, lpos_sp.y, -lpos_sp.z};
	const bool have_nominal_airspeed = have_nominal_lon
					 && PX4_ISFINITE(nominal_lon_sp.equivalent_airspeed)
					 && nominal_lon_sp.equivalent_airspeed > 1.f;
	context.cruise_airspeed = have_nominal_airspeed
				 ? math::max(nominal_lon_sp.equivalent_airspeed * eas2tas, 8.f)
				 : math::max(context.vehicle_airspeed_ref, 8.f);
}

void FwMpcAvoidance::fill_setpoints_from_mpc_command(const MpcStateInputs &inputs, const MpcStepContext &context,
		const FwMpcController::ControlVec &u_cmd, const FwMpcController::StateVec &x_pred, hrt_abstime now, float dt,
		const fixed_wing_longitudinal_setpoint_s &nominal_lon_sp, bool have_nominal_lon,
		PublishedSetpoints &setpoints) const
{
	const float roll_lim_rad = math::radians(math::max(_param_fw_r_lim.get(), 5.f));
	const float pitch_min_rad = math::radians(_param_fw_p_lim_min.get());
	const float pitch_max_rad = math::radians(_param_fw_p_lim_max.get());
	const float roll_rate_rad_s = math::radians(math::max(_param_fw_r_rmax.get(), 0.f));
	const float pitch_rate_neg_rad_s = math::radians(math::max(_param_fw_p_rmax_neg.get(), 0.f));
	const float pitch_rate_pos_rad_s = math::radians(math::max(_param_fw_p_rmax_pos.get(), 0.f));
	const float phi_cmd_slewed = limit_roll_setpoint_for_downstream(x_pred(6), dt);
	const float phi_cmd = limit_attitude_error_for_downstream(phi_cmd_slewed, context.x_now(6),
				     _param_fw_r_tc.get(), roll_rate_rad_s, roll_rate_rad_s);
	const float theta_cmd_raw = math::constrain(x_pred(7), pitch_min_rad, pitch_max_rad);
	const float theta_cmd_dyn = limit_attitude_error_for_downstream(theta_cmd_raw, context.x_now(7),
				      _param_fw_p_tc.get(), pitch_rate_neg_rad_s, pitch_rate_pos_rad_s);
	const float theta_cmd = constrain_pitch_safety(theta_cmd_dyn, context.vehicle_airspeed_cas, context.x_now(11),
				      pitch_min_rad, pitch_max_rad);
	float lateral_accel_cmd = CONSTANTS_ONE_G * tanf(phi_cmd) / inputs.can_run_factor;
	lateral_accel_cmd = PX4_ISFINITE(lateral_accel_cmd) ? lateral_accel_cmd : 0.f;
	const float lateral_accel_pub_lim = CONSTANTS_ONE_G * tanf(roll_lim_rad) / inputs.can_run_factor;
	lateral_accel_cmd = math::constrain(lateral_accel_cmd, -lateral_accel_pub_lim, lateral_accel_pub_lim);
	const matrix::Quatf q_pred(matrix::Eulerf(x_pred(6), x_pred(7), x_pred(8)));
	const matrix::Dcmf R_nb_pred{q_pred};
	const Vector3f v_air_pred_B{x_pred(0), x_pred(1), x_pred(2)};
	const Vector3f v_air_pred_N = R_nb_pred * v_air_pred_B;
	const Vector2f v_air_pred_xy{v_air_pred_N(0), v_air_pred_N(1)};
	const float airspeed_direction_cmd = (v_air_pred_xy.norm() > 2.f)
					     ? atan2f(v_air_pred_xy(1), v_air_pred_xy(0))
					     : NAN;

	setpoints.lat.timestamp = now;
	setpoints.lat.course = NAN;
	setpoints.lat.airspeed_direction = airspeed_direction_cmd;
	setpoints.lat.lateral_acceleration = lateral_accel_cmd;

	if (have_nominal_lon) {
		setpoints.lon.altitude = nominal_lon_sp.altitude;
		setpoints.lon.height_rate = nominal_lon_sp.height_rate;
		setpoints.lon.equivalent_airspeed = nominal_lon_sp.equivalent_airspeed;

	} else {
		setpoints.lon.altitude = NAN;
		setpoints.lon.height_rate = NAN;
		setpoints.lon.equivalent_airspeed = NAN;
	}

	setpoints.lon.timestamp = now;
	setpoints.lon.pitch_direct = theta_cmd;
	setpoints.lon.throttle_direct = _param_fw_mpc_thr_en.get() ? thrust_to_direct_throttle(u_cmd(3)) : NAN;
	setpoints.have_lat = true;
	setpoints.have_lon = true;
}

void FwMpcAvoidance::update_cached_mpc_solution(const FwMpcController::QpDebug &qp_debug,
		const PublishedSetpoints &setpoints, hrt_abstime now)
{
	if (!qp_debug.solve_success || !setpoints.have_lat || !setpoints.have_lon) {
		return;
	}

	_last_valid_lat_sp = setpoints.lat;
	_last_valid_lon_sp = setpoints.lon;
	_time_last_valid_mpc_setpoint = now;
	_have_last_valid_mpc_setpoint = true;
	_last_model_prediction_horizon = _controller.last_solved_state_horizon();
	_last_model_prediction_horizon_steps = _controller.horizon();
	_time_last_model_prediction = now;
	_have_last_model_prediction = true;
}

void FwMpcAvoidance::maybe_use_last_valid_mpc_setpoints(hrt_abstime now, PublishedSetpoints &setpoints) const
{
	const hrt_abstime hold_timeout_us =
		static_cast<hrt_abstime>(math::max(_param_fw_mpc_fail_hold.get(), 0.f) * 1e6f);

	if (!_have_last_valid_mpc_setpoint || hrt_elapsed_time(&_time_last_valid_mpc_setpoint) > hold_timeout_us) {
		return;
	}

	setpoints.lat = _last_valid_lat_sp;
	setpoints.lon = _last_valid_lon_sp;
	setpoints.lat.timestamp = now;
	setpoints.lon.timestamp = now;
	setpoints.have_lat = true;
	setpoints.have_lon = true;
}

void FwMpcAvoidance::run_active_mpc(const MpcStateInputs &inputs, const MpcStepContext &context, hrt_abstime now, float dt,
		const fixed_wing_longitudinal_setpoint_s &nominal_lon_sp, bool have_nominal_lon,
		const MpcRunTelemetry &telemetry, PublishedSetpoints &setpoints)
{
	if (!_mpc_ready) {
		_controller.initTrim(context.cruise_airspeed, context.x_now(11), context.goal_up);
		_mpc_ready = true;
	}

	_controller.set_guidance_quality_factor(inputs.can_run_factor);
	const float model_pos_error_term = PX4_ISFINITE(telemetry.model_pred_pos_error) ? math::max(telemetry.model_pred_pos_error, 0.f) : 0.f;
	const float prediction_age_term = PX4_ISFINITE(telemetry.model_pred_age_s) ? math::max(telemetry.model_pred_age_s, 0.f) : 0.f;
	const float robust_margin = math::constrain(
					 math::max(_param_fw_mpc_rb_base.get(), 0.f)
					 + math::max(_param_fw_mpc_rb_vscl.get(), 0.f) * math::max(context.vehicle_airspeed_ref, 0.f)
					 + math::max(_param_fw_mpc_rb_perr.get(), 0.f) * model_pos_error_term
					 + math::max(_param_fw_mpc_rb_fage.get(), 0.f) * prediction_age_term
					 + math::max(_param_fw_mpc_rb_qfac.get(), 0.f) * math::max(1.f - inputs.can_run_factor, 0.f),
					 0.f, 20.f);
	_controller.set_robustness_margin(robust_margin);

	FwMpcController::ControlVec u_cmd{};
	FwMpcController::StateVec x_pred{};
	const float obstacle_attention_distance = PX4_ISFINITE(telemetry.trigger_distance) ? telemetry.trigger_distance : 0.f;
	const bool have_mpc_command = _controller.step(context.x_now, context.goal_up, context.cruise_airspeed, false,
					      obstacle_attention_distance, dt, u_cmd, x_pred);
	const FwMpcController::QpDebug &step_qp_debug = _controller.last_qp_debug();

	if (have_mpc_command) {
		fill_setpoints_from_mpc_command(inputs, context, u_cmd, x_pred, now, dt, nominal_lon_sp, have_nominal_lon, setpoints);
		update_cached_mpc_solution(step_qp_debug, setpoints, now);
		return;
	}

	maybe_use_last_valid_mpc_setpoints(now, setpoints);
}

void FwMpcAvoidance::reset_inactive_mpc_state()
{
	_have_last_model_prediction = false;
	_last_model_prediction_horizon_steps = 0;
	_have_last_published_roll_sp = false;
}

void FwMpcAvoidance::finalize_run_cycle(hrt_abstime now, bool mpc_allowed, bool obstacle_data_fresh,
		const MpcRunTelemetry &telemetry, const PublishedSetpoints &setpoints)
{
	if (!telemetry.mpc_active_now) {
		reset_inactive_mpc_state();
	}

	_mpc_active_last = telemetry.mpc_active_now;
	const FwMpcController::QpDebug &qp_debug = _controller.last_qp_debug();
	maybe_log_active_console_status(now, telemetry.mpc_active_now, telemetry.nearest_obstacle_index,
				       telemetry.nearest_obstacle_distance, telemetry.trigger_distance,
				       telemetry.vehicle_speed, qp_debug.solve_success, qp_debug.solve_tier_used,
				       _controller.last_qp_status(), qp_debug.primal_residual, qp_debug.dual_residual,
				       qp_debug.active_slack_max, telemetry.model_pred_pos_error, telemetry.model_pred_vel_error,
				       telemetry.model_pred_att_error, telemetry.model_pred_age_s);
	publish_mpc_status(mpc_allowed, telemetry.mpc_active_now, obstacle_data_fresh, telemetry.obstacle_triggered,
			   telemetry.emergency_turn_active, telemetry.nearest_obstacle_index, telemetry.nearest_obstacle_distance,
			   telemetry.trigger_distance, telemetry.vehicle_speed, _controller.last_qp_status(),
			   telemetry.model_pred_pos_error, telemetry.model_pred_vel_error, telemetry.model_pred_att_error,
			   telemetry.model_pred_age_s, qp_debug.solve_success, qp_debug.solve_tier_used,
			   qp_debug.status_polish, qp_debug.objective_value, qp_debug.primal_residual,
			   qp_debug.dual_residual, qp_debug.active_slack_max, qp_debug.active_slack_sum,
			   qp_debug.iterations, qp_debug.solve_time_us, qp_debug.nonlinear_min_clearance,
			   qp_debug.accepted_step_scale, qp_debug.full_step_rejected);

	if (setpoints.have_lat) {
		if (telemetry.mpc_active_now && PX4_ISFINITE(setpoints.lat.lateral_acceleration)) {
			const float roll_lim_rad = math::radians(math::max(_param_fw_r_lim.get(), 5.f));
			_last_published_roll_sp_rad = math::constrain(atanf(setpoints.lat.lateral_acceleration / CONSTANTS_ONE_G),
						      -roll_lim_rad, roll_lim_rad);
			_have_last_published_roll_sp = true;
		}

		_lat_sp_pub.publish(setpoints.lat);
	}

	if (setpoints.have_lon) {
		_lon_sp_pub.publish(setpoints.lon);
	}
}

void FwMpcAvoidance::Run()
{
	if (should_exit()) {
		_lpos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	parameters_update();
	refresh_auxiliary_publications();

	hrt_abstime now{};
	float dt{0.f};
	update_run_timing(now, dt);

	fixed_wing_longitudinal_setpoint_s nominal_lon_sp{};
	vehicle_local_position_setpoint_s lpos_sp{};
	const bool have_goal = _lpos_sp_sub.copy(&lpos_sp);
	const bool have_nominal_lon = _fw_nominal_lon_sp_sub.copy(&nominal_lon_sp);
	bool obstacle_data_fresh = false;
	const bool mpc_allowed = compute_mpc_allowed(obstacle_data_fresh);
	MpcRunTelemetry telemetry{};
	telemetry.nearest_obstacle_distance = NAN;
	telemetry.trigger_distance = NAN;
	telemetry.vehicle_speed = NAN;
	telemetry.model_pred_pos_error = NAN;
	telemetry.model_pred_vel_error = NAN;
	telemetry.model_pred_att_error = NAN;
	telemetry.model_pred_age_s = NAN;
	PublishedSetpoints setpoints{};

	if (_param_fw_mpc_avoid_en.get() && mpc_allowed) {
		const MpcStateInputs inputs = collect_mpc_state_inputs();
		update_mpc_activation_state(inputs, have_goal, obstacle_data_fresh, now, telemetry);

		if (telemetry.mpc_active_now && inputs.have_state && have_goal) {
			MpcStepContext context{};
			build_mpc_step_context(inputs, lpos_sp, nominal_lon_sp, have_nominal_lon, now, telemetry, context);
			run_active_mpc(inputs, context, now, dt, nominal_lon_sp, have_nominal_lon, telemetry, setpoints);
		}
	}

	finalize_run_cycle(now, mpc_allowed, obstacle_data_fresh, telemetry, setpoints);
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
