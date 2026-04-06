#pragma once

#include "FwMpcController.hpp"

#include <dataman_client/DatamanClient.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/fixed_wing_lateral_setpoint.h>
#include <uORB/topics/fixed_wing_lateral_status.h>
#include <uORB/topics/fixed_wing_longitudinal_setpoint.h>
#include <uORB/topics/fw_mpc_obstacles.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_setpoint_position.h>
#include <uORB/topics/mpc_status.h>
#include <uORB/topics/obstacle_position.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>

using namespace time_literals;

/**
 * Fixed-wing MPC avoidance prototype module.
 */
class FwMpcAvoidance final : public ModuleBase<FwMpcAvoidance>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	FwMpcAvoidance();
	~FwMpcAvoidance() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	struct MpcStateInputs {
		vehicle_attitude_s att{};
		vehicle_angular_velocity_s rates{};
		vehicle_local_position_s lpos{};
		matrix::Vector3f vel_ned{};
		float vehicle_speed{0.f};
		float can_run_factor{1.f};
		bool have_state{false};
	};

	struct MpcStepContext {
		FwMpcController::StateVec x_now{};
		matrix::Vector3f goal_up{};
		float vehicle_airspeed_cas{0.f};
		float vehicle_airspeed_ref{0.f};
		float cruise_airspeed{0.f};
	};

	struct MpcRunTelemetry {
		float nearest_obstacle_distance{0.f};
		float trigger_distance{0.f};
		int nearest_obstacle_index{-1};
		float vehicle_speed{0.f};
		float model_pred_pos_error{0.f};
		float model_pred_vel_error{0.f};
		float model_pred_att_error{0.f};
		float model_pred_age_s{0.f};
		bool obstacle_triggered{false};
		bool emergency_turn_active{false};
		bool mpc_active_now{false};
	};

	struct PublishedSetpoints {
		fixed_wing_lateral_setpoint_s lat{};
		fixed_wing_longitudinal_setpoint_s lon{};
		bool have_lat{false};
		bool have_lon{false};
	};

	void Run() override;
	void parameters_update();
	bool configure_controller_from_params();
	bool poll_obstacle_updates();
	void refresh_auxiliary_publications();
	void update_run_timing(hrt_abstime &now, float &dt);
	void configure_controller_runtime();
	float limit_roll_setpoint_for_downstream(float desired_roll_sp, float dt_s) const;
	float limit_attitude_error_for_downstream(float desired_att_sp, float current_att,
			float time_constant_s, float max_rate_neg_rad_s, float max_rate_pos_rad_s) const;
	float thrust_to_direct_throttle(float thrust_cmd_N) const;
	float constrain_pitch_safety(float pitch_cmd, float vehicle_speed, float altitude_up,
				     float pitch_min_rad, float pitch_max_rad) const;
	void maybe_log_active_console_status(hrt_abstime now, bool mpc_active, int nearest_obstacle_index,
					    float nearest_distance, float trigger_distance, float vehicle_speed,
					    bool solve_success, int qp_tier_used, int qp_status,
					    float qp_primal_residual, float qp_dual_residual, float qp_active_slack_max,
					    float model_pred_pos_error, float model_pred_vel_error,
					    float model_pred_att_error, float model_pred_age_s);
	void publish_obstacle_position();
	void publish_mission_setpoint_position(const vehicle_local_position_s *lpos,
					      const mission_s *mission,
					      const home_position_s *home_pos);
	bool sample_model_prediction(float pred_age_s, float model_dt_s, FwMpcController::StateVec &x_pred) const;
	bool should_allow_mpc(const vehicle_status_s &status, const vehicle_control_mode_s &control_mode) const;
	bool should_activate_mpc(const vehicle_local_position_s &lpos, const matrix::Vector3f &vel_ned,
				 float &nearest_distance, float &trigger_distance, int &nearest_obstacle_index) const;
	bool compute_mpc_allowed(bool &obstacle_data_fresh);
	MpcStateInputs collect_mpc_state_inputs();
	void update_mpc_activation_state(const MpcStateInputs &inputs, bool have_goal, bool obstacle_data_fresh,
			hrt_abstime now, MpcRunTelemetry &telemetry);
	void update_prediction_error_metrics(const FwMpcController::StateVec &x_now, hrt_abstime now,
			MpcRunTelemetry &telemetry) const;
	void build_mpc_step_context(const MpcStateInputs &inputs, const vehicle_local_position_setpoint_s &lpos_sp,
			const fixed_wing_longitudinal_setpoint_s &nominal_lon_sp, bool have_nominal_lon, hrt_abstime now,
			MpcRunTelemetry &telemetry, MpcStepContext &context);
	void run_active_mpc(const MpcStateInputs &inputs, const MpcStepContext &context, hrt_abstime now, float dt,
			const fixed_wing_longitudinal_setpoint_s &nominal_lon_sp, bool have_nominal_lon,
			const MpcRunTelemetry &telemetry, PublishedSetpoints &setpoints);
	void fill_setpoints_from_mpc_command(const MpcStateInputs &inputs, const MpcStepContext &context,
			const FwMpcController::ControlVec &u_cmd, const FwMpcController::StateVec &x_pred, hrt_abstime now, float dt,
			const fixed_wing_longitudinal_setpoint_s &nominal_lon_sp, bool have_nominal_lon,
			PublishedSetpoints &setpoints) const;
	void update_cached_mpc_solution(const FwMpcController::QpDebug &qp_debug, const PublishedSetpoints &setpoints,
			hrt_abstime now);
	void maybe_use_last_valid_mpc_setpoints(hrt_abstime now, PublishedSetpoints &setpoints) const;
	void reset_inactive_mpc_state();
	bool should_publish_mission_setpoint_on_local_ref_change(const vehicle_local_position_s *lpos) const;
	bool should_publish_mission_setpoint_on_home_update(const home_position_s *home_pos) const;
	void publish_obstacle_position_if_requested(bool publish_requested);
	void publish_mission_setpoint_position_if_requested(bool publish_requested, const vehicle_local_position_s *lpos,
			const mission_s *mission, const home_position_s *home_pos);
	void finalize_run_cycle(hrt_abstime now, bool mpc_allowed, bool obstacle_data_fresh,
			const MpcRunTelemetry &telemetry, const PublishedSetpoints &setpoints);
	void publish_mpc_status(bool mpc_allowed, bool mpc_active, bool obstacle_data_fresh, bool obstacle_triggered,
			       bool emergency_turn_active, int nearest_obstacle_index, float nearest_distance, float trigger_distance, float vehicle_speed,
			       int qp_status, float model_pred_pos_error, float model_pred_vel_error, float model_pred_att_error,
			       float model_pred_age_s, bool solve_success, int qp_tier_used, int qp_status_polish, float objective_value,
			       float qp_primal_residual, float qp_dual_residual, float qp_active_slack_max,
			       float qp_active_slack_sum, int qp_iterations,
			       float qp_solve_time_us, float qp_nonlinear_min_clearance,
			       float qp_accepted_step_scale, bool qp_full_step_rejected);

	uORB::SubscriptionCallbackWorkItem _lpos_sub{this, ORB_ID(vehicle_local_position)};
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _rates_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _home_pos_sub{ORB_ID(home_position)};
	uORB::Subscription _lpos_sp_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription _mission_sub{ORB_ID(mission)};
	uORB::Subscription _fw_nominal_lon_sp_sub{ORB_ID(fixed_wing_longitudinal_setpoint)};
	uORB::Subscription _fw_lat_status_sub{ORB_ID(fixed_wing_lateral_status)};
	uORB::Subscription _fw_mpc_obstacles_sub{ORB_ID(fw_mpc_obstacles)};
	uORB::Subscription _wind_sub{ORB_ID(wind)};

	uORB::SubscriptionInterval _param_update_sub{ORB_ID(parameter_update), 1000000};

	uORB::PublicationData<fixed_wing_lateral_setpoint_s> _lat_sp_pub{ORB_ID(mpc_lateral_setpoint)};
	uORB::PublicationData<fixed_wing_longitudinal_setpoint_s> _lon_sp_pub{ORB_ID(mpc_longitudinal_setpoint)};
	uORB::Publication<obstacle_position_s> _obstacle_position_pub{ORB_ID(obstacle_position)};
	uORB::Publication<mpc_status_s> _mpc_status_pub{ORB_ID(mpc_status)};

	hrt_abstime _last_run{0};
	FwMpcController _controller{};
	DatamanClient _dataman_client{};
	bool _mpc_ready{false};
	bool _mpc_active_last{false};
	hrt_abstime _time_obstacle_last_update{0};
	hrt_abstime _time_last_obstacle_trigger{0};
	hrt_abstime _time_last_valid_mpc_setpoint{0};
	hrt_abstime _time_last_model_prediction{0};
	hrt_abstime _time_last_active_console_log{0};
	int _obstacle_count{0};
	bool _have_last_valid_mpc_setpoint{false};
	bool _have_last_model_prediction{false};
	bool _have_last_obstacle_position_publish{false};
	bool _have_last_mission_setpoint_position_publish{false};
	bool _have_latest_obstacles_msg{false};
	bool _have_last_mission_ref_state{false};
	bool _last_mission_ref_valid{false};
	bool _have_last_active_console_state{false};
	bool _last_console_active{false};
	bool _last_console_solve_success{false};
	mutable bool _have_last_published_roll_sp{false};
	mutable float _last_published_roll_sp_rad{0.f};
	int _last_console_qp_tier{-1};
	int _last_console_qp_status{0};
	int _last_model_prediction_horizon_steps{0};
	matrix::Matrix<float, FwMpcController::kStateSize, FwMpcController::kMaxHorizon + 1> _last_model_prediction_horizon{};
	FwMpcController::Obstacle _obstacles[fw_mpc_obstacles_s::MAX_OBSTACLES]{};
	fixed_wing_lateral_setpoint_s _last_valid_lat_sp{};
	fixed_wing_longitudinal_setpoint_s _last_valid_lon_sp{};
	fw_mpc_obstacles_s _latest_obstacles_msg{};
	obstacle_position_s _last_obstacle_position_msg{};
	mission_setpoint_position_s _last_mission_setpoint_position_msg{};
	orb_advert_t _mission_setpoint_position_pub_handles[mission_setpoint_position_s::MAX_CHUNKS]{};
	uint8_t _mission_setpoint_position_pub_count{0};
	uint32_t _last_home_update_count{0};
	uint64_t _last_mission_ref_timestamp{0};
	uint64_t _obstacle_timeout{500_ms};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FW_MPC_AVOID_EN>) _param_fw_mpc_avoid_en,
		(ParamBool<px4::params::FW_MPC_THR_EN>) _param_fw_mpc_thr_en,
		(ParamInt<px4::params::FW_MPC_HORIZON>) _param_fw_mpc_horizon,
		(ParamFloat<px4::params::FW_MPC_AVOID_DT>) _param_fw_mpc_avoid_dt,
		(ParamFloat<px4::params::FW_MPC_FAIL_HOLD>) _param_fw_mpc_fail_hold,
		(ParamFloat<px4::params::FW_MPC_ACT_HYS>) _param_fw_mpc_act_hys,
		(ParamFloat<px4::params::FW_MPC_DEACT_T>) _param_fw_mpc_deact_t,
		(ParamFloat<px4::params::FW_MPC_OBS_DMIN>) _param_fw_mpc_obs_dmin,
		(ParamFloat<px4::params::FW_MPC_OBS_LKHD>) _param_fw_mpc_obs_lkhd,
		(ParamFloat<px4::params::FW_MPC_OBS_BIAS>) _param_fw_mpc_obs_bias,
		(ParamFloat<px4::params::FW_MPC_OBS_TMAX>) _param_fw_mpc_obs_tmax,
		(ParamFloat<px4::params::FW_MPC_OBS_PLAN>) _param_fw_mpc_obs_plan,
		(ParamFloat<px4::params::FW_MPC_OBS_CW>) _param_fw_mpc_obs_cw,
		(ParamFloat<px4::params::FW_MPC_OBS_CD>) _param_fw_mpc_obs_cd,
		(ParamFloat<px4::params::FW_MPC_AV_TRK>) _param_fw_mpc_av_trk,
		(ParamFloat<px4::params::FW_MPC_AV_TERM>) _param_fw_mpc_av_term,
		(ParamFloat<px4::params::FW_MPC_AV_CTL>) _param_fw_mpc_av_ctl,
		(ParamFloat<px4::params::FW_MPC_MIN_ALT>) _param_fw_mpc_min_alt,
		(ParamFloat<px4::params::FW_MPC_MASS>) _param_fw_mpc_mass,
		(ParamFloat<px4::params::FW_MPC_IXX>) _param_fw_mpc_ixx,
			(ParamFloat<px4::params::FW_MPC_IYY>) _param_fw_mpc_iyy,
			(ParamFloat<px4::params::FW_MPC_IZZ>) _param_fw_mpc_izz,
			(ParamFloat<px4::params::FW_MPC_KDV>) _param_fw_mpc_kdv,
			(ParamFloat<px4::params::FW_MPC_KDW>) _param_fw_mpc_kdw,
			(ParamFloat<px4::params::FW_R_LIM>) _param_fw_r_lim,
			(ParamFloat<px4::params::FW_P_LIM_MIN>) _param_fw_p_lim_min,
			(ParamFloat<px4::params::FW_P_LIM_MAX>) _param_fw_p_lim_max,
			(ParamFloat<px4::params::FW_PN_R_SLEW_MAX>) _param_fw_pn_r_slew_max,
			(ParamFloat<px4::params::FW_R_TC>) _param_fw_r_tc,
			(ParamFloat<px4::params::FW_R_RMAX>) _param_fw_r_rmax,
			(ParamFloat<px4::params::FW_P_TC>) _param_fw_p_tc,
			(ParamFloat<px4::params::FW_P_RMAX_NEG>) _param_fw_p_rmax_neg,
			(ParamFloat<px4::params::FW_P_RMAX_POS>) _param_fw_p_rmax_pos,
			(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min,
			(ParamFloat<px4::params::FW_THR_MIN>) _param_fw_thr_min,
			(ParamFloat<px4::params::FW_MPC_RB_BASE>) _param_fw_mpc_rb_base,
			(ParamFloat<px4::params::FW_MPC_RB_VSCL>) _param_fw_mpc_rb_vscl,
			(ParamFloat<px4::params::FW_MPC_RB_PERR>) _param_fw_mpc_rb_perr,
			(ParamFloat<px4::params::FW_MPC_RB_FAGE>) _param_fw_mpc_rb_fage,
			(ParamFloat<px4::params::FW_MPC_RB_QFAC>) _param_fw_mpc_rb_qfac
		)
};
