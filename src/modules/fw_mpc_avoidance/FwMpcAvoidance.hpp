#pragma once

#include "FwMpcDynamics.hpp"
#include "FwMpcController.hpp"

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/fixed_wing_lateral_setpoint.h>
#include <uORB/topics/fixed_wing_longitudinal_setpoint.h>
#include <uORB/topics/fw_mpc_obstacles.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wind.h>

/**
 * Fixed-wing MPC avoidance prototype module.
 * Currently pass-through with an internal SIH-like dynamics model step for future rollouts.
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
	void Run() override;
	void parameters_update();
	void step_internal_model(float dt);
	bool should_allow_mpc(const vehicle_status_s &status, const vehicle_control_mode_s &control_mode) const;
	bool should_activate_mpc(const vehicle_local_position_s &lpos, const matrix::Vector3f &vel_ned) const;

	uORB::SubscriptionCallbackWorkItem _lpos_sub{this, ORB_ID(vehicle_local_position)};
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _rates_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _wind_sub{ORB_ID(wind)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _lpos_sp_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription _fw_mpc_obstacles_sub{ORB_ID(fw_mpc_obstacles)};

	uORB::SubscriptionInterval _param_update_sub{ORB_ID(parameter_update), 1000000};

	uORB::PublicationData<fixed_wing_lateral_setpoint_s> _lat_sp_pub{ORB_ID(mpc_lateral_setpoint)};
	uORB::PublicationData<fixed_wing_longitudinal_setpoint_s> _lon_sp_pub{ORB_ID(mpc_longitudinal_setpoint)};

	hrt_abstime _last_run{0};
	FwMpcDynamics _dynamics{};
	FwMpcController _controller{};
	bool _mpc_ready{false};
	bool _mpc_active_last{false};
	hrt_abstime _time_obstacle_last_update{0};
	int _obstacle_count{0};
	FwMpcController::Obstacle _obstacles[fw_mpc_obstacles_s::MAX_OBSTACLES]{};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FW_MPC_AVOID_EN>) _param_fw_mpc_avoid_en,
		(ParamFloat<px4::params::FW_MPC_AVOID_DT>) _param_fw_mpc_avoid_dt,
		(ParamFloat<px4::params::SIH_MASS>) _param_sih_mass,
		(ParamFloat<px4::params::SIH_IXX>) _param_sih_ixx,
		(ParamFloat<px4::params::SIH_IYY>) _param_sih_iyy,
		(ParamFloat<px4::params::SIH_IZZ>) _param_sih_izz,
		(ParamFloat<px4::params::SIH_KDV>) _param_sih_kdv,
		(ParamFloat<px4::params::SIH_KDW>) _param_sih_kdw
	)
};
