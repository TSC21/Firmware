/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include "ModeOverlayPolicy.hpp"
#include <lib/motion_planning/VelocitySmoothing.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/goto_setpoint.h>

/** Multicopter position-controller adapter. All work runs on its existing work queue. */
class ModeOverlay : public ModuleParams
{
public:

	explicit ModeOverlay(ModuleParams *parent) : ModuleParams(parent) {}

	trajectory_setpoint_s update(const trajectory_setpoint_s &raw, const vehicle_local_position_s &local,
				     const vehicle_control_mode_s &control, bool landed,
				     const matrix::Vector3f &acceleration, float dt, bool goto_active);
	bool selected() const { return _selected; }

private:

	ModeOverlayPolicy _policy;
	uORB::Subscription _request_sub{ORB_ID(mode_overlay_request)};
	uORB::Subscription _output_sub{ORB_ID(mode_overlay_output)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _goto_sub{ORB_ID(goto_setpoint)};
	uORB::Publication<mode_overlay_reply_s> _reply_pub{ORB_ID(mode_overlay_reply)};
	uORB::Publication<mode_overlay_input_s> _input_pub{ORB_ID(mode_overlay_input)};
	uORB::Publication<mode_overlay_status_s> _status_pub{ORB_ID(mode_overlay_status)};
	vehicle_status_s _vehicle_status{};
	position_setpoint_triplet_s _triplet{};
	goto_setpoint_s _goto{};
	VelocitySmoothing _brake[3];
	uint64_t _last_publish{0};
	uint32_t _reset_count{0};
	uint8_t _reset_counters[5] {};
	uint32_t _brake_epoch{0};
	bool _brake_active{false};
	bool _selected{false};
	float _brake_yaw{0.f};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::COM_OVL_EN>) _param_enabled,
		(ParamFloat<px4::params::COM_OVL_TOUT>) _param_timeout,
		(ParamFloat<px4::params::COM_OVL_DEV>) _param_deviation,
		(ParamInt<px4::params::COM_OVL_MASK>) _param_modes,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_velocity_xy,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_velocity_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_velocity_down,
		(ParamFloat<px4::params::MPC_ACC_HOR>) _param_acceleration_xy,
		(ParamFloat<px4::params::MPC_ACC_UP_MAX>) _param_acceleration_up,
		(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_acceleration_down,
		(ParamFloat<px4::params::MPC_JERK_AUTO>) _param_jerk
	)
};
