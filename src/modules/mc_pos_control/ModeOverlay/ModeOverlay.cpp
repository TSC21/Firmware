/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "ModeOverlay.hpp"
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>

trajectory_setpoint_s ModeOverlay::update(const trajectory_setpoint_s &raw,
		const vehicle_local_position_s &local, const vehicle_control_mode_s &control,
		bool landed, const matrix::Vector3f &acceleration, float dt, bool goto_active)
{
	const uint64_t now = hrt_absolute_time();
	_vehicle_status_sub.update(&_vehicle_status);
	_triplet_sub.update(&_triplet);
	_goto_sub.update(&_goto);
	ModeOverlayPolicy::Config config{};
	config.enabled = _param_enabled.get();
	config.timeout_us = static_cast<uint64_t>(math::constrain(_param_timeout.get(), 0.1f, 1.f) * 1e6f);
	config.max_deviation = _param_deviation.get();
	config.modes = static_cast<uint32_t>(_param_modes.get());
	config.max_velocity_xy = _param_velocity_xy.get();
	config.max_velocity_up = _param_velocity_up.get();
	config.max_velocity_down = _param_velocity_down.get();
	config.max_acceleration = math::min(_param_acceleration_xy.get(),
					    math::min(_param_acceleration_up.get(), _param_acceleration_down.get()));
	config.max_jerk = _param_jerk.get();
	_policy.configure(config);

	if (!config.enabled) {
		_policy.updateContext(now, control.flag_armed, _vehicle_status.nav_state, _reset_count, false);
		_selected = false;
		_brake_active = false;
		mode_overlay_request_s request{};

		if (_request_sub.update(&request)) { _reply_pub.publish(_policy.request(request, control.flag_armed, now)); }

		if (now - _last_publish >= 1000000) {
			_status_pub.publish(_policy.status(now));
			_last_publish = now;
		}

		return raw;
	}

	const uint8_t counters[] = {local.xy_reset_counter, local.z_reset_counter,
				    local.vxy_reset_counter, local.vz_reset_counter, local.heading_reset_counter
				   };

	for (unsigned i = 0; i < 5; ++i) {
		if (counters[i] != _reset_counters[i]) {
			++_reset_count;
			_reset_counters[i] = counters[i];
		}
	}

	const matrix::Vector3f position{local.x, local.y, local.z};
	const matrix::Vector3f velocity{local.vx, local.vy, local.vz};
	const bool navigator_mode = _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION ||
				    _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER ||
				    _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
	const bool navigator_takeoff_or_land = navigator_mode && _triplet.current.valid &&
					       (_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF ||
						_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND);
	const bool available = control.flag_multicopter_position_control_enabled && !landed && !navigator_takeoff_or_land &&
			       _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
			       !_vehicle_status.in_transition_mode && position.isAllFinite() && velocity.isAllFinite() &&
			       local.xy_valid && local.z_valid && local.v_xy_valid && local.v_z_valid;
	_policy.updateContext(now, control.flag_armed, _vehicle_status.nav_state, _reset_count, available);

	mode_overlay_request_s request{};

	for (unsigned i = 0; i < mode_overlay_request_s::ORB_QUEUE_LENGTH && _request_sub.update(&request); ++i) {
		_reply_pub.publish(_policy.request(request, control.flag_armed, now));
	}

	mode_overlay_output_s output{};

	if (_output_sub.update(&output)) { _policy.output(output, position, now); }

	trajectory_setpoint_s effective = raw;
	const auto selection = _policy.select(now, effective, position);
	_selected = selection != ModeOverlayPolicy::Selection::Passthrough;
	const auto status = _policy.status(now);

	if (selection == ModeOverlayPolicy::Selection::Brake) {
		if (!_brake_active || _brake_epoch != status.reset_counter) {
			_brake_yaw = PX4_ISFINITE(local.heading) ? local.heading : 0.f;
			_brake_epoch = status.reset_counter;

			for (unsigned i = 0; i < 3; ++i) {
				_brake[i].reset(PX4_ISFINITE(acceleration(i)) ? acceleration(i) : 0.f, velocity(i), position(i));
			}
		}

		_brake_active = true;
		effective.timestamp = now;
		effective.yaw = _brake_yaw;
		effective.yawspeed = 0.f;

		// Per-axis limits divided by sqrt(3) bound the total vector magnitude.
		constexpr float inverse_sqrt_three = 0.577350269f;

		for (auto &axis : _brake) {
			axis.setMaxJerk(config.max_jerk * inverse_sqrt_three);
			axis.setMaxAccel(config.max_acceleration * inverse_sqrt_three);
			axis.setMaxVel(math::max(config.max_velocity_xy, math::max(config.max_velocity_up, config.max_velocity_down)));
			axis.updateDurations(0.f);
		}

		VelocitySmoothing::timeSynchronization(_brake, 3);

		for (unsigned i = 0; i < 3; ++i) {
			_brake[i].updateTraj(dt);
			effective.position[i] = _brake[i].getCurrentPosition();
			effective.velocity[i] = _brake[i].getCurrentVelocity();
			effective.acceleration[i] = _brake[i].getCurrentAcceleration();
			effective.jerk[i] = _brake[i].getCurrentJerk();
		}

	} else {
		_brake_active = false;
	}

	// Correlation tokens are generated at 50 Hz, independently of controller frequency.
	if (now - _last_publish >= 20000) {
		auto input = _policy.input(now, raw);

		if (goto_active && matrix::Vector3f(_goto.position).isAllFinite()) {
			matrix::Vector3f(_goto.position).copyTo(input.position_goal);
			input.position_goal_valid = true;

		} else if (navigator_mode && _triplet.current.valid && local.xy_global && local.z_global &&
			   PX4_ISFINITE(_triplet.current.lat) && PX4_ISFINITE(_triplet.current.lon) &&
			   PX4_ISFINITE(_triplet.current.alt) && PX4_ISFINITE(local.ref_lat) &&
			   PX4_ISFINITE(local.ref_lon) && PX4_ISFINITE(local.ref_alt)) {
			MapProjection projection(local.ref_lat, local.ref_lon);
			projection.project(_triplet.current.lat, _triplet.current.lon, input.position_goal[0], input.position_goal[1]);
			input.position_goal[2] = local.ref_alt - _triplet.current.alt;
			input.position_goal_valid = matrix::Vector3f(input.position_goal).isAllFinite();
		}

		_input_pub.publish(input);
		_status_pub.publish(status);
		_last_publish = now;
	}

	return effective;
}
