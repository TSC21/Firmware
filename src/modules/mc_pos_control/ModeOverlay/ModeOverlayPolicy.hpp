/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include <cfloat>
#include <px4_platform_common/defines.h>
#include <uORB/topics/mode_overlay_input.h>
#include <uORB/topics/mode_overlay_output.h>
#include <uORB/topics/mode_overlay_reply.h>
#include <uORB/topics/mode_overlay_request.h>
#include <uORB/topics/mode_overlay_status.h>
#include <uORB/topics/vehicle_status.h>

/** Bounded, allocation-free selection policy. It never publishes or mutates raw intent. */
class ModeOverlayPolicy
{
public:

	static constexpr uint32_t SUPPORTED_MODES =
		(1u << vehicle_status_s::NAVIGATION_STATE_POSCTL) |
		(1u << vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) |
		(1u << vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER) |
		(1u << vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) |
		(1u << vehicle_status_s::NAVIGATION_STATE_POSITION_SLOW) |
		(1u << vehicle_status_s::NAVIGATION_STATE_OFFBOARD) |
		(0xffu << vehicle_status_s::NAVIGATION_STATE_EXTERNAL1);

	struct Config {
		bool enabled{false};
		bool yaw_enabled{false}; ///< COM_OVL_YAW: grant requested heading authority
		uint32_t modes{SUPPORTED_MODES};
		uint64_t timeout_us{300000};
		float max_deviation{3.f}; ///< [m]
		float max_velocity_xy{5.f}; ///< [m/s]
		float max_velocity_up{2.f}; ///< [m/s]
		float max_velocity_down{1.5f}; ///< [m/s]
		float max_acceleration_xy{2.f}; ///< horizontal norm [m/s^2]
		float max_acceleration_up{2.f}; ///< NED -z [m/s^2]
		float max_acceleration_down{2.f}; ///< NED +z [m/s^2]
		float max_jerk{4.f}; ///< norm [m/s^3]
		float max_yawspeed{1.f}; ///< [rad/s]
		float max_yaw_acceleration{0.35f}; ///< [rad/s^2]

		/** Acceleration norm accepted in every direction [m/s^2]. */
		float maxAcceleration() const { return math::min(max_acceleration_xy, math::min(max_acceleration_up, max_acceleration_down)); }
	};

	enum class Selection { Passthrough, Replace, Brake };

	void configure(const Config &config)
	{
		if (config.enabled != _config.enabled) {
			_session = 0;
			_modes = 0;
			clearHistory();
		}

		_config = config;
	}
	const Config &config() const { return _config; }
	uint64_t session() const { return _session; }
	bool failed() const { return _failed; }
	/** The session requested heading authority and COM_OVL_YAW currently allows it. */
	bool yawAuthority() const { return _session != 0 && _yaw_granted && _config.yaw_enabled; }
	/** Granted companion heading [rad] and rate [rad/s] of the last Replace selection; NaN without one. */
	float companionYaw() const { return _companion_yaw; }
	float companionYawspeed() const { return _companion_yawspeed; }
	bool ready(uint64_t now) const
	{
		return _session != 0 && _ready && !_failed && fresh(now, _accepted_at)
		       && fresh(now, _accepted_intent_at);
	}

	mode_overlay_reply_s request(const mode_overlay_request_s &request, bool armed, uint64_t now)
	{
		mode_overlay_reply_s reply{};
		reply.timestamp = now;
		reply.request_id = request.request_id;
		reply.session_id = request.session_id;
		reply.result = mode_overlay_reply_s::RESULT_INVALID;
		const uint32_t modes = request.applicable_modes & _config.modes & SUPPORTED_MODES;

		if (request.request_id == 0 || request.session_id == 0) { return reply; }

		if (!_config.enabled) {
			reply.result = mode_overlay_reply_s::RESULT_DISABLED;
			return reply;
		}

		if (armed) {
			reply.result = mode_overlay_reply_s::RESULT_ARMED;
			return reply;
		}

		if (request.action == mode_overlay_request_s::ACTION_UNREGISTER) {
			if (_session == request.session_id) {
				_session = 0;
				_ready = false;
				clearHistory();
				reply.result = mode_overlay_reply_s::RESULT_ACCEPTED;
			}

			return reply;
		}

		if (request.action != mode_overlay_request_s::ACTION_REGISTER || modes == 0 ||
		    !PX4_ISFINITE(request.max_deviation) || request.max_deviation <= 0.f || request.name[0] == '\0') {
			return reply;
		}

		// A dead owner's lease may only be reclaimed while disarmed. Repeated
		// registration transactions do not extend a lease without live responses.
		if (_session != 0 && _session != request.session_id &&
		    now >= _lease_at && now - _lease_at <= LEASE_TIMEOUT_US) {
			reply.result = mode_overlay_reply_s::RESULT_BUSY;
			return reply;
		}

		const float deviation = math::min(request.max_deviation, _config.max_deviation);

		if (_session == request.session_id && (modes != _modes || fabsf(deviation - _max_deviation) > FLT_EPSILON
						       || request.yaw_authority != _yaw_requested)) {
			return reply; // Session settings are immutable; unregister before changing them.
		}

		if (_session != request.session_id) {
			_session = request.session_id;
			_modes = modes;
			_max_deviation = deviation;
			_yaw_requested = request.yaw_authority;
			// Heading authority is decided once per session; COM_OVL_YAW can revoke it later.
			_yaw_granted = request.yaw_authority && _config.yaw_enabled;
			_lease_at = now;
			_sequence = 0;
			_ready = false;
			_failed = false;
			clearHistory();
		}

		reply.result = mode_overlay_reply_s::RESULT_ACCEPTED;
		reply.applicable_modes = _modes;
		reply.max_deviation = _max_deviation;
		reply.yaw_authority = yawAuthority();
		return reply;
	}

	/** Start a controller cycle; invalidate asynchronous work on mode/frame changes. */
	void updateContext(uint64_t now, bool armed, uint8_t nav_state, uint32_t reset_counter,
			   bool control_available)
	{
		const bool applicable = _config.enabled && control_available && nav_state < 32 &&
					((_config.modes & SUPPORTED_MODES & (1u << nav_state)) != 0);

		if (!_initialized || nav_state != _nav_state || reset_counter != _estimator_reset || armed != _armed ||
		    applicable != _applicable) {
			clearHistory();
			_context_since = now;
			_initialized = true;
		}

		_nav_state = nav_state;
		_estimator_reset = reset_counter;
		_armed = armed;
		_applicable = applicable;

		if (!armed) { _failed = false; }
	}

	mode_overlay_input_s input(uint64_t now, const trajectory_setpoint_s &raw)
	{
		mode_overlay_input_s input{};
		input.timestamp = now;
		input.session_id = _session;
		input.intent_id = ++_next_intent;
		input.reset_counter = _estimator_reset;
		input.nav_state = _nav_state;
		input.applicable = _applicable;
		input.armed = _armed;
		input.intent = raw;
		_history[input.intent_id % HISTORY_SIZE] = {input.intent_id, now};
		return input;
	}

	bool output(const mode_overlay_output_s &output, const matrix::Vector3f &position, uint64_t now)
	{
		if (_session == 0 || output.session_id != _session || output.sequence <= _sequence) {
			++_rejected;
			return false;
		}

		const Intent &intent = _history[output.intent_id % HISTORY_SIZE];

		if (intent.id != output.intent_id || !fresh(now, intent.timestamp) ||
		    output.intent_id < _accepted_intent) {
			++_rejected;
			return false;
		}

		if (output.action > mode_overlay_output_s::ACTION_STOP ||
		    (output.action == mode_overlay_output_s::ACTION_REPLACE
		     && !(withinLimits(output.setpoint) && withinRadius(matrix::Vector3f(output.setpoint.position), position)))) {
			_ready = false;
			_failed |= _armed && _applicable;
			++_rejected;
			return false;
		}

		// The reference holds at PX4 reception; a publisher stamp only counts when fresh.
		_setpoint_at = (output.timestamp != 0 && fresh(now, output.timestamp)) ? output.timestamp : now;
		_accepted_at = now;
		_sequence = output.sequence;
		_accepted_intent_at = intent.timestamp;
		_accepted_intent = intent.id;
		_lease_at = now;
		_ready = output.ready;
		_action = output.action;
		_setpoint = output.setpoint;
		return true;
	}

	Selection select(uint64_t now, trajectory_setpoint_s &effective,
			 const matrix::Vector3f &position)
	{
		_engaged = false;
		_braking = false;
		_companion_yaw = NAN;
		_companion_yawspeed = NAN;

		if (!_applicable || !_armed) { return Selection::Passthrough; }

		const bool owned_mode = _session != 0 && (_modes & (1u << _nav_state));

		if (!ready(now) || !owned_mode) {
			// Brake immediately; latch loss after a bounded context-transition grace period.
			if (now >= _context_since && now - _context_since > _config.timeout_us) { _failed = true; }

			_braking = true;
			return Selection::Brake;
		}

		if (_action == mode_overlay_output_s::ACTION_STOP) {
			_braking = true;
			return Selection::Brake;
		}

		if (_action == mode_overlay_output_s::ACTION_REPLACE) {
			// Continue the reference along its own derivatives between companion outputs.
			const trajectory_setpoint_s reference = propagate(_setpoint, now);

			// The vehicle and operator limits may have changed since reception.
			if (!withinLimits(_setpoint) || !withinRadius(matrix::Vector3f(reference.position), position)) {
				_ready = false;
				_failed = true;
				_braking = true;
				++_rejected;
				return Selection::Brake;
			}

			const float yaw = effective.yaw;
			const float yawspeed = effective.yawspeed;
			effective = reference;
			effective.timestamp = now;
			// The source heading stays in effect; granted companion headings are exposed separately
			// so the adapter can engage them without steps.
			effective.yaw = yaw;
			effective.yawspeed = yawspeed;

			if (yawAuthority() && PX4_ISFINITE(reference.yaw)) {
				_companion_yaw = reference.yaw;
				_companion_yawspeed = reference.yawspeed;
			}

			_engaged = true;
			return Selection::Replace;
		}

		return Selection::Passthrough;
	}

	mode_overlay_status_s status(uint64_t now) const
	{
		mode_overlay_status_s status{};
		status.timestamp = now;
		status.session_id = _session;
		status.accepted_sequence = _sequence;
		status.applicable_modes = _modes;
		status.reset_counter = _estimator_reset;
		status.rejected_outputs = _rejected;
		status.nav_state = _nav_state;
		status.enabled = _config.enabled;
		status.registered = _session != 0;
		status.applicable = _applicable;
		status.engaged = _engaged;
		status.ready = ready(now);
		status.braking = _braking;
		status.failed = _failed;
		status.max_deviation = math::min(_max_deviation, _config.max_deviation);
		status.max_velocity_xy = _config.max_velocity_xy;
		status.max_velocity_up = _config.max_velocity_up;
		status.max_velocity_down = _config.max_velocity_down;
		status.max_acceleration = _config.maxAcceleration();
		status.max_acceleration_xy = _config.max_acceleration_xy;
		status.max_acceleration_up = _config.max_acceleration_up;
		status.max_acceleration_down = _config.max_acceleration_down;
		status.max_jerk = _config.max_jerk;
		status.yaw_authority = yawAuthority();
		status.max_yawspeed = _config.max_yawspeed;
		status.max_yaw_acceleration = _config.max_yaw_acceleration;
		status.timeout = _config.timeout_us * 1e-6f;
		return status;
	}

private:

	static constexpr unsigned HISTORY_SIZE = 64; // 1.28 s at the 50 Hz intent rate.
	static constexpr uint64_t LEASE_TIMEOUT_US = 2000000;
	struct Intent { uint64_t id{0}; uint64_t timestamp{0}; };
	bool fresh(uint64_t now, uint64_t stamp) const
	{
		return stamp != 0 && now >= stamp && now - stamp <= _config.timeout_us;
	}
	void clearHistory()
	{
		for (auto &intent : _history) { intent = {}; }

		_ready = false;
		_accepted_at = 0;
		_accepted_intent_at = 0;
	}
	/** Finite full-state reference inside the granted velocity, acceleration and jerk limits. */
	bool withinLimits(const trajectory_setpoint_s &setpoint) const
	{
		const matrix::Vector3f p{setpoint.position};
		const matrix::Vector3f v{setpoint.velocity};
		const matrix::Vector3f a{setpoint.acceleration};
		const matrix::Vector3f j{setpoint.jerk};
		return p.isAllFinite() && v.isAllFinite() && a.isAllFinite() && j.isAllFinite() &&
		       matrix::Vector2f(v(0), v(1)).norm() <= _config.max_velocity_xy &&
		       v(2) >= -_config.max_velocity_up && v(2) <= _config.max_velocity_down &&
		       matrix::Vector2f(a(0), a(1)).norm() <= _config.max_acceleration_xy &&
		       a(2) >= -_config.max_acceleration_up && a(2) <= _config.max_acceleration_down &&
		       j.norm() <= _config.max_jerk;
	}

	/** Reference position inside the granted radius around the vehicle [m]. */
	bool withinRadius(const matrix::Vector3f &reference, const matrix::Vector3f &position) const
	{
		return reference.isAllFinite() && position.isAllFinite() &&
		       (reference - position).norm() <= math::min(_max_deviation, _config.max_deviation);
	}

	/**
	 * Advance a full-state reference by its age since reception, at most the response timeout.
	 * Position, velocity and acceleration follow the reference jerk exactly (a cubic segment);
	 * a finite heading follows its heading rate.
	 */
	trajectory_setpoint_s propagate(const trajectory_setpoint_s &setpoint, uint64_t now) const
	{
		const uint64_t age_us = now > _setpoint_at ? math::min(now - _setpoint_at, _config.timeout_us) : 0;
		const float t = age_us * 1e-6f;
		const matrix::Vector3f p{setpoint.position};
		const matrix::Vector3f v{setpoint.velocity};
		const matrix::Vector3f a{setpoint.acceleration};
		const matrix::Vector3f j{setpoint.jerk};
		trajectory_setpoint_s reference = setpoint;
		(p + v * t + a * (0.5f * t * t) + j * (t * t * t / 6.f)).copyTo(reference.position);
		(v + a * t + j * (0.5f * t * t)).copyTo(reference.velocity);
		(a + j * t).copyTo(reference.acceleration);

		if (PX4_ISFINITE(setpoint.yaw) && PX4_ISFINITE(setpoint.yawspeed)) {
			reference.yaw = matrix::wrap_pi(setpoint.yaw + setpoint.yawspeed * t);
		}

		return reference;
	}

	Config _config{};
	Intent _history[HISTORY_SIZE] {};
	trajectory_setpoint_s _setpoint{};
	uint64_t _session{0}, _sequence{0}, _next_intent{0}, _accepted_intent{0};
	uint64_t _accepted_at{0}, _accepted_intent_at{0}, _lease_at{0}, _context_since{0};
	uint64_t _setpoint_at{0}; ///< PX4 reception time of the accepted reference
	uint32_t _modes{0}, _estimator_reset{0}, _rejected{0};
	float _max_deviation{0.f};
	float _companion_yaw{NAN}, _companion_yawspeed{NAN};
	uint8_t _nav_state{0}, _action{mode_overlay_output_s::ACTION_STOP};
	bool _ready{false}, _failed{false}, _initialized{false}, _armed{false};
	bool _applicable{false}, _engaged{false}, _braking{false};
	bool _yaw_requested{false}, _yaw_granted{false};
};
