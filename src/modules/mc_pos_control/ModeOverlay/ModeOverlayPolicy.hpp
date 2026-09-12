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
		uint32_t modes{SUPPORTED_MODES};
		uint64_t timeout_us{300000};
		float max_deviation{3.f};
		float max_velocity_xy{5.f};
		float max_velocity_up{2.f};
		float max_velocity_down{1.5f};
		float max_acceleration{2.f};
		float max_jerk{4.f};
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

		if (_session == request.session_id && (modes != _modes || fabsf(deviation - _max_deviation) > FLT_EPSILON)) {
			return reply; // Session settings are immutable; unregister before changing them.
		}

		if (_session != request.session_id) {
			_session = request.session_id;
			_modes = modes;
			_max_deviation = deviation;
			_lease_at = now;
			_sequence = 0;
			_ready = false;
			_failed = false;
			clearHistory();
		}

		reply.result = mode_overlay_reply_s::RESULT_ACCEPTED;
		reply.applicable_modes = _modes;
		reply.max_deviation = _max_deviation;
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
		    (output.action == mode_overlay_output_s::ACTION_REPLACE && !valid(output.setpoint, position))) {
			_ready = false;
			_failed |= _armed && _applicable;
			++_rejected;
			return false;
		}

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
			// The vehicle and operator limits may have changed since reception.
			if (!valid(_setpoint, position)) {
				_ready = false;
				_failed = true;
				_braking = true;
				++_rejected;
				return Selection::Brake;
			}

			const float yaw = effective.yaw;
			const float yawspeed = effective.yawspeed;
			effective = _setpoint;
			effective.timestamp = now;
			effective.yaw = yaw; // The overlay has no yaw authority.
			effective.yawspeed = yawspeed;
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
		status.max_acceleration = _config.max_acceleration;
		status.max_jerk = _config.max_jerk;
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
	bool valid(const trajectory_setpoint_s &setpoint, const matrix::Vector3f &position) const
	{
		const matrix::Vector3f p{setpoint.position};
		const matrix::Vector3f v{setpoint.velocity};
		const matrix::Vector3f a{setpoint.acceleration};
		const matrix::Vector3f j{setpoint.jerk};
		return p.isAllFinite() && v.isAllFinite() && a.isAllFinite() && j.isAllFinite() && position.isAllFinite() &&
		       (p - position).norm() <= math::min(_max_deviation, _config.max_deviation) &&
		       matrix::Vector2f(v(0), v(1)).norm() <= _config.max_velocity_xy &&
		       v(2) >= -_config.max_velocity_up && v(2) <= _config.max_velocity_down &&
		       a.norm() <= _config.max_acceleration && j.norm() <= _config.max_jerk;
	}

	Config _config{};
	Intent _history[HISTORY_SIZE] {};
	trajectory_setpoint_s _setpoint{};
	uint64_t _session{0}, _sequence{0}, _next_intent{0}, _accepted_intent{0};
	uint64_t _accepted_at{0}, _accepted_intent_at{0}, _lease_at{0}, _context_since{0};
	uint32_t _modes{0}, _estimator_reset{0}, _rejected{0};
	float _max_deviation{0.f};
	uint8_t _nav_state{0}, _action{mode_overlay_output_s::ACTION_STOP};
	bool _ready{false}, _failed{false}, _initialized{false}, _armed{false};
	bool _applicable{false}, _engaged{false}, _braking{false};
};
