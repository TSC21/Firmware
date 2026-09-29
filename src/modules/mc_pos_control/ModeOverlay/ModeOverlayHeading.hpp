/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>

/**
 * Heading reference for an overlay that holds heading authority.
 *
 * The reference follows the companion heading and heading rate exactly while
 * they stay within the rate and acceleration limits, and slews at the limits
 * otherwise. It engages from the previous effective heading, so taking
 * authority never steps the heading. When the companion stops supplying a
 * heading, the reference returns to the heading the overlay commands without
 * authority at the same limits and then passes that heading through unchanged.
 * A brake that starts while the reference is engaged holds the brake heading
 * without handing back, because the brake heading is not the mode heading.
 */
class ModeOverlayHeading
{
public:
	struct Limits {
		float rate{0.f}; ///< heading rate limit [rad/s]
		float acceleration{0.f}; ///< heading acceleration limit [rad/s^2]
	};

	/**
	 * Select this cycle's heading reference.
	 * @param base_yaw heading the overlay commands without authority [rad]; NaN leaves heading uncontrolled
	 * @param base_yawspeed its heading rate [rad/s], NaN for none
	 * @param companion_yaw granted companion heading [rad], NaN when the output supplies none
	 * @param companion_yawspeed companion heading rate [rad/s], NaN for none
	 * @param braking the overlay brake selects base_yaw
	 * @param estimate heading estimate [rad] used when no finite reference precedes engagement
	 * @param dt controller period [s]
	 */
	void update(float base_yaw, float base_yawspeed, float companion_yaw, float companion_yawspeed, bool braking,
		    float estimate, float dt, const Limits &limits)
	{
		const bool companion = PX4_ISFINITE(companion_yaw);

		if (!_active && !companion) {
			pass(base_yaw, base_yawspeed);
			return;
		}

		if (!_active) {
			// Engage from the previous effective reference.
			_active = true;
			_yaw = PX4_ISFINITE(_yaw) ? _yaw : (PX4_ISFINITE(estimate) ? estimate : 0.f);
			_yawspeed = PX4_ISFINITE(_yawspeed) ? _yawspeed : 0.f;
		}

		const bool dt_valid = PX4_ISFINITE(dt) && dt > 0.f;

		if (companion) {
			step(companion_yaw, companion_yawspeed, dt_valid ? dt : 0.f, limits);

		} else if (PX4_ISFINITE(base_yaw)) {
			// Hand back to the mode heading; a brake keeps the reference engaged.
			if (step(base_yaw, base_yawspeed, dt_valid ? dt : 0.f, limits) && !braking) {
				pass(base_yaw, base_yawspeed);
			}

		} else if (step(_yaw, 0.f, dt_valid ? dt : 0.f, limits)) {
			// No heading to return to: stop turning, then leave heading uncontrolled.
			pass(base_yaw, base_yawspeed);
		}
	}

	/** Shift the reference by an estimator heading reset [rad]. */
	void shift(float delta_heading)
	{
		if (PX4_ISFINITE(_yaw) && PX4_ISFINITE(delta_heading)) {
			_yaw = matrix::wrap_pi(_yaw + delta_heading);
		}
	}

	/** Forget the reference, e.g. on disarm. */
	void reset()
	{
		_active = false;
		_yaw = NAN;
		_yawspeed = NAN;
	}

	bool active() const { return _active; }
	float yaw() const { return _yaw; } ///< [rad]
	float yawspeed() const { return _yawspeed; } ///< [rad/s]

private:
	void pass(float yaw, float yawspeed)
	{
		_active = false;
		_yaw = yaw;
		_yawspeed = yawspeed;
	}

	/**
	 * Move one period towards a target heading and rate.
	 * @return true when the reference equals the target
	 */
	bool step(float target_yaw, float target_yawspeed, float dt, const Limits &limits)
	{
		const float rate_limit = math::max(limits.rate, 0.f);
		const float rate_step = math::max(limits.acceleration, 0.f) * dt; // largest rate change per period
		const float target_rate = math::constrain(PX4_ISFINITE(target_yawspeed) ? target_yawspeed : 0.f,
					  -rate_limit, rate_limit);
		const float error = matrix::wrap_pi(target_yaw - _yaw);

		// Track exactly when this period's trapezoidal heading change reaches the
		// target within the acceleration limit; a consistent target leaves no residual.
		if (fabsf(target_rate - _yawspeed) <= rate_step + kRateTolerance
		    && fabsf(error - 0.5f * (_yawspeed + target_rate) * dt) <= rate_step * dt + kHeadingTolerance) {
			_yaw = matrix::wrap_pi(target_yaw);
			_yawspeed = target_rate;
			return true;
		}

		// Otherwise close the error time-optimally. With the closing rate r relative to
		// the target after this period's trapezoidal step, braking at the acceleration
		// limit covers r^2 / (2 a) more; r below is the largest that still stops on the target.
		const float acceleration = math::max(limits.acceleration, 0.f);
		const float half_step = 0.5f * rate_step;
		const float remaining = error - 0.5f * (_yawspeed + target_rate) * dt;
		const float closing = matrix::sign(remaining) * (sqrtf(half_step * half_step + 2.f * acceleration * fabsf(remaining))
				      - half_step);
		const float desired = math::constrain(target_rate + closing, -rate_limit, rate_limit);
		const float rate = _yawspeed + math::constrain(desired - _yawspeed, -rate_step, rate_step);
		_yaw = matrix::wrap_pi(_yaw + 0.5f * (_yawspeed + rate) * dt);
		_yawspeed = rate;
		return false;
	}

	static constexpr float kHeadingTolerance = 1e-6f; ///< float resolution of a wrapped heading [rad]
	static constexpr float kRateTolerance = 1e-6f; ///< [rad/s]

	float _yaw{NAN}; ///< reference heading [rad], NaN while uncontrolled
	float _yawspeed{NAN}; ///< reference heading rate [rad/s]
	bool _active{false}; ///< the reference differs from the heading without authority
};
