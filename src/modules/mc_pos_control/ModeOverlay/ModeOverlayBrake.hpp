/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/motion_planning/VelocitySmoothing.hpp>
#include <px4_platform_common/defines.h>
#include <uORB/topics/trajectory_setpoint.h>

/**
 * Jerk-limited stop along the direction of travel.
 *
 * The reference is the straight line through the estimated position along the
 * estimated velocity. One VelocitySmoothing profile drives the along-track speed
 * to zero with the full deceleration the directional limits allow along that
 * line, so all axes stay time-synchronized and the vector limits hold exactly.
 * The profile starts from the along-track part of the estimated acceleration,
 * clamped to zero or below: the stop never begins by accelerating forward, and
 * the part across the line is dropped so the reference cannot swerve.
 */
class ModeOverlayBrake
{
public:
	struct Limits {
		float acceleration_xy{0.f}; ///< horizontal acceleration norm limit [m/s^2]
		float acceleration_up{0.f}; ///< upward (NED -z) acceleration limit [m/s^2]
		float acceleration_down{0.f}; ///< downward (NED +z) acceleration limit [m/s^2]
		float jerk{0.f}; ///< jerk norm limit [m/s^3]
	};

	/** Start a stop from the estimated NED position [m], velocity [m/s] and acceleration [m/s^2]. */
	void start(const matrix::Vector3f &position, const matrix::Vector3f &velocity,
		   const matrix::Vector3f &acceleration)
	{
		_origin = position;
		const float speed = velocity.isAllFinite() ? velocity.norm() : 0.f;
		_direction = speed > FLT_EPSILON ? matrix::Vector3f(velocity / speed) : matrix::Vector3f{};
		const float along_track = acceleration.isAllFinite() ? acceleration.dot(_direction) : 0.f;
		_along_track.reset(math::min(along_track, 0.f), speed, 0.f);
		_along_track.setMaxVel(speed);
	}

	/**
	 * Largest deceleration along a unit direction that respects the directional limits [m/s^2].
	 * Braking a descent accelerates upwards and braking a climb accelerates downwards.
	 */
	static float deceleration(const matrix::Vector3f &direction, const Limits &limits)
	{
		float deceleration = FLT_MAX;
		const float horizontal = matrix::Vector2f(direction(0), direction(1)).norm();

		if (horizontal > FLT_EPSILON) {
			deceleration = limits.acceleration_xy / horizontal;
		}

		if (direction(2) > FLT_EPSILON) {
			deceleration = math::min(deceleration, limits.acceleration_up / direction(2));

		} else if (direction(2) < -FLT_EPSILON) {
			deceleration = math::min(deceleration, limits.acceleration_down / -direction(2));
		}

		return deceleration < FLT_MAX ? deceleration : 0.f;
	}

	/** Advance the stop by dt [s] and write the NED position, velocity, acceleration and jerk references. */
	void update(float dt, const Limits &limits, trajectory_setpoint_s &setpoint)
	{
		_along_track.setMaxAccel(deceleration(_direction, limits));
		_along_track.setMaxJerk(limits.jerk);
		_along_track.updateDurations(0.f);
		_along_track.updateTraj(dt);

		(_origin + _direction * _along_track.getCurrentPosition()).copyTo(setpoint.position);
		(_direction * _along_track.getCurrentVelocity()).copyTo(setpoint.velocity);
		(_direction * _along_track.getCurrentAcceleration()).copyTo(setpoint.acceleration);
		(_direction * _along_track.getCurrentJerk()).copyTo(setpoint.jerk);
	}

	const matrix::Vector3f &direction() const { return _direction; }

private:
	VelocitySmoothing _along_track; ///< along-track acceleration, speed and distance
	matrix::Vector3f _origin{}; ///< NED position at the start of the stop [m]
	matrix::Vector3f _direction{}; ///< unit NED direction of travel, zero when stationary
};
