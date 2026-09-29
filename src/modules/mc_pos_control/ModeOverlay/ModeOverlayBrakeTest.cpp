/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "ModeOverlayBrake.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

using matrix::Vector3f;

namespace
{
// PX4 multicopter defaults: MPC_ACC_HOR, MPC_ACC_UP_MAX, MPC_ACC_DOWN_MAX, MPC_JERK_AUTO.
constexpr ModeOverlayBrake::Limits kLimits{3.f, 4.f, 3.f, 4.f};
constexpr float kDt = 0.01f; // 100 Hz position controller
constexpr float kHighSpeed = 6.f; // m/s
// Float integration of a 10 s profile at 100 Hz stays within a millimetre; one
// controller step at 6 m/s moves the reference 6 cm.
constexpr float kDistanceTolerance = 0.005f;
constexpr float kLimitTolerance = 1e-4f;

/** Collinear jerk-limited stop from rest acceleration (aerial_navigation JerkLimitedBrake). */
float stoppingDistance(float speed, float deceleration, float jerk)
{
	const float ramp = fminf(deceleration / jerk, sqrtf(speed / jerk));
	const float coast = fmaxf(0.f, speed / (jerk * ramp) - ramp);
	return 0.5f * speed * (2.f * ramp + coast);
}

struct Result {
	Vector3f final_position;
	float max_acceleration{0.f};
	float max_jerk{0.f};
	float max_cross_track{0.f};
	float min_along_track_speed{INFINITY};
	float first_acceleration{0.f};
	int steps{0};
};

Result run(const Vector3f &velocity, const Vector3f &acceleration, const ModeOverlayBrake::Limits &limits = kLimits)
{
	ModeOverlayBrake brake;
	const Vector3f origin{10.f, -20.f, -5.f};
	brake.start(origin, velocity, acceleration);
	const Vector3f direction = velocity.unit_or_zero();
	Result result{};
	trajectory_setpoint_s setpoint{};

	for (int i = 0; i < 2000; ++i) {
		brake.update(kDt, limits, setpoint);
		const Vector3f position{setpoint.position};
		const Vector3f v{setpoint.velocity};
		const Vector3f a{setpoint.acceleration};

		if (i == 0) { result.first_acceleration = a.dot(direction); }

		result.max_acceleration = fmaxf(result.max_acceleration, a.norm());
		result.max_jerk = fmaxf(result.max_jerk, Vector3f(setpoint.jerk).norm());
		const Vector3f offset = position - origin;
		result.max_cross_track = fmaxf(result.max_cross_track, (offset - direction * offset.dot(direction)).norm());
		result.min_along_track_speed = fminf(result.min_along_track_speed, v.dot(direction));
		result.final_position = position;
		result.steps = i + 1;

		if (v.norm() < 1e-6f && a.norm() < 1e-6f) { break; }
	}

	return result;
}
} // namespace

TEST(ModeOverlayBrakeTest, StopsWithinTheCollinearModelInEveryDirection)
{
	const Vector3f directions[] = {
		{1.f, 0.f, 0.f}, {-1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {1.f, 1.f, 0.f},
		{1.f, -1.f, 0.f}, {0.f, 0.f, 1.f}, {0.f, 0.f, -1.f}, {1.f, 1.f, 1.f}, {3.f, -1.f, 0.5f}
	};

	for (const Vector3f &raw : directions) {
		const Vector3f direction = raw.normalized();
		const float deceleration = ModeOverlayBrake::deceleration(direction, kLimits);
		const Result result = run(direction * kHighSpeed, {});
		const float distance = (result.final_position - Vector3f{10.f, -20.f, -5.f}).norm();
		SCOPED_TRACE(testing::Message() << "direction " << direction(0) << "," << direction(1) << "," << direction(2));
		EXPECT_NEAR(distance, stoppingDistance(kHighSpeed, deceleration, kLimits.jerk), kDistanceTolerance);
		EXPECT_LT(result.max_cross_track, 1e-4f);
		EXPECT_LE(result.max_acceleration, deceleration + kLimitTolerance);
		EXPECT_LE(result.max_jerk, kLimits.jerk + kLimitTolerance);
		EXPECT_GE(result.min_along_track_speed, -kLimitTolerance);
		EXPECT_LT(result.steps, 2000);
	}
}

TEST(ModeOverlayBrakeTest, HorizontalStopUsesTheFullHorizontalLimit)
{
	// 6 m/s along x with MPC defaults: 6^2/(2*3) + 6*3/(2*4) = 8.25 m.
	const Result result = run({kHighSpeed, 0.f, 0.f}, {});
	EXPECT_NEAR(result.final_position(0) - 10.f, 8.25f, kDistanceTolerance);
	EXPECT_NEAR(result.max_acceleration, kLimits.acceleration_xy, kLimitTolerance);
	EXPECT_FLOAT_EQ(result.final_position(1), -20.f);
	EXPECT_FLOAT_EQ(result.final_position(2), -5.f);
}

TEST(ModeOverlayBrakeTest, DirectionalLimitsSetTheDeceleration)
{
	const ModeOverlayBrake::Limits limits{5.f, 4.f, 2.f, 10.f};
	EXPECT_FLOAT_EQ(ModeOverlayBrake::deceleration({1.f, 0.f, 0.f}, limits), 5.f);
	EXPECT_FLOAT_EQ(ModeOverlayBrake::deceleration({0.f, 0.f, 1.f}, limits), 4.f); // descent: brake upwards
	EXPECT_FLOAT_EQ(ModeOverlayBrake::deceleration({0.f, 0.f, -1.f}, limits), 2.f); // climb: brake downwards
	const Vector3f climb = Vector3f{3.f, 0.f, -4.f}.normalized();
	// Horizontal part 5/0.6 = 8.33, vertical part 2/0.8 = 2.5 m/s^2.
	EXPECT_FLOAT_EQ(ModeOverlayBrake::deceleration(climb, limits), 2.5f);
	EXPECT_FLOAT_EQ(ModeOverlayBrake::deceleration({}, limits), 0.f);
}

TEST(ModeOverlayBrakeTest, NeverStartsByAcceleratingForward)
{
	const Result accelerating = run({kHighSpeed, 0.f, 0.f}, {2.5f, 0.f, 0.f});
	EXPECT_LE(accelerating.first_acceleration, 0.f);
	EXPECT_NEAR(accelerating.final_position(0) - 10.f, 8.25f, kDistanceTolerance);

	// Already decelerating harder than the limit still ends at rest, sooner.
	const Result decelerating = run({kHighSpeed, 0.f, 0.f}, {-5.f, 0.f, 0.f});
	EXPECT_LT(decelerating.final_position(0) - 10.f, 8.25f);
	EXPECT_LT(decelerating.steps, 2000);
}

TEST(ModeOverlayBrakeTest, DropsCrossTrackAccelerationInsteadOfSwerving)
{
	// Braking in a 3 m/s^2 turn: the reference stays on the line of travel.
	const Result result = run({kHighSpeed, 0.f, 0.f}, {0.f, 3.f, -1.f});
	EXPECT_FLOAT_EQ(result.final_position(1), -20.f);
	EXPECT_FLOAT_EQ(result.final_position(2), -5.f);
	EXPECT_NEAR(result.final_position(0) - 10.f, 8.25f, kDistanceTolerance);
}

TEST(ModeOverlayBrakeTest, StationaryOrInvalidEstimatesHoldPosition)
{
	const float nan = std::numeric_limits<float>::quiet_NaN();

	for (const Vector3f &velocity : {Vector3f{}, Vector3f{nan, 0.f, 0.f}}) {
		ModeOverlayBrake brake;
		brake.start({1.f, 2.f, -3.f}, velocity, {nan, 1.f, 1.f});
		trajectory_setpoint_s setpoint{};

		for (int i = 0; i < 100; ++i) { brake.update(kDt, kLimits, setpoint); }

		EXPECT_EQ(Vector3f(setpoint.position), Vector3f(1.f, 2.f, -3.f));
		EXPECT_EQ(Vector3f(setpoint.velocity), Vector3f{});
		EXPECT_EQ(Vector3f(setpoint.acceleration), Vector3f{});
		EXPECT_EQ(Vector3f(setpoint.jerk), Vector3f{});
	}
}

TEST(ModeOverlayBrakeTest, LowSpeedStopWithoutReachingTheDecelerationLimit)
{
	// Below A^2/J = 2.25 m/s the jerk limit alone shapes the stop: d = v * sqrt(v / J).
	const Result result = run({1.f, 0.f, 0.f}, {});
	EXPECT_NEAR(result.final_position(0) - 10.f, stoppingDistance(1.f, kLimits.acceleration_xy, kLimits.jerk),
		    kDistanceTolerance);
	EXPECT_NEAR(result.final_position(0) - 10.f, 0.5f, kDistanceTolerance);
	EXPECT_LT(result.max_acceleration, kLimits.acceleration_xy);
}
