/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "ModeOverlayHeading.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <limits>

namespace
{
constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
constexpr float kDt = 0.01f; // 100 Hz position controller
// MPC_YAWRAUTO_MAX 60 deg/s and MPC_YAWRAUTO_ACC 20 deg/s^2.
const ModeOverlayHeading::Limits kLimits{math::radians(60.f), math::radians(20.f)};
constexpr float kTolerance = 1e-5f; // float resolution of headings and rates [rad, rad/s]

bool sameBits(float a, float b) { return std::memcmp(&a, &b, sizeof(float)) == 0; }

/** Checks every step against the rate and acceleration limits and the previous heading. */
struct Recorder {
	float yaw{kNan};
	float yawspeed{kNan};
	float max_rate{0.f};
	float max_acceleration{0.f};
	float max_step{0.f};

	void record(const ModeOverlayHeading &heading)
	{
		if (PX4_ISFINITE(yaw) && PX4_ISFINITE(heading.yaw())) {
			max_step = fmaxf(max_step, fabsf(matrix::wrap_pi(heading.yaw() - yaw)));
		}

		if (PX4_ISFINITE(yawspeed) && PX4_ISFINITE(heading.yawspeed())) {
			max_acceleration = fmaxf(max_acceleration, fabsf(heading.yawspeed() - yawspeed) / kDt);
		}

		if (PX4_ISFINITE(heading.yawspeed())) { max_rate = fmaxf(max_rate, fabsf(heading.yawspeed())); }

		yaw = heading.yaw();
		yawspeed = heading.yawspeed();
	}
};
} // namespace

TEST(ModeOverlayHeadingTest, WithoutCompanionHeadingTheBasePassesThroughUnchanged)
{
	ModeOverlayHeading heading;

	for (float base : {0.3f, -3.1f, kNan, 2.9f}) {
		for (bool braking : {false, true}) {
			const float rate = braking ? 0.f : kNan;
			heading.update(base, rate, kNan, kNan, braking, 1.f, kDt, kLimits);
			EXPECT_TRUE(sameBits(heading.yaw(), base));
			EXPECT_TRUE(sameBits(heading.yawspeed(), rate));
			EXPECT_FALSE(heading.active());
		}
	}
}

TEST(ModeOverlayHeadingTest, EngagesFromThePreviousHeadingWithoutAStep)
{
	ModeOverlayHeading heading;
	Recorder recorder;
	heading.update(0.2f, 0.f, kNan, kNan, false, 0.f, kDt, kLimits);
	recorder.record(heading);
	int steps = 0;

	for (; steps < 1000 && !sameBits(heading.yaw(), 1.5f); ++steps) {
		heading.update(0.2f, 0.f, 1.5f, 0.f, false, 0.f, kDt, kLimits);
		recorder.record(heading);
		EXPECT_TRUE(heading.active());
	}

	// Time optimal at 20 deg/s^2 for 1.3 rad: 2*sqrt(1.3/0.349) = 3.86 s; 60 deg/s is not reached.
	EXPECT_NEAR(steps * kDt, 3.86f, 0.1f);
	EXPECT_LE(recorder.max_step, kLimits.rate * kDt + kTolerance);
	EXPECT_LE(recorder.max_rate, kLimits.rate + kTolerance);
	EXPECT_LE(recorder.max_acceleration, kLimits.acceleration + kTolerance / kDt);
	EXPECT_FLOAT_EQ(heading.yawspeed(), 0.f);
}

TEST(ModeOverlayHeadingTest, EngagesFromTheEstimateWhenHeadingWasUncontrolled)
{
	ModeOverlayHeading heading;
	heading.update(kNan, kNan, kNan, kNan, false, -0.4f, kDt, kLimits);
	heading.update(kNan, kNan, 1.f, 0.f, false, -0.4f, kDt, kLimits);
	EXPECT_TRUE(heading.active());
	EXPECT_NEAR(heading.yaw(), -0.4f, kLimits.acceleration * kDt * kDt);
}

TEST(ModeOverlayHeadingTest, TracksAFeasibleTurningHeadingExactly)
{
	ModeOverlayHeading heading;
	heading.update(0.f, 0.f, kNan, kNan, false, 0.f, kDt, kLimits);
	// Heading rate ramps at 0.2 rad/s^2 up to 0.6 rad/s, inside both limits.
	float target = 0.f;
	float rate = 0.f;
	int exact = 0;

	for (int i = 0; i < 600; ++i) {
		const float next_rate = fminf(rate + 0.2f * kDt, 0.6f);
		target = matrix::wrap_pi(target + 0.5f * (rate + next_rate) * kDt);
		rate = next_rate;
		heading.update(0.f, 0.f, target, rate, false, 0.f, kDt, kLimits);

		if (fabsf(matrix::wrap_pi(heading.yaw() - target)) < kTolerance && fabsf(heading.yawspeed() - rate) < kTolerance) {
			++exact;
		}
	}

	// Engaged at the same heading and rate, the reference never lags.
	EXPECT_EQ(exact, 600);
}

TEST(ModeOverlayHeadingTest, InfeasibleHeadingsSlewAtTheLimits)
{
	// A pi/2 heading step with a 3 rad/s^2 acceleration limit saturates the 60 deg/s rate limit.
	const ModeOverlayHeading::Limits limits{kLimits.rate, 3.f};
	ModeOverlayHeading heading;
	Recorder recorder;
	heading.update(0.f, 0.f, kNan, kNan, false, 0.f, kDt, limits);
	recorder.record(heading);
	float furthest = 0.f;

	for (int i = 0; i < 500; ++i) {
		heading.update(0.f, 0.f, M_PI_2_F, 0.f, false, 0.f, kDt, limits);
		recorder.record(heading);
		furthest = fmaxf(furthest, heading.yaw());
	}

	EXPECT_NEAR(recorder.max_rate, limits.rate, kTolerance);
	EXPECT_LE(recorder.max_acceleration, limits.acceleration + kTolerance / kDt);
	EXPECT_LE(recorder.max_step, limits.rate * kDt + kTolerance);
	EXPECT_FLOAT_EQ(heading.yaw(), M_PI_2_F);
	EXPECT_LE(furthest, M_PI_2_F + kTolerance);

	// A companion heading turning at 3 rad/s is followed at the rate limit.
	ModeOverlayHeading spinning;
	Recorder spin;
	spinning.update(0.f, 0.f, kNan, kNan, false, 0.f, kDt, kLimits);
	spin.record(spinning);
	float target = 0.f;

	for (int i = 0; i < 1000; ++i) {
		target = matrix::wrap_pi(target + 3.f * kDt);
		spinning.update(0.f, 0.f, target, 3.f, false, 0.f, kDt, kLimits);
		spin.record(spinning);
	}

	EXPECT_LE(spin.max_rate, kLimits.rate + kTolerance);
	EXPECT_LE(spin.max_acceleration, kLimits.acceleration + kTolerance / kDt);
	EXPECT_LE(spin.max_step, kLimits.rate * kDt + kTolerance);
}

TEST(ModeOverlayHeadingTest, HandsBackToTheModeHeadingWithoutAStep)
{
	ModeOverlayHeading heading;
	heading.update(1.f, 0.f, kNan, kNan, false, 0.f, kDt, kLimits);

	for (int i = 0; i < 10; ++i) { heading.update(1.f, 0.f, 1.f, 0.f, false, 0.f, kDt, kLimits); }

	ASSERT_TRUE(heading.active());
	Recorder recorder;
	recorder.record(heading);
	int steps = 0;

	for (; steps < 1000 && heading.active(); ++steps) {
		heading.update(-0.5f, 0.f, kNan, kNan, false, 0.f, kDt, kLimits);
		recorder.record(heading);
	}

	EXPECT_FALSE(heading.active());
	EXPECT_LE(recorder.max_step, kLimits.rate * kDt + kTolerance);
	EXPECT_LE(recorder.max_acceleration, kLimits.acceleration + kTolerance / kDt);
	// Once returned, the mode heading passes through bit for bit.
	heading.update(-0.45f, 0.1f, kNan, kNan, false, 0.f, kDt, kLimits);
	EXPECT_TRUE(sameBits(heading.yaw(), -0.45f));
	EXPECT_TRUE(sameBits(heading.yawspeed(), 0.1f));
}

TEST(ModeOverlayHeadingTest, HandsBackToAnUncontrolledHeadingByStoppingTheTurn)
{
	ModeOverlayHeading heading;
	heading.update(0.f, 0.f, kNan, kNan, false, 0.f, kDt, kLimits);
	float target = 0.f;

	for (int i = 0; i < 600; ++i) {
		target += 0.5f * kDt;
		heading.update(0.f, 0.f, target, 0.5f, false, 0.f, kDt, kLimits);
	}

	ASSERT_NEAR(heading.yawspeed(), 0.5f, kTolerance);
	int steps = 0;

	for (; steps < 1000 && heading.active(); ++steps) {
		heading.update(kNan, kNan, kNan, kNan, false, 0.f, kDt, kLimits);

		if (heading.active()) { EXPECT_GE(heading.yawspeed(), -kTolerance); }
	}

	EXPECT_FALSE(heading.active());
	EXPECT_TRUE(std::isnan(heading.yaw()));
	// Stops within 0.5/0.349 = 1.43 s; the turn only decelerates.
	EXPECT_NEAR(steps * kDt, 0.5f / kLimits.acceleration, 0.02f);
}

TEST(ModeOverlayHeadingTest, BrakeHeadingKeepsTheReferenceEngaged)
{
	ModeOverlayHeading heading;
	heading.update(0.f, 0.f, kNan, kNan, false, 0.f, kDt, kLimits);

	for (int i = 0; i < 600; ++i) { heading.update(0.f, 0.f, 1.2f, 0.f, false, 0.f, kDt, kLimits); }

	ASSERT_FLOAT_EQ(heading.yaw(), 1.2f);

	// The brake holds the heading estimate at its start; the engaged reference settles on it.
	for (int i = 0; i < 300; ++i) { heading.update(1.15f, 0.f, kNan, kNan, true, 0.f, kDt, kLimits); }

	EXPECT_FLOAT_EQ(heading.yaw(), 1.15f);
	EXPECT_TRUE(heading.active());

	// Afterwards the mode heading takes over without a step.
	Recorder recorder;
	recorder.record(heading);

	for (int i = 0; i < 1000 && heading.active(); ++i) {
		heading.update(0.f, 0.f, kNan, kNan, false, 0.f, kDt, kLimits);
		recorder.record(heading);
	}

	EXPECT_FALSE(heading.active());
	EXPECT_LE(recorder.max_step, kLimits.rate * kDt + kTolerance);
}

TEST(ModeOverlayHeadingTest, TakesTheShortWayAcrossPi)
{
	ModeOverlayHeading heading;
	heading.update(3.1f, 0.f, kNan, kNan, false, 0.f, kDt, kLimits);
	float furthest = 0.f;

	for (int i = 0; i < 500; ++i) {
		heading.update(3.1f, 0.f, -3.1f, 0.f, false, 0.f, kDt, kLimits);
		furthest = fmaxf(furthest, fabsf(matrix::wrap_pi(heading.yaw() - 3.1f)));
	}

	EXPECT_FLOAT_EQ(heading.yaw(), -3.1f);
	EXPECT_LE(furthest, 2.f * M_PI_F - 6.2f + kTolerance);
}

TEST(ModeOverlayHeadingTest, HeadingResetShiftsTheReference)
{
	ModeOverlayHeading heading;
	heading.update(0.f, 0.f, kNan, kNan, false, 0.f, kDt, kLimits);

	for (int i = 0; i < 600; ++i) { heading.update(0.f, 0.f, 1.f, 0.f, false, 0.f, kDt, kLimits); }

	heading.shift(0.5f);
	EXPECT_FLOAT_EQ(heading.yaw(), 1.5f);
	heading.shift(kNan);
	EXPECT_FLOAT_EQ(heading.yaw(), 1.5f);
	heading.shift(2.f);
	EXPECT_NEAR(heading.yaw(), 3.5f - 2.f * M_PI_F, kTolerance);
	// The companion heading in the new frame is tracked exactly again.
	heading.update(0.f, 0.f, heading.yaw(), 0.f, false, 0.f, kDt, kLimits);
	EXPECT_NEAR(heading.yaw(), 3.5f - 2.f * M_PI_F, kTolerance);
}

TEST(ModeOverlayHeadingTest, ResetAndInvalidPeriodsAreSafe)
{
	ModeOverlayHeading heading;
	heading.update(0.f, 0.f, kNan, kNan, false, 0.f, kDt, kLimits);
	heading.update(0.f, 0.f, 1.f, 0.f, false, 0.f, kDt, kLimits);
	const float yaw = heading.yaw();

	for (float dt : {0.f, -0.01f, kNan}) {
		heading.update(0.f, 0.f, 1.f, 0.f, false, 0.f, dt, kLimits);
		EXPECT_FLOAT_EQ(heading.yaw(), yaw);
	}

	heading.reset();
	EXPECT_FALSE(heading.active());
	EXPECT_TRUE(std::isnan(heading.yaw()));
	heading.shift(1.f);
	EXPECT_TRUE(std::isnan(heading.yaw()));
}
