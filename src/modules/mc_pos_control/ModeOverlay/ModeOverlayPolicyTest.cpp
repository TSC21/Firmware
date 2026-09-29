/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "ModeOverlayPolicy.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <limits>

class ModeOverlayPolicyTest : public ::testing::Test
{
protected:
	ModeOverlayPolicy policy;
	ModeOverlayPolicy::Config config;
	uint64_t now{1000000};
	trajectory_setpoint_s raw{};

	void SetUp() override
	{
		config.enabled = true;
		policy.configure(config);
		raw.position[0] = 100.f; // A distant mission target is not a local authority bound.
		raw.yaw = 1.f;
		ASSERT_EQ(policy.request(request(), false, now).result, mode_overlay_reply_s::RESULT_ACCEPTED);
		policy.updateContext(now, true, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, 0, true);
	}
	mode_overlay_request_s request(uint64_t session = 17)
	{
		mode_overlay_request_s request{};
		request.session_id = session;
		request.request_id = 29;
		request.applicable_modes = ModeOverlayPolicy::SUPPORTED_MODES;
		request.max_deviation = 3.f;
		strcpy(request.name, "Navigation");
		return request;
	}
	mode_overlay_output_s response(uint8_t action = mode_overlay_output_s::ACTION_REPLACE)
	{
		mode_overlay_output_s output{};
		output.session_id = policy.session();
		output.intent_id = policy.input(now, raw).intent_id;
		output.sequence = 1;
		output.ready = true;
		output.action = action;
		output.setpoint.position[0] = 1.f;
		output.setpoint.velocity[0] = 0.5f;
		return output;
	}
	ModeOverlayPolicy::Selection select()
	{
		auto effective = raw;
		return policy.select(now, effective, {});
	}
};

TEST_F(ModeOverlayPolicyTest, ReplaceIsLocalAndDoesNotMutateIntentOrYaw)
{
	ASSERT_TRUE(policy.output(response(), {}, now));
	auto effective = raw;
	EXPECT_EQ(policy.select(now, effective, {}), ModeOverlayPolicy::Selection::Replace);
	EXPECT_FLOAT_EQ(effective.position[0], 1.f);
	EXPECT_FLOAT_EQ(effective.yaw, raw.yaw);
	EXPECT_FLOAT_EQ(raw.position[0], 100.f);
	EXPECT_TRUE(policy.status(now).engaged);
}

TEST_F(ModeOverlayPolicyTest, CompetingRegistrationIsRejected)
{
	EXPECT_EQ(policy.request(request(42), false, now).result, mode_overlay_reply_s::RESULT_BUSY);
	EXPECT_EQ(policy.session(), 17u);
	EXPECT_EQ(policy.request(request(), true, now).result, mode_overlay_reply_s::RESULT_ARMED);
}

TEST_F(ModeOverlayPolicyTest, RegistrationRetriesPreserveSequence)
{
	const auto output = response();
	ASSERT_TRUE(policy.output(output, {}, now));
	EXPECT_EQ(policy.request(request(), false, now).result, mode_overlay_reply_s::RESULT_ACCEPTED);
	EXPECT_FALSE(policy.output(output, {}, now));
	EXPECT_TRUE(policy.ready(now));
}

TEST_F(ModeOverlayPolicyTest, LeaseCannotBeStolenWhileArmed)
{
	now += 3000000;
	EXPECT_EQ(policy.request(request(42), true, now).result, mode_overlay_reply_s::RESULT_ARMED);
	EXPECT_EQ(policy.request(request(42), false, now).result, mode_overlay_reply_s::RESULT_ACCEPTED);
	EXPECT_FALSE(policy.ready(now));
}

TEST_F(ModeOverlayPolicyTest, RejectsWrongOwnerAndUnknownIntent)
{
	auto output = response();
	output.session_id = 99;
	EXPECT_FALSE(policy.output(output, {}, now));
	output.session_id = 17;
	output.intent_id += 1000;
	EXPECT_FALSE(policy.output(output, {}, now));
	EXPECT_FALSE(policy.ready(now));
}

TEST_F(ModeOverlayPolicyTest, RejectsStaleIntentEvenWhenReceiveTimeIsCurrent)
{
	const auto output = response();
	now += config.timeout_us + 1;
	EXPECT_FALSE(policy.output(output, {}, now));
	EXPECT_EQ(select(), ModeOverlayPolicy::Selection::Brake);
	EXPECT_TRUE(policy.failed());
}

TEST_F(ModeOverlayPolicyTest, RejectsFutureTimeAndDuplicateSequence)
{
	const auto output = response();
	EXPECT_FALSE(policy.output(output, {}, now - 1));
	ASSERT_TRUE(policy.output(output, {}, now));
	EXPECT_FALSE(policy.output(output, {}, now));
	EXPECT_EQ(policy.status(now).accepted_sequence, 1u);
}

TEST_F(ModeOverlayPolicyTest, LossLatchesUntilDisarm)
{
	ASSERT_TRUE(policy.output(response(), {}, now));
	now += config.timeout_us + 1;
	EXPECT_EQ(select(), ModeOverlayPolicy::Selection::Brake);
	auto recovered = response();
	recovered.sequence = 2;
	ASSERT_TRUE(policy.output(recovered, {}, now));
	EXPECT_EQ(select(), ModeOverlayPolicy::Selection::Brake);
	EXPECT_TRUE(policy.failed());
	policy.updateContext(now, false, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, 0, true);
	EXPECT_FALSE(policy.failed());
	EXPECT_FALSE(policy.ready(now));
}

TEST_F(ModeOverlayPolicyTest, EstimatorResetInvalidatesOutstandingWork)
{
	const auto output = response();
	policy.updateContext(now, true, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, 1, true);
	EXPECT_FALSE(policy.output(output, {}, now));
	EXPECT_EQ(select(), ModeOverlayPolicy::Selection::Brake);
	EXPECT_FALSE(policy.failed());
}

TEST_F(ModeOverlayPolicyTest, ModeSwitchInvalidatesOutstandingWork)
{
	const auto output = response();
	policy.updateContext(now, true, vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER, 0, true);
	EXPECT_FALSE(policy.output(output, {}, now));
	EXPECT_EQ(select(), ModeOverlayPolicy::Selection::Brake);
}

TEST_F(ModeOverlayPolicyTest, ExplicitStopIsHealthyAndNotEngagedReplacement)
{
	ASSERT_TRUE(policy.output(response(mode_overlay_output_s::ACTION_STOP), {}, now));
	EXPECT_EQ(select(), ModeOverlayPolicy::Selection::Brake);
	EXPECT_TRUE(policy.status(now).braking);
	EXPECT_TRUE(policy.ready(now));
	EXPECT_FALSE(policy.status(now).engaged);
	EXPECT_FALSE(policy.failed());
}

TEST_F(ModeOverlayPolicyTest, UnreadyCannotAuthorizePassthrough)
{
	auto output = response(mode_overlay_output_s::ACTION_PASSTHROUGH);
	output.ready = false;
	ASSERT_TRUE(policy.output(output, {}, now));
	EXPECT_EQ(select(), ModeOverlayPolicy::Selection::Brake);
}

TEST_F(ModeOverlayPolicyTest, InvalidOrOverLimitReplacementFailsClosed)
{
	for (float bad : {
		     std::numeric_limits<float>::quiet_NaN(),
		     std::numeric_limits<float>::infinity(), 10.f
	     }) {
		auto output = response();
		output.setpoint.position[0] = bad;
		EXPECT_FALSE(policy.output(output, {}, now));
		EXPECT_EQ(select(), ModeOverlayPolicy::Selection::Brake);
	}
	EXPECT_TRUE(policy.failed());
}

TEST_F(ModeOverlayPolicyTest, EnforcesVelocityAccelerationAndJerk)
{
	auto output = response();
	output.setpoint.velocity[2] = -config.max_velocity_up - 1.f;
	EXPECT_FALSE(policy.output(output, {}, now));
	output = response();
	output.setpoint.acceleration[0] = config.max_acceleration_xy + 1.f;
	EXPECT_FALSE(policy.output(output, {}, now));
	output = response();
	output.setpoint.jerk[0] = config.max_jerk + 1.f;
	EXPECT_FALSE(policy.output(output, {}, now));
}

TEST_F(ModeOverlayPolicyTest, LandingAndNonPositionModesRemainOutsideOverlay)
{
	for (uint8_t mode : {
		     vehicle_status_s::NAVIGATION_STATE_AUTO_LAND,
		     vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF,
		     vehicle_status_s::NAVIGATION_STATE_STAB,
		     vehicle_status_s::NAVIGATION_STATE_TERMINATION, uint8_t(255)
	     }) {
		policy.updateContext(now, true, mode, 0, true);
		EXPECT_EQ(select(), ModeOverlayPolicy::Selection::Passthrough);
		EXPECT_FALSE(policy.status(now).applicable);
	}
}

TEST_F(ModeOverlayPolicyTest, DisableRevokesSession)
{
	ASSERT_TRUE(policy.output(response(), {}, now));
	config.enabled = false;
	policy.configure(config);
	EXPECT_EQ(policy.session(), 0u);
	EXPECT_FALSE(policy.ready(now));
	EXPECT_EQ(policy.request(request(), false, now).result, mode_overlay_reply_s::RESULT_DISABLED);
}

TEST_F(ModeOverlayPolicyTest, RechecksAuthorityAgainstCurrentVehiclePosition)
{
	ASSERT_TRUE(policy.output(response(), {}, now));
	auto effective = raw;
	EXPECT_EQ(policy.select(now, effective, {5.f, 0.f, 0.f}), ModeOverlayPolicy::Selection::Brake);
	EXPECT_TRUE(policy.failed());
}

TEST_F(ModeOverlayPolicyTest, EligibilityChangeInvalidatesGroundPassthrough)
{
	policy.updateContext(now, true, vehicle_status_s::NAVIGATION_STATE_OFFBOARD, 0, false);
	const auto output = response(mode_overlay_output_s::ACTION_PASSTHROUGH);
	ASSERT_TRUE(policy.output(output, {}, now));
	policy.updateContext(now, true, vehicle_status_s::NAVIGATION_STATE_OFFBOARD, 0, true);
	EXPECT_FALSE(policy.output(output, {}, now));
	EXPECT_EQ(select(), ModeOverlayPolicy::Selection::Brake);
	EXPECT_FALSE(policy.failed());
}

TEST_F(ModeOverlayPolicyTest, InvalidActionCannotPreserveReadiness)
{
	auto output = response();
	output.action = 255;
	EXPECT_FALSE(policy.output(output, {}, now));
	EXPECT_TRUE(policy.failed());
}

TEST_F(ModeOverlayPolicyTest, ReplaceReferenceAdvancesBetweenOutputs)
{
	auto output = response();
	output.setpoint.acceleration[0] = 0.2f;
	output.setpoint.jerk[0] = 0.1f;
	ASSERT_TRUE(policy.output(output, {}, now));
	now += 10000; // one 100 Hz controller period later, before the next 50 Hz output
	auto effective = raw;
	ASSERT_EQ(policy.select(now, effective, {}), ModeOverlayPolicy::Selection::Replace);
	const float t = 0.01f;
	EXPECT_NEAR(effective.position[0], 1.f + 0.5f * t + 0.1f * t * t + 0.1f * t * t * t / 6.f, 1e-6f);
	EXPECT_NEAR(effective.velocity[0], 0.5f + 0.2f * t + 0.05f * t * t, 1e-6f);
	EXPECT_NEAR(effective.acceleration[0], 0.2f + 0.1f * t, 1e-6f);
	EXPECT_FLOAT_EQ(effective.jerk[0], 0.1f);
}

TEST_F(ModeOverlayPolicyTest, PropagationStopsAtTheResponseTimeout)
{
	ASSERT_TRUE(policy.output(response(), {}, now));
	// A publisher stamp older than the timeout does not reach back in time.
	auto stale = response();
	stale.sequence = 2;
	stale.timestamp = now - 2 * config.timeout_us;
	ASSERT_TRUE(policy.output(stale, {}, now));
	auto effective = raw;
	ASSERT_EQ(policy.select(now, effective, {}), ModeOverlayPolicy::Selection::Replace);
	EXPECT_FLOAT_EQ(effective.position[0], 1.f);

	// A fresh publisher stamp sets the epoch.
	auto stamped = response();
	stamped.sequence = 3;
	stamped.timestamp = now - 20000;
	ASSERT_TRUE(policy.output(stamped, {}, now));
	ASSERT_EQ(policy.select(now, effective, {}), ModeOverlayPolicy::Selection::Replace);
	EXPECT_NEAR(effective.position[0], 1.f + 0.5f * 0.02f, 1e-6f);

	// An old but fresh stamp selected late advances the reference by the timeout, no further.
	auto lagging = response();
	lagging.sequence = 4;
	lagging.timestamp = now - 250000;
	ASSERT_TRUE(policy.output(lagging, {}, now));
	now += 100000;
	ASSERT_EQ(policy.select(now, effective, {1.f, 0.f, 0.f}), ModeOverlayPolicy::Selection::Replace);
	EXPECT_NEAR(effective.position[0], 1.f + 0.5f * config.timeout_us * 1e-6f, 1e-6f);
}

TEST_F(ModeOverlayPolicyTest, HighSpeedTrackingLagDoesNotTripAuthorityChecks)
{
	// 8 m/s straight line, 50 Hz responses sampled 40 ms ahead, a controller at 100 and
	// 250 Hz and a vehicle 0.5 m behind the reference: nothing may be rejected.
	for (const uint64_t period_us : {10000ull, 4000ull}) {
		policy = ModeOverlayPolicy{};
		policy.configure(config);
		ASSERT_EQ(policy.request(request(), false, now).result, mode_overlay_reply_s::RESULT_ACCEPTED);
		policy.updateContext(now, true, vehicle_status_s::NAVIGATION_STATE_OFFBOARD, 0, true);
		const float speed = 8.f;
		const float lookahead = 0.04f;
		const float lag = 0.5f;
		uint64_t sequence = 0;
		uint64_t start = now;
		ModeOverlayPolicy::Config fast = config;
		fast.max_velocity_xy = 10.f;
		policy.configure(fast);

		for (uint64_t t = start; t < start + 2000000; t += period_us) {
			const float time = (t - start) * 1e-6f;
			const float along_track = speed * time - lag;
			const matrix::Vector3f vehicle{along_track, 0.f, -2.f};

			if ((t - start) % 20000 < period_us) {
				mode_overlay_output_s output{};
				output.session_id = policy.session();
				output.intent_id = policy.input(t, raw).intent_id;
				output.sequence = ++sequence;
				output.ready = true;
				output.action = mode_overlay_output_s::ACTION_REPLACE;
				output.setpoint.position[0] = speed * (time + lookahead);
				output.setpoint.position[2] = -2.f;
				output.setpoint.velocity[0] = speed;
				ASSERT_TRUE(policy.output(output, vehicle, t));
			}

			auto effective = raw;
			ASSERT_EQ(policy.select(t, effective, vehicle), ModeOverlayPolicy::Selection::Replace);
			// The propagated reference is continuous: always 40 ms ahead of the controller time.
			EXPECT_NEAR(effective.position[0], speed * (time + lookahead), 1e-3f);
		}

		EXPECT_FALSE(policy.failed());
		EXPECT_EQ(policy.status(start).rejected_outputs, 0u);
	}
}

TEST_F(ModeOverlayPolicyTest, DirectionalAccelerationLimits)
{
	config.max_acceleration_xy = 5.f;
	config.max_acceleration_up = 4.f;
	config.max_acceleration_down = 3.f;
	policy.configure(config);
	auto output = response();
	output.setpoint.acceleration[0] = 3.f;
	output.setpoint.acceleration[1] = 4.f; // |a_xy| = 5
	output.setpoint.acceleration[2] = -4.f; // upwards
	ASSERT_TRUE(policy.output(output, {}, now));
	output.sequence = 2;
	output.setpoint.acceleration[2] = 3.f; // downwards
	ASSERT_TRUE(policy.output(output, {}, now));

	for (const matrix::Vector3f &over : {
		     matrix::Vector3f{3.1f, 4.f, 0.f}, matrix::Vector3f{0.f, 0.f, -4.1f},
		     matrix::Vector3f{0.f, 0.f, 3.1f}
	     }) {
		auto rejected = response();
		rejected.sequence = 3;
		over.copyTo(rejected.setpoint.acceleration);
		EXPECT_FALSE(policy.output(rejected, {}, now));
	}

	const auto status = policy.status(now);
	EXPECT_FLOAT_EQ(status.max_acceleration, 3.f);
	EXPECT_FLOAT_EQ(status.max_acceleration_xy, 5.f);
	EXPECT_FLOAT_EQ(status.max_acceleration_up, 4.f);
	EXPECT_FLOAT_EQ(status.max_acceleration_down, 3.f);
	EXPECT_FLOAT_EQ(status.timeout, 0.3f);
}
