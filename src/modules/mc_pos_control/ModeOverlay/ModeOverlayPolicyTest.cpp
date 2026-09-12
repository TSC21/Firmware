/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "ModeOverlayPolicy.hpp"
#include <gtest/gtest.h>
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
	output.setpoint.acceleration[0] = config.max_acceleration + 1.f;
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
