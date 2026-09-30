/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Functional test of FlightModeManager's navigation overlay anchoring: while a
 * fresh mode_overlay_status reports that the overlay replaces or brakes the
 * reference, the Auto task restarts every cycle from the vehicle state, so Hold,
 * Mission, Return and Land continue from the vehicle once the overlay releases the
 * reference. Every other case (no, stale or disabled status, manual tasks) runs the
 * task exactly as without an overlay.
 *
 * The scenarios move the vehicle estimate by 20 m between two manager cycles, as an
 * overlay manoeuvre does between hand-over and hand-back, and read the published
 * trajectory_setpoint. Cycles run microseconds apart in wall time, so the smoothers
 * advance by well under a millimetre per cycle: an anchored task lies on the vehicle
 * and an unanchored one where the manoeuvre started, 20 m away.
 *
 * to run: make tests TESTFILTER=FlightModeManagerOverlay
 */

#include <gtest/gtest.h>

#include "FlightModeManager.hpp"

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/parameters/param.h>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mode_overlay_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;
using matrix::Vector2f;

class FlightModeManagerTestPeer : public FlightModeManager
{
public:
	FlightTaskError switchTo(FlightTaskIndex index) { return switchTask(index); }
	FlightTaskIndex taskIndex() const { return _current_task.index; }
	void cycle(const vehicle_local_position_s &local) { generateTrajectorySetpoint(0.02f, local); }
};

namespace
{
// PX4 SITL home; the local frame origin of every scenario.
constexpr double kRefLat = 47.397742;
constexpr double kRefLon = 8.545594;
constexpr float kRefAlt = 488.f; // [m AMSL]
constexpr float kAltitude = 5.f; // hold altitude above the origin [m]
// An anchored setpoint lies within the distance the smoother covers in one cycle
// (microseconds of wall time at <= 5 m/s) of the vehicle; 5 cm is two orders of
// magnitude above that and two below the 20 m manoeuvre.
constexpr float kOnVehicle = 0.05f; // [m]
constexpr float kManoeuvre = 20.f; // [m]
}

class FlightModeManagerOverlayTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
		_projection.initReference(kRefLat, kRefLon, _ref_timestamp);
		_manager = new FlightModeManagerTestPeer();
		publishFlight();
	}

	void TearDown() override
	{
		delete _manager;
		// Later tests start from a status that owns nothing.
		publishStatus(false, false, hrt_absolute_time());
	}

	void publishFlight()
	{
		takeoff_status_s takeoff{};
		takeoff.takeoff_state = takeoff_status_s::TAKEOFF_STATE_FLIGHT;
		takeoff.timestamp = hrt_absolute_time();
		_takeoff_pub.publish(takeoff);

		home_position_s home{};
		home.lat = kRefLat;
		home.lon = kRefLon;
		home.alt = kRefAlt;
		home.valid_hpos = home.valid_lpos = home.valid_alt = true;
		home.timestamp = hrt_absolute_time();
		_home_pub.publish(home);
	}

	void publishVehicleStatus(uint8_t nav_state)
	{
		vehicle_status_s status{};
		status.nav_state = nav_state;
		status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		status.timestamp = hrt_absolute_time();
		_vehicle_status_pub.publish(status);
	}

	void publishTriplet(uint8_t type, const Vector2f &target_ne, float yaw = NAN)
	{
		position_setpoint_triplet_s triplet{};
		triplet.current.valid = true;
		triplet.current.type = type;
		_projection.reproject(target_ne(0), target_ne(1), triplet.current.lat, triplet.current.lon);
		triplet.current.alt = kRefAlt + kAltitude;
		triplet.current.yaw = yaw;
		triplet.current.cruising_speed = NAN;
		triplet.current.acceptance_radius = 1.f;
		triplet.current.timestamp = hrt_absolute_time();
		triplet.timestamp = triplet.current.timestamp;
		_triplet_pub.publish(triplet);
	}

	void publishStatus(bool engaged, bool braking, hrt_abstime timestamp, bool enabled = true)
	{
		mode_overlay_status_s status{};
		status.enabled = enabled;
		status.registered = enabled;
		status.engaged = engaged;
		status.braking = braking;
		status.timestamp = timestamp;
		_overlay_status_pub.publish(status);
	}

	/** Publish the vehicle estimate. */
	vehicle_local_position_s publishLocal(const Vector2f &position_ne, const Vector2f &velocity_ne = {},
					      float heading = 0.f)
	{
		vehicle_local_position_s local{};
		local.timestamp = local.timestamp_sample = hrt_absolute_time();
		local.x = position_ne(0);
		local.y = position_ne(1);
		local.z = -kAltitude;
		local.vx = velocity_ne(0);
		local.vy = velocity_ne(1);
		local.heading = heading;
		local.heading_good_for_control = true;
		local.xy_valid = local.z_valid = local.v_xy_valid = local.v_z_valid = true;
		local.xy_global = local.z_global = true;
		local.ref_lat = kRefLat;
		local.ref_lon = kRefLon;
		local.ref_alt = kRefAlt;
		local.ref_timestamp = _ref_timestamp;
		local.dist_bottom = NAN;
		local.vxy_max = local.vz_max = local.hagl_min = local.hagl_max_z = local.hagl_max_xy = INFINITY;
		_local_position_pub.publish(local);
		return local;
	}

	/** Publish the vehicle estimate and run one manager cycle of the active task on it. */
	trajectory_setpoint_s cycle(const Vector2f &position_ne, const Vector2f &velocity_ne = {}, float heading = 0.f)
	{
		_manager->cycle(publishLocal(position_ne, velocity_ne, heading));
		trajectory_setpoint_s setpoint{};
		EXPECT_TRUE(_setpoint_sub.update(&setpoint));
		return setpoint;
	}

	/** Hold at the origin with the Auto task running on its own trajectory. */
	void startHold(float triplet_yaw = NAN)
	{
		publishVehicleStatus(vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER);
		publishTriplet(position_setpoint_s::SETPOINT_TYPE_LOITER, Vector2f{0.f, 0.f}, triplet_yaw);
		publishLocal(Vector2f{0.f, 0.f});
		ASSERT_EQ(_manager->switchTo(FlightTaskIndex::Auto), FlightTaskError::NoError);

		for (int i = 0; i < 5; ++i) {
			const trajectory_setpoint_s setpoint = cycle(Vector2f{0.f, 0.f});
			ASSERT_LT(horizontalDistance(setpoint, Vector2f{0.f, 0.f}), kOnVehicle);
		}
	}

	static float horizontalDistance(const trajectory_setpoint_s &setpoint, const Vector2f &position_ne)
	{
		return (Vector2f{setpoint.position[0], setpoint.position[1]} - position_ne).norm();
	}

	FlightModeManagerTestPeer *_manager{nullptr};
	MapProjection _projection{};
	const hrt_abstime _ref_timestamp{1_s};

	uORB::Publication<home_position_s> _home_pub{ORB_ID(home_position)};
	uORB::Publication<mode_overlay_status_s> _overlay_status_pub{ORB_ID(mode_overlay_status)};
	uORB::Publication<position_setpoint_triplet_s> _triplet_pub{ORB_ID(position_setpoint_triplet)};
	uORB::Publication<takeoff_status_s> _takeoff_pub{ORB_ID(takeoff_status)};
	uORB::Publication<vehicle_local_position_s> _local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<manual_control_setpoint_s> _manual_pub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _setpoint_sub{ORB_ID(trajectory_setpoint)};
};

// Declared first so that it runs before any test publishes an overlay status.
TEST_F(FlightModeManagerOverlayTest, WithoutOverlayStatusLandStartsWhereTheManoeuvreBegan)
{
	// GIVEN: Hold at the origin and no overlay in the system
	startHold();

	// WHEN: the vehicle ends up 20 m away and the operator switches to Land there
	const Vector2f vehicle{kManoeuvre, -5.f};
	cycle(vehicle);
	publishVehicleStatus(vehicle_status_s::NAVIGATION_STATE_AUTO_LAND);
	publishTriplet(position_setpoint_s::SETPOINT_TYPE_LAND, vehicle);
	const trajectory_setpoint_s setpoint = cycle(vehicle);

	// THEN: the task trajectory still starts at the origin (the 23.5 m Land excursion of
	// SITL run A), which is the behaviour the anchoring removes for overlay manoeuvres
	EXPECT_LT(horizontalDistance(setpoint, Vector2f{0.f, 0.f}), kOnVehicle);
	EXPECT_GT(horizontalDistance(setpoint, vehicle), kManoeuvre - 1.f);
}

TEST_F(FlightModeManagerOverlayTest, AutoTaskFollowsTheVehicleWhileTheOverlayReplaces)
{
	startHold();

	// WHEN: the overlay replaces the reference and flies the vehicle 20 m at 5 m/s
	for (int i = 1; i <= 10; ++i) {
		publishStatus(true, false, hrt_absolute_time());
		const Vector2f vehicle{2.f * i, 0.f};
		const trajectory_setpoint_s setpoint = cycle(vehicle, Vector2f{5.f, 0.f});

		// THEN: every cycle the task trajectory restarts from the vehicle state
		EXPECT_LT(horizontalDistance(setpoint, vehicle), kOnVehicle) << "cycle " << i;
		EXPECT_NEAR(setpoint.velocity[0], 5.f, 0.05f) << "cycle " << i;
		EXPECT_NEAR(setpoint.velocity[1], 0.f, 0.05f) << "cycle " << i;
	}
}

TEST_F(FlightModeManagerOverlayTest, AutoTaskFollowsTheVehicleWhileTheOverlayBrakes)
{
	startHold();

	// WHEN: the overlay brake owns the reference and the vehicle comes to rest 20 m away
	publishStatus(false, true, hrt_absolute_time());
	const Vector2f vehicle{0.f, kManoeuvre};
	const trajectory_setpoint_s setpoint = cycle(vehicle);

	// THEN: the task trajectory is on the vehicle
	EXPECT_LT(horizontalDistance(setpoint, vehicle), kOnVehicle);
}

TEST_F(FlightModeManagerOverlayTest, LandAfterAnOverlayStopStartsAtTheVehicle)
{
	// GIVEN: Hold at the origin, then an overlay dash and stop that leaves the vehicle 20 m away
	startHold();
	const Vector2f vehicle{kManoeuvre, -5.f};
	publishStatus(true, false, hrt_absolute_time());
	cycle(Vector2f{10.f, -2.5f}, Vector2f{5.f, -1.25f});
	publishStatus(false, true, hrt_absolute_time());
	cycle(vehicle);

	// WHEN: the operator switches to Land, which the overlay does not own
	publishStatus(false, false, hrt_absolute_time());
	publishVehicleStatus(vehicle_status_s::NAVIGATION_STATE_AUTO_LAND);
	publishTriplet(position_setpoint_s::SETPOINT_TYPE_LAND, vehicle);
	const trajectory_setpoint_s setpoint = cycle(vehicle);

	// THEN: Land starts at the vehicle instead of flying back to the Hold point
	EXPECT_LT(horizontalDistance(setpoint, vehicle), kOnVehicle);
}

TEST_F(FlightModeManagerOverlayTest, HandBackContinuesTheModeFromTheVehicle)
{
	// GIVEN: Hold at the origin and an overlay manoeuvre that ends 20 m away at rest
	startHold();
	const Vector2f vehicle{-kManoeuvre, 0.f};
	publishStatus(true, false, hrt_absolute_time());
	cycle(vehicle);

	// WHEN: the overlay passes the reference through again
	publishStatus(false, false, hrt_absolute_time());
	const trajectory_setpoint_s setpoint = cycle(vehicle);

	// THEN: Hold resumes from the vehicle and returns to its point on its own trajectory
	EXPECT_LT(horizontalDistance(setpoint, vehicle), kOnVehicle);
}

TEST_F(FlightModeManagerOverlayTest, StaleOrDisabledStatusLeavesTheTaskUnanchored)
{
	// hrt counts from process start; the stale stamp below must not wrap around.
	while (hrt_absolute_time() < 200_ms) { px4_usleep(10_ms); }

	startHold();
	const Vector2f vehicle{kManoeuvre, 0.f};

	// A status older than 100 ms describes an earlier selection.
	publishStatus(true, true, hrt_absolute_time() - 150_ms);
	EXPECT_GT(horizontalDistance(cycle(vehicle), vehicle), kManoeuvre - 1.f);

	// With COM_OVL_EN=0 the overlay publishes an idle status at 1 Hz.
	publishStatus(false, false, hrt_absolute_time(), false);
	EXPECT_GT(horizontalDistance(cycle(vehicle), vehicle), kManoeuvre - 1.f);

	// A fresh status just inside the timeout still anchors.
	publishStatus(true, false, hrt_absolute_time() - 50_ms);
	EXPECT_LT(horizontalDistance(cycle(vehicle), vehicle), kOnVehicle);
}

TEST_F(FlightModeManagerOverlayTest, InvalidEstimateNeverSeedsTheAnchor)
{
	startHold();
	publishStatus(true, false, hrt_absolute_time());

	// WHEN: the horizontal estimate is invalid for one cycle while the overlay owns the reference
	vehicle_local_position_s local = publishLocal(Vector2f{0.f, 0.f});
	local.xy_valid = false;
	local.v_xy_valid = false;
	_local_position_pub.publish(local);
	_manager->cycle(local);
	trajectory_setpoint_s setpoint{};
	ASSERT_TRUE(_setpoint_sub.update(&setpoint));

	// THEN: the manager publishes the empty setpoint, so the position controller fails safe
	EXPECT_FALSE(PX4_ISFINITE(setpoint.position[0]) || PX4_ISFINITE(setpoint.velocity[0]));

	// AND: when the estimate recovers after the overlay released the reference, the task
	// continues from its last valid state instead of from a NaN seed
	publishStatus(false, false, hrt_absolute_time());
	setpoint = cycle(Vector2f{0.f, 0.f});
	EXPECT_LT(horizontalDistance(setpoint, Vector2f{0.f, 0.f}), kOnVehicle);
}

TEST_F(FlightModeManagerOverlayTest, HeadingSmoothingContinuesFromTheTaskHeading)
{
	// GIVEN: Hold with a heading of 0 rad
	startHold(0.f);

	// WHEN: the overlay owns the reference and has turned the vehicle to 1.5 rad
	publishStatus(true, false, hrt_absolute_time());
	const trajectory_setpoint_s setpoint = cycle(Vector2f{0.f, 0.f}, Vector2f{}, 1.5f);

	// THEN: the task heading continues from its own smoothed state instead of stepping to
	// the vehicle heading (the smoother moves by at most MPC_YAWRAUTO_MAX times a cycle)
	ASSERT_TRUE(PX4_ISFINITE(setpoint.yaw));
	EXPECT_LT(fabsf(matrix::wrap_pi(setpoint.yaw)), 0.05f);
}

TEST_F(FlightModeManagerOverlayTest, ManualTasksAreNeverAnchored)
{
	// GIVEN: Position mode with centred sticks, holding the origin
	manual_control_setpoint_s manual{};
	manual.valid = true;
	manual.data_source = manual_control_setpoint_s::SOURCE_RC;
	manual.timestamp = manual.timestamp_sample = hrt_absolute_time();
	_manual_pub.publish(manual);
	publishVehicleStatus(vehicle_status_s::NAVIGATION_STATE_POSCTL);
	publishLocal(Vector2f{0.f, 0.f});
	ASSERT_EQ(_manager->switchTo(FlightTaskIndex::ManualAcceleration), FlightTaskError::NoError);
	const trajectory_setpoint_s held = cycle(Vector2f{0.f, 0.f});

	// WHEN: an overlay status reports an engaged overlay and the estimate moves 20 m
	manual.timestamp = manual.timestamp_sample = hrt_absolute_time();
	_manual_pub.publish(manual);
	publishStatus(true, true, hrt_absolute_time());
	const trajectory_setpoint_s setpoint = cycle(Vector2f{kManoeuvre, 0.f});

	// THEN: the manual task keeps its own reference: the pilot's request is not restarted
	// from the vehicle
	EXPECT_EQ(_manager->taskIndex(), FlightTaskIndex::ManualAcceleration);
	EXPECT_LT(horizontalDistance(setpoint, Vector2f{held.position[0], held.position[1]}), kOnVehicle);
}
