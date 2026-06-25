#include "software/embedded/primitive_executor.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "proto/message_translation/tbots_geometry.h"
#include "proto/primitive.pb.h"
#include "shared/robot_constants.h"
#include "software/physics/velocity_conversion_util.h"
#include "software/world/robot_state.h"

// These tests exercise the forward-only (non-holonomic) motion behavior of the primitive
// executor, gated by PrimitiveExecutor::ENABLE_FORWARD_ONLY_MOTION. When that flag is
// enabled, the robot must never command a sideways (local-y / strafe) velocity: it should
// only ever drive forwards/backwards along its heading and rotate. While travelling it
// faces its direction of travel; once near the destination it rotates to the requested
// final orientation.

namespace
{
// Build a Move primitive proto that drives from start -> destination, rotating from
// start_angle -> final_angle, starting from rest.
TbotsProto::Primitive createMovePrimitive(const Point& start, const Point& destination,
                                          const Angle& start_angle,
                                          const Angle& final_angle)
{
    TbotsProto::Primitive primitive;
    auto* move = primitive.mutable_move();

    auto* xy                          = move->mutable_xy_traj_params();
    *(xy->mutable_start_position())   = *createPointProto(start);
    *(xy->mutable_destination())      = *createPointProto(destination);
    *(xy->mutable_initial_velocity()) = *createVectorProto(Vector(0, 0));
    xy->set_max_speed_mode(TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    auto* w                     = move->mutable_w_traj_params();
    *(w->mutable_start_angle()) = *createAngleProto(start_angle);
    *(w->mutable_final_angle()) = *createAngleProto(final_angle);
    *(w->mutable_initial_velocity()) =
        *createAngularVelocityProto(AngularVelocity::zero());

    return primitive;
}

// Pull the commanded local velocity out of the executor's direct control output.
Vector getCommandedLocalVelocity(const TbotsProto::DirectControlPrimitive& output)
{
    return createVector(output.motor_control().direct_velocity_control().velocity());
}

AngularVelocity getCommandedAngularVelocity(
    const TbotsProto::DirectControlPrimitive& output)
{
    return createAngularVelocity(
        output.motor_control().direct_velocity_control().angular_velocity());
}
}  // namespace

// The robot should never command a sideways velocity, even when the destination is
// directly to its side (which a holonomic robot would reach by strafing).
TEST(PrimitiveExecutorForwardOnlyTest, never_commands_sideways_velocity)
{
    if (!PrimitiveExecutor::ENABLE_FORWARD_ONLY_MOTION)
    {
        GTEST_SKIP() << "Forward-only motion is disabled";
    }

    const auto robot_constants = robot_constants::createRobotConstants();
    PrimitiveExecutor executor(robot_constants);

    // Robot at the origin facing +x, destination diagonally forward-left.
    const RobotState state(Point(0, 0), Vector(0, 0), Angle::zero(),
                           AngularVelocity::zero());
    executor.updateState(state);
    executor.updatePrimitive(
        createMovePrimitive(Point(0, 0), Point(1, 1), Angle::zero(), Angle::zero()));

    const Duration delta_time = Duration::fromSeconds(0.01);
    for (int i = 0; i < 100; ++i)
    {
        TbotsProto::PrimitiveExecutorStatus status;
        const auto output           = executor.stepPrimitive(status, delta_time);
        const Vector local_velocity = getCommandedLocalVelocity(*output);

        // No strafe: the local-y (sideways) velocity must always be zero.
        EXPECT_NEAR(local_velocity.y(), 0.0, 1e-9)
            << "Robot commanded a sideways velocity on step " << i;

        // The robot should be rotating left (positive angular velocity) to face its
        // ~45 degree travel direction.
        EXPECT_GT(getCommandedAngularVelocity(*output).toRadians(), 0.0)
            << "Robot should rotate toward its travel direction on step " << i;
    }
}

// With perfect command tracking, the robot should drive forwards to a destination ahead
// of it and then rotate in place to the requested final orientation, all without ever
// strafing.
TEST(PrimitiveExecutorForwardOnlyTest, reaches_destination_and_final_orientation)
{
    if (!PrimitiveExecutor::ENABLE_FORWARD_ONLY_MOTION)
    {
        GTEST_SKIP() << "Forward-only motion is disabled";
    }

    const auto robot_constants = robot_constants::createRobotConstants();
    PrimitiveExecutor executor(robot_constants);

    const Point destination(2, 0);
    const Angle final_angle = Angle::quarter();  // 90 degrees

    // Start at the origin facing +x (already aligned with the straight-ahead path).
    Point position               = Point(0, 0);
    Angle orientation            = Angle::zero();
    Vector global_velocity       = Vector(0, 0);
    AngularVelocity ang_velocity = AngularVelocity::zero();

    executor.updateState(
        RobotState(position, global_velocity, orientation, ang_velocity));
    executor.updatePrimitive(
        createMovePrimitive(position, destination, orientation, final_angle));

    const Duration delta_time = Duration::fromSeconds(0.005);
    double max_abs_strafe     = 0.0;
    for (int i = 0; i < 1000; ++i)
    {
        executor.updateState(
            RobotState(position, global_velocity, orientation, ang_velocity));

        TbotsProto::PrimitiveExecutorStatus status;
        const auto output           = executor.stepPrimitive(status, delta_time);
        const Vector local_velocity = getCommandedLocalVelocity(*output);
        ang_velocity                = getCommandedAngularVelocity(*output);

        max_abs_strafe = std::max(max_abs_strafe, std::abs(local_velocity.y()));

        // Integrate the commanded motion assuming the robot tracks it perfectly.
        global_velocity = localToGlobalVelocity(local_velocity, orientation);
        position        = position + global_velocity * delta_time.toSeconds();
        orientation     = orientation + ang_velocity * delta_time.toSeconds();
    }

    // Never strafed.
    EXPECT_NEAR(max_abs_strafe, 0.0, 1e-6);

    // Reached the destination.
    EXPECT_LT((position - destination).length(), 0.1)
        << "Robot ended at " << position << " instead of " << destination;

    // Settled at the requested final orientation.
    EXPECT_LT(orientation.minDiff(final_angle).toDegrees(), 5.0)
        << "Robot ended facing " << orientation << " instead of " << final_angle;
}

// Regression test for orientation oscillation. With realistic command/sensing latency,
// the heading controller must still settle on the target orientation rather than
// overshooting it and oscillating forever. (An undamped time-optimal deceleration profile
// passes the zero-latency tests above but limit-cycles here.)
TEST(PrimitiveExecutorForwardOnlyTest, settles_without_oscillation_under_latency)
{
    if (!PrimitiveExecutor::ENABLE_FORWARD_ONLY_MOTION)
    {
        GTEST_SKIP() << "Forward-only motion is disabled";
    }

    const auto robot_constants = robot_constants::createRobotConstants();
    PrimitiveExecutor executor(robot_constants);

    // Destination far in the +y direction, so the robot must rotate ~90 degrees from its
    // initial +x heading to face its travel direction, then drive towards it. The
    // destination is far enough that the robot never arrives during the test, so its
    // target orientation stays the (constant) +y travel direction throughout.
    const Point destination(0, 50);
    const Angle travel_direction = Angle::quarter();  // +y is 90 degrees

    Point position               = Point(0, 0);
    Angle orientation            = Angle::zero();
    Vector global_velocity       = Vector(0, 0);
    AngularVelocity ang_velocity = AngularVelocity::zero();

    executor.updatePrimitive(
        createMovePrimitive(position, destination, orientation, Angle::zero()));

    // Apply commanded velocities to the simulated robot a few steps late to emulate
    // command + sensing latency, which is what destabilizes an undamped controller.
    const int latency_steps = 12;  // 60ms at 5ms/step
    std::vector<Vector> commanded_local_velocity;
    std::vector<AngularVelocity> commanded_angular_velocity;

    const Duration delta_time = Duration::fromSeconds(0.005);
    const int num_steps       = 1000;
    double max_tail_error_deg = 0.0;
    for (int i = 0; i < num_steps; ++i)
    {
        executor.updateState(
            RobotState(position, global_velocity, orientation, ang_velocity));

        TbotsProto::PrimitiveExecutorStatus status;
        const auto output = executor.stepPrimitive(status, delta_time);
        commanded_local_velocity.push_back(getCommandedLocalVelocity(*output));
        commanded_angular_velocity.push_back(getCommandedAngularVelocity(*output));

        // Integrate the latency-delayed commands.
        const int applied = i - latency_steps;
        const Vector local_velocity =
            applied >= 0 ? commanded_local_velocity[applied] : Vector(0, 0);
        ang_velocity =
            applied >= 0 ? commanded_angular_velocity[applied] : AngularVelocity::zero();

        global_velocity = localToGlobalVelocity(local_velocity, orientation);
        position        = position + global_velocity * delta_time.toSeconds();
        orientation     = orientation + ang_velocity * delta_time.toSeconds();

        // After the robot has had ample time to settle (2s), it must stay locked on the
        // travel direction -- no sustained oscillation.
        if (i > 400)
        {
            max_tail_error_deg = std::max(
                max_tail_error_deg, orientation.minDiff(travel_direction).toDegrees());
        }
    }

    EXPECT_LT(max_tail_error_deg, 3.0)
        << "Orientation oscillates around its travel direction instead of settling";
}
