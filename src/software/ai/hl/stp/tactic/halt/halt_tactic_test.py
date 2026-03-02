import software.python_bindings as tbots_cpp

from proto.import_all_protos import (
    Angle,
    AngularVelocity,
    AssignedTacticPlayControlParams,
    HaltTactic,
    Point,
    RobotState,
    Vector,
)
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.pytest_validations.robot_speed_threshold import (
    RobotSpeedEventuallyBelowThreshold,
)
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


def test_robot_already_stopped(simulated_test_runner):
    def setup(*args):
        world_state = create_world_state(
            blue_robot_locations=[tbots_cpp.Point(-3, 2.5), tbots_cpp.Point(0, 0)],
            yellow_robot_locations=[tbots_cpp.Point(4, 0)],
            ball_location=tbots_cpp.Point(0, 0.5),
            ball_velocity=tbots_cpp.Vector(0, 0),
        )

        simulated_test_runner.set_world_state(world_state)

        params = AssignedTacticPlayControlParams()
        params.assigned_tactics[1].halt.CopyFrom(HaltTactic())
        simulated_test_runner.set_tactics(params, is_friendly=True)

        params = AssignedTacticPlayControlParams()
        simulated_test_runner.set_tactics(params, is_friendly=False)

    eventually_validation_sequence_set = [
        [
            RobotSpeedEventuallyBelowThreshold(speed_threshold=0.05),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=5,
    )


def test_robot_start_moving(simulated_test_runner):
    def setup(*args):
        world_state = create_world_state(
            blue_robot_locations=[tbots_cpp.Point(-3, 2.5), tbots_cpp.Point(0, 0)],
            yellow_robot_locations=[tbots_cpp.Point(4, 0)],
            ball_location=tbots_cpp.Point(0, 0.5),
            ball_velocity=tbots_cpp.Vector(0, 0),
        )

        # TODO (#2588): create better way of specifying this
        world_state.blue_robots[1].CopyFrom(
            RobotState(
                global_position=Point(x_meters=0, y_meters=0),
                global_velocity=Vector(x_component_meters=4, y_component_meters=4),
                global_orientation=Angle(radians=0),
                global_angular_velocity=AngularVelocity(radians_per_second=0),
            )
        )

        simulated_test_runner.set_world_state(world_state)

        params = AssignedTacticPlayControlParams()
        params.assigned_tactics[1].halt.CopyFrom(HaltTactic())
        simulated_test_runner.set_tactics(params, is_friendly=True)

        params = AssignedTacticPlayControlParams()
        simulated_test_runner.set_tactics(params, is_friendly=False)

    eventually_validation_sequence_set = [
        [
            RobotSpeedEventuallyBelowThreshold(speed_threshold=0.05),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=6,
    )


if __name__ == "__main__":
    pytest_main(__file__)
