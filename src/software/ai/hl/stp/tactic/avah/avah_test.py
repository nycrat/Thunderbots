import pytest
import software.python_bindings as tbots_cpp

from proto.import_all_protos import *
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.ball_enters_region import (
    BallAlwaysStaysInRegion,
    BallNeverEntersRegion,
)
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


@pytest.mark.parametrize(
    "blue_bot, ball_initial_pos",
    [
        (
            tbots_cpp.Point(-1, 0),
            tbots_cpp.Point(0, 0),
        ),
        (
            tbots_cpp.Point(-1, 0),
            tbots_cpp.Point(1, 0),
        ),
    ],
)
def test_avah_function(
    blue_bot,
    ball_initial_pos,
    simulated_test_runner,
):
    # Setup Robot
    def setup(*args):
        simulated_test_runner.simulator_proto_unix_io.send_proto(
            WorldState,
            create_world_state(
                blue_robot_locations=[blue_bot],
                yellow_robot_locations=[],
                ball_location=ball_initial_pos,
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

        # Setup no tactics
        params = AssignedTacticPlayControlParams()
        simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
            AssignedTacticPlayControlParams, params
        )
        simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
            AssignedTacticPlayControlParams, params
        )

    # Always Validation
    always_validation_sequence_set = [
        [
            BallAlwaysStaysInRegion(
                regions=[
                    tbots_cpp.Circle(ball_initial_pos, 0.1),
                ]
            ),
            BallNeverEntersRegion(
                regions=[
                    tbots_cpp.Circle(
                        ball_initial_pos + tbots_cpp.Vector(0.5, 0.5), 0.1
                    ),
                ]
            ),
        ]
    ]

    # Eventually Validation
    eventually_validation_sequence_set = [[]]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=2,
    )


if __name__ == "__main__":
    pytest_main(__file__)
