import numpy as np

from pd_controller_pkg.core import OuterLoopPositionController


def make_controller():
    n = 12
    return OuterLoopPositionController(
        n,
        kp=[2.0] * n,
        kd=[0.1] * n,
        ki=[0.5] * n,
        max_velocity=[1.0] * n,
        max_acceleration=[10.0] * n,
        integral_error_limit=[0.25] * n,
        lower_limits=[-1.0] * n,
        upper_limits=[1.0] * n,
    )


def test_outer_pd_generates_bounded_position_step():
    controller = make_controller()
    controller.reset([0.0] * 12)
    result = controller.compute(
        reference_position=[0.2] * 12,
        reference_velocity=[0.0] * 12,
        current_position=[0.0] * 12,
        current_velocity=[0.0] * 12,
        dt=0.02,
        mode='outer_pd',
    )

    # Acceleration cap: 10 rad/s^2 * 0.02 s = 0.2 rad/s on first cycle.
    assert np.allclose(result.velocity_command, 0.2)
    assert np.allclose(result.position_command, 0.004)
    assert np.allclose(result.integral_error, 0.0)


def test_outer_pid_integral_is_bounded():
    controller = make_controller()
    controller.reset([0.0] * 12)
    for _ in range(1000):
        controller.compute(
            reference_position=[0.5] * 12,
            reference_velocity=[0.0] * 12,
            current_position=[0.0] * 12,
            current_velocity=[0.0] * 12,
            dt=0.02,
            mode='outer_pid',
        )

    assert np.all(np.abs(controller.integral_error) <= 0.25 + 1e-12)
    assert np.all(controller.output_position <= 1.0 + 1e-12)
