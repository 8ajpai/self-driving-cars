import os
import math
from dataclasses import dataclass
from sdc_course.control.pid import PIDLateralController, PIDLongitudinalController
from sdc_course.control.purepursuit import PurePursuitLateralController
from sdc_course.control.stanley import StanleyLateralController
from sdc_course.control.mpc import ModelPredictiveController
from sdc_course.utils.car import Car
from sdc_course.utils.utility import load_params


class dummy_xyz:
    x = 24.2
    y = 14.6
    z = -13.7


class dummy_yawpitchroll:
    yaw = 12.0
    pitch = 0.3
    roll = 1.2


class dummy_vel:
    x = 7.4
    y = 10.5
    z = 0.5


class dummy_control:
    steer = -2.2
    throttle = 3.4
    brake = 2.1


class dummy_transform:
    location = dummy_xyz()
    rotation = dummy_yawpitchroll()


class dummy_car:
    def get_transform(self):
        return dummy_transform()

    def get_velocity(self):
        return dummy_vel()

    def get_control(self):
        return dummy_control()


def test_task_1():
    print("\nEvaluating task 1:")
    exist = os.path.exists("results/recorded_trajectory.txt")
    if exist:
        print("-> Task 1 passed!\n")
    else:
        print("-> Task 1 not passed!\n")
    assert exist


def test_task_2():
    print("Evaluating task 2:")
    vehicle = dummy_car()
    pid_long = PIDLongitudinalController(vehicle, 0.11, 10.4, 20.8, 10.7)
    pid_long._error_buffer.append(6.4)
    pid_long._error_buffer.append(5.9)
    pid_long._error_buffer.append(5.2)
    acceleration = pid_long.run_step(18)
    passed_longitudinal = math.isclose(acceleration, 69.6911156016661, rel_tol=1e-4)
    if not passed_longitudinal:
        print(
            "Your computed acceleration with PID is",
            acceleration,
            "which differs from the one we computed for these dummy values.",
        )
    pid_lat = PIDLateralController(vehicle, 0.95, 1.5, 2.8, 0.7)
    waypoints = [[25.1, 14.1, -13.2]]
    pid_lat._error_buffer.append(4.4)
    pid_lat._error_buffer.append(-2.9)
    pid_lat._error_buffer.append(3.2)
    steering = pid_lat.run_step(waypoints)
    passed_lateral = math.isclose(steering, -11.56958438813039, rel_tol=1e-4)
    if not passed_lateral:
        print(
            "Your computed steering with PID is",
            steering,
            "which differs from the one we computed for these dummy values.",
        )
    passed = passed_longitudinal and passed_lateral
    if passed:
        print("-> Task 2 passed!\n")
    else:
        print("-> Task 2 not passed!\n")
    assert passed


def test_task_3():
    print("Evaluating task 3:")
    vehicle = dummy_car()
    waypoints = [[20.0, 10.4, -12.0], [25.1, 14.1, -13.2], [29.4, 18.9, -13.0]]

    purepursuit = PurePursuitLateralController(vehicle, 9.3, 3.4, 39.2)
    steering = purepursuit.run_step(waypoints)
    passed_pp = math.isclose(steering, 0.01709177472245794, rel_tol=1e-4)
    if steering != 0.0 and not passed_pp:
        print(
            "Your computed steering with Pure Pursuit is",
            steering,
            "which differs from the one we computed for these dummy values.",
        )

    stanley = StanleyLateralController(vehicle, 20.2)
    steering = stanley.run_step(waypoints)
    passed_stanley = math.isclose(steering, -0.386304237978627, rel_tol=1e-4)
    if steering != 0.0 and not passed_stanley:
        print(
            "Your computed steering with Stanley is",
            steering,
            "which differs from the one we computed for these dummy values.",
        )
    passed = passed_stanley or passed_pp
    if passed:
        print(" Task 3 passed!\n")
    else:
        print(" Task 3 not passed!\n")
    assert passed


def test_task_4():
    print("Evaluating task 4:")
    vehicle = dummy_car()
    mpc = ModelPredictiveController(vehicle, load_params("./params.yaml"))
    waypoints = [
        [24.3, 14.7, -13.6],
        [24.6, 14.8, -13.2],
        [24.9, 14.9, -13.0],
        [25.1, 15.0, -12.0],
    ]
    control = mpc.compute_control(15.3, waypoints)
    throttle = control.throttle
    brake = control.brake
    steering = control.steer
    passed = (
        math.isclose(throttle, 0.0, rel_tol=1e-4)
        and math.isclose(steering, -0.4767518639564514, rel_tol=1e-4)
        and math.isclose(brake, 0.30000001192092896, rel_tol=1e-4)
    )
    if passed:
        print(" Task 4 passed!\n")
    else:
        print(
            " Task 4 not passed! Is MPC implemented? If yes, your code does not gives the same result as ours using some dummy values. \n"
        )
    assert passed
