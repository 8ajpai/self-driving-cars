from typing import Union
import math
import numpy as np
import carla

from pyilqr.pyilqr.costs import CompositeCost, QuadraticCost
from pyilqr.pyilqr.example_costs import SetpointTrackingCost, PolylineTrackingCost, Polyline
from pyilqr.pyilqr.example_dynamics import UnicycleDynamics
from pyilqr.pyilqr.ocp import OptimalControlProblem
from pyilqr.pyilqr.ilqr import ILQRSolver
from pyilqr.pyilqr.strategies import AbstractStrategy, OpenLoopStrategy, FunctionStrategy

from sdc_course.utils.car import Car
from sdc_course.utils.utility import *
from sdc_course.control.controller import AbstractController


class ModelPredictiveController(AbstractController):
    """
    Receding horizon controller using an iterative LQR solver. At each iteration, the nonlinear optimal control problem is solved with local linear-quadratic approximations.
    """

    def __init__(self, vehicle: Car, params):
        """
        Constructor method.
        :param vehicle: actor to apply to local planner logic onto
        :param params: All the controller related parameters
        """
        super().__init__(vehicle, params)
        self.dynamics = UnicycleDynamics(self.dt)
        self.prediction_horizon = 20
        self._initial_strategy = None
        self.reset_initial_strategy(self._initial_strategy)

    def reset_initial_strategy(self, initial_strategy: Union[AbstractStrategy, None]):
        if initial_strategy is None:
            initial_strategy = FunctionStrategy(lambda x, t: np.zeros(self.dynamics.dims[1]))
        object.__setattr__(self, "_initial_strategy", initial_strategy)

    def compute_steering_and_acceleration(self, target_velocity_ms, waypoints):
        """
        Computes steering and acceleration
        :param target_velocity_ms: desired vehicle velocity in m/s
        :param waypoints: local trajectory waypoints
        :return: control command for the vehicle.
        """
        
        steering = 0.0
        acceleration = 0.0
        #######################################################################
        ################## TODO: IMPLEMENT MPC CONTROL HERE ###################
        #######################################################################

        #1. Model dynamics --> use unicycle 
        # self.dt should define??
        self.dt = 0.2
        dyn = UnicycleDynamics(self.dt)
        #x_car = vehicle_transform.location.x
        #y_car = vehicle_transform.location.y
        #car_yaw = vehicle_transform.rotation.yaw
        #vel_car = get_velocity_ms(self._vehicle)

        #transform = self.vehicle.get_transform()
        print(Car.get_transform(self))
        """ ILQR  -- 
        # inputs: - Dynamic; UnicycleDynamics(self.dt)
                  - State; x0 -- extract vehicle??
                  - State_Cost -- SetpointTrackingCost(np.eye(4), x_target = np.array([1,1,0,0]) )
                  - Input_Cost -- ocp = QuadraticCost(dynamics, State_Cost, Input_Cost, horizon)
        # output: solver = ILQR(ocp) -- give x and u at time t
        # """
        # x0 = 
        
        #QuadraticCost
        #dynamics = self.dynamics
	    #simulation_horizon = 200
	    #prediction_horizon = 20
	    #x0 = np.array([0,0,0,0.5])
        return steering, acceleration
