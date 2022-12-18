import math
import numpy as np
from sdc_course.utils.utility import *


class StanleyLateralController:
    """
    StanleyLateralController implements lateral control using the stanley controller.
    """

    def __init__(self, vehicle, K_cte):
        self._vehicle = vehicle
        self._k_cte = K_cte

    def run_step(self, waypoints):
        return self._stanley_control(waypoints, self._vehicle.get_transform())

    def _get_heading_error(self, waypoints, ind_nearest, vehicle_yaw):
        waypoint_delta_x = waypoints[ind_nearest + 1][0] - waypoints[ind_nearest][0]
        waypoint_delta_y = waypoints[ind_nearest + 1][1] - waypoints[ind_nearest][1]
        waypoint_heading = np.arctan2(waypoint_delta_y, waypoint_delta_x)
        heading_error = ((waypoint_heading - vehicle_yaw) + np.pi) % (2 * np.pi) - np.pi
        return heading_error

    def _get_steering_direction(self, v1, v2):
        """
        Note that Carla uses a left hand coordinate system, this is why a positive
        cross product requires a negative steering direction.
        :param v1: vector between vehicle and waypoint
        :param v2: vector in direction of vehicle
        :return: steering direction
        """
        cross_prod = v1[0] * v2[1] - v1[1] * v2[0]
        if cross_prod >= 0:
            return -1
        return 1

    def _get_cte_heading_error(self, vel, nearest_waypoint):
        nearest_distance = compute_distance_to_waypoint(self._vehicle, nearest_waypoint)
        proportional_cte_error = self._k_cte * nearest_distance
        cte_heading_error = np.arctan2(proportional_cte_error, vel)
        cte_heading_error = (cte_heading_error + np.pi) % (2 * np.pi) - np.pi
        return cte_heading_error

    def _stanley_control(self, waypoints, vehicle_transform):
        """
        :param waypoint: list of waypoints
        :param vehicle_transform: current transform of the vehicle
        :return: steering control
        """
        steering = 0.0
        #######################################################################
        ################## TODO: IMPLEMENT STANLEY CONTROL HERE ###############
        ###############################################################t########
        # x_wp = waypoints[0][0]
        # y_wp = waypoints[0][1]
        # x_car = vehicle_transform.location.x
        # y_car = vehicle_transform.location.y
        # # vel_car_x = self._vehicle.get_velocity().x
        # # vel_car_y = self._vehicle.get_velocity().y
        # # vel_car_z = self._vehicle.get_velocity().z
        # vel_car = get_velocity_ms(self._vehicle)
        # next_waypoint = get_nearest_waypoint(self._vehicle,waypoints)
        # print(f"type of wp : {next_waypoint}")
        # steering = math.atan(self._k_cte/vel_car)*StanleyLateralController._get_cte_heading_error(self,vel_car,tuple(next_waypoint))
    
        waypoint, waypoint_index = get_nearest_waypoint(self._vehicle,waypoints)
        heading_error = self._get_heading_error(waypoints, waypoint_index, vehicle_transform.rotation.yaw*np.pi/180 )
        cte_heading_error = self._get_cte_heading_error(get_velocity_ms(self._vehicle), tuple(waypoint))
        v1 = [vehicle_transform.location.x - waypoint[0] , vehicle_transform.location.y - waypoint[1]]
        v2 = [vehicle_transform.get_forward_vector().x,vehicle_transform.get_forward_vector().y]
        steering_direction = self._get_steering_direction(v1, v2)       
        steering = steering_direction * cte_heading_error + heading_error

        return steering
