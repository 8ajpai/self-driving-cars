import math
import numpy as np
from sdc_course.utils.utility import *


class PurePursuitLateralController:
    """
    PurePursuitLateralController implements lateral control using the pure pursuit controller.
    """

    def __init__(self, vehicle, L, ld, K_pp):
        self._vehicle = vehicle
        self._L = L
        self._ld = ld
        self._k_pp = K_pp

    def run_step(self, waypoints):
        return self._pure_pursuit_control(waypoints, self._vehicle.get_transform())

    def _get_goal_waypoint_index(self, vehicle, waypoints, lookahead_dist):
        for i in range(len(waypoints)):
            dist = compute_distance_to_waypoint(vehicle, waypoints[i])
            if dist >= lookahead_dist:
                #print(dist, max(0, i))
                return max(0, i)
        return len(waypoints) - 1

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

    def _pure_pursuit_control(self, waypoints, vehicle_transform):
        """
        :param waypoint: list of waypointsld = self._k_pp*vel_car = get_velocity_ms(self._vehicle)
        :param vehicle_transform: current transform of the vehicle
        :return: steering control
        """
        steering = 0.0
        #######################################################################
        ################## TODO: IMPLEMENT PURE-PURSUIT CONTROL HERE ##########
        #######################################################################
        
        

        x_car = vehicle_transform.location.x
        y_car = vehicle_transform.location.y
        car_yaw = vehicle_transform.rotation.yaw
        vel_car = get_velocity_ms(self._vehicle)
        #next_waypoint = get_nearest_waypoint(self._vehicle,waypoints)
        ld = self._k_pp*vel_car 
        #ld = self._ld
        next_waypoint = PurePursuitLateralController._get_goal_waypoint_index(self,self._vehicle,waypoints,ld)
        x_wp = waypoints[next_waypoint][0]
        y_wp = waypoints[next_waypoint][1]
        #print(x_wp,y_wp)
        v1 = [x_car - waypoints[next_waypoint][0], y_car - waypoints[next_waypoint][1] ]
        v2 = [vehicle_transform.get_forward_vector().x,vehicle_transform.get_forward_vector().y]
        print(v1, v2)
        #v_cross = np.cross(vehicle_transform.get_forward_vector().x, vehicle_transform.get_forward_vector().y) 
        
        v1_norm =np.linalg.norm(np.asarray(v1))
        v2_norm = np.linalg.norm(np.asarray(v2))
        alpha = np.arccos(np.dot(np.asarray(v1),np.asarray(v2))/np.dot(v1_norm,v2_norm))
        print(alpha)
        #cte = np.sqrt((y_wp-y_car)**2+(x_wp-x_car)**2) 
        cte = ld*np.sin(-alpha)
        kappa = (2 * cte)/ld**2
        steering =  np.arctan(kappa*self._L) * self._get_steering_direction(v1, v2)
        print(f" WP Index : {next_waypoint}, yaw : {car_yaw}, Delta : {steering}, Lookahead distance : {ld}")
        return steering
