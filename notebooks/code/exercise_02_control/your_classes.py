import numpy as np


class Controller():
    def __init__(self):
        pass

    def angle_control_commands(self, dist, angle):
        # Return the angular velocity in order to control the Duckiebot so that it follows the lane.
        # Parameters:
        #     dist: distance from the center of the lane. Left is negative, right is positive.
        #     angle: angle from the lane direction, in rad. Left is negative, right is positive.
        # Outputs:
        #     omega: angular velocity, in rad/sec. Right is negative, left is positive.
        
        omega = 0. 
        
        #######
        #
        # MODIFY ANGULAR VELOCITY
        #
        # YOUR CODE HERE
        #
        #######
        
        return  omega

    def control_commands(self, dist, angle):
        # Return the linear and angular velocities in order to control the Duckiebot so that it follows the lane as fast as possible.
        # Parameters:
        #     dist: distance from the center of the lane. Left is negative, right is positive.
        #     angle: angle from the lane direction, in rad. Left is negative, right is positive.
        # Outputs:
        #     v: linear velocity.
        #     omega: angular velocity. Right is negative, left is positive.
        
        v = 0.5
        omega = 0. 
                
        #######
        #
        # MODIFY LINEAR AND ANGULAR VELOCITY
        #
        # YOUR CODE HERE
        #
        #######

        return  v, omega
