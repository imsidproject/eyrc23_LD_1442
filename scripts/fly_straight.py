#!/usr/bin/env python3

'''
# Team ID:          eYRC#LD#1442
# Theme:            Luminosity Drone
# Author List:      Rupankar Podder, Sidharth Kumar Priyadarshi, Kausar Kamal
# Filename:         biota_detector.py
# Functions:        fly_to_coord, discovered, 
# Global variables: bridge


This python file runs a ROS-node of name drone_control which holds the position of Swift-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error              /swift/camera_rgb/image_raw
		/pitch_error
		/roll_error
        /astrobiolocation

					
								
'''
# Importing the required libraries

from swift_msgs.msg import *
from luminosity_drone.msg import Biolocation
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import rospy
import time
import numpy as np


class swift():
    """docstring for swift"""

    def __init__(self):

        # initializing ros node with name life_form_detector
        rospy.init_node('life_form_detector')

        # current position of drone. updated each time in whycon callback[x,y,z]
        self.drone_position = np.zeros(3, dtype=np.float64)
        # whether there is an organism detected in most recent frame.
        self.found_alien = False
        # pixel coordinates of organism in the last frame with an organism
        self.alien_xy = np.zeros(3, dtype=np.float64)
        self.alien_type = None

        # Declaring a cmd of message type swift_msgs and initializing values
        self.cmd = swift_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500

        # initial setting of Kp, Kd and ki for [roll, pitch, throttle].
        self.Kp = np.array([10, 12, 10.0], dtype=np.float64)
        self.Ki = np.array([0.003, 0.001, 0.1], dtype=np.float64)
        self.Kd = np.array([200, 200, 400], dtype=np.float64)

        # -----------------------Other required variables for pid----------------------------------------------
        self.error = np.zeros(3, dtype=np.float64)  # x,y,z
        self.prev_error = np.zeros(3, dtype=np.float64)  # x,y,z
        self.sum_error = np.zeros(3, dtype=np.float64)  # x,y,z
        self.change_in_error = np.zeros(
            3, dtype=np.float64)  # derivative of x,y,z error

        # used to reset change_in_error,sum_error every time setpoint changes
        self.pid_reset = False
        self.max_values = np.array(
            [2000, 2000, 2000], dtype=np.int16)  # roll,pitch, throttle
        self.min_values = np.array(
            [1000, 1000, 1000], dtype=np.int16)  # roll,pitch, throttle

        # offset roll,pitch,throttle from pid calculation
        self.pid_out = np.zeros(3, dtype=np.float64)

        # Creating publishers for /drone_command, /alt_error, /pitch_error, /roll_error
        self.command_pub = rospy.Publisher(
            '/drone_command', swift_msgs, queue_size=1)
        self.alt_error_pub = rospy.Publisher(
            "/alt_error", Float64, queue_size=1)
        self.pitch_error_pub = rospy.Publisher(
            "/pitch_error", Float64, queue_size=1)
        self.roll_error_pub = rospy.Publisher(
            "/roll_error", Float64, queue_size=1)

        # Subscribing to /whycon/poses and /swift/camera_rgb/image_raw
        rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)

        self.arm()  # ARMING THE DRONE

    # Disarming condition of the drone

    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    # Arming condition of the drone : Best practise is to disarm and then arm the drone.

    def arm(self):

        self.disarm()
        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.command_pub.publish(self.cmd)  # Publishing /drone_command
        rospy.sleep(1)

    # Whycon callback function
    # The function gets executed each time when /whycon node publishes /whycon/poses

    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    def pid(self, setpoint, vel=None):
        '''
        Purpose:
        Compute the pid offset values and publish to /drone_command
        Arguments:
        Setpoint, the coordinates that the drone should go to
        Returns:
        None
        '''
        if vel is None:
            vel = np.zeros(3, np.float64)
        self.error = self.drone_position - setpoint
        if self.pid_reset:
            # if pid_reset was triggered, set derivative to 0 for all axes, and integral to 0 for x,y only
            self.change_in_error.fill(0)
            self.sum_error[:2].fill(0)
            self.pid_reset = False
        else:
            self.change_in_error = self.error-self.prev_error

        # updating the integral of error
        self.sum_error += self.error
        # calculating pid
        self.pid_out = self.Kp*self.error + self.Kd * \
            (self.change_in_error - vel) + self.Ki*self.sum_error

        # Adding or subtracting the avg value 1500 to get value for drone command
        self.pid_out[0] = 1500 - self.pid_out[0]
        self.pid_out[1] = 1500 + self.pid_out[1]
        self.pid_out[2] += 1500
        # Note: pid_out[] now stores the value for drone command, not the offset value

        # Checking and setting the value for drone command to lie in required range [1000,2000]
        for i in range(3):
            if (self.pid_out[i] > self.max_values[i]):
                self.pid_out[i] = self.max_values[i]
            elif (self.pid_out[i] < self.min_values[i]):
                self.pid_out[i] = self.min_values[i]

        self.cmd.rcRoll = int(self.pid_out[0])
        self.cmd.rcPitch = int(self.pid_out[1])
        self.cmd.rcThrottle = int(self.pid_out[2])

        # Updating previous Error
        self.prev_error[:] = self.error[:]
        # publishing error along all 3 axes
        self.command_pub.publish(self.cmd)
        self.alt_error_pub.publish(self.error[2])
        self.pitch_error_pub.publish(self.error[1])
        self.roll_error_pub.publish(self.error[0])


def fly_to_coord(swift_drone, r, setpoint, accuracy=0.2, zaccuracy=0.2, noSleep=False):
    """
    Purpose:
    Flies the drone to the given setpoint, then returns
    Arguments:
    swift_drone- swift object
    r- rospy.Rate object, so that r.sleep() can be called
    setpoint- Coordinates to move to
    accuracy- Required accuracy in x,y axes
    zaccuracy- Required accuracy in z axis, separated for better adjustment
    noSleep- When True, Returns without calling r.sleep() at the end

    """
    setpoint = np.array(setpoint, dtype=np.float64)
    swift_drone.pid(setpoint)
    while True:
        r.sleep()
        swift_drone.pid(setpoint)  # call pid at intervals
        # if drone coordinate errors are within required accuracy in all 3 axes
        if np.logical_and(swift_drone.error[:2] < accuracy, swift_drone.error[:2] > -accuracy).all() and swift_drone.error[2] < zaccuracy and swift_drone.error[2] > -zaccuracy:
            break
    if not noSleep:
        r.sleep()
    swift_drone.pid_reset = True  # trigger pid reset, as setpoint is likely to change


if __name__ == '__main__':

    swift_drone = swift()
    r = rospy.Rate(33)
    fly_to_coord(swift_drone, r, [-7, -2, 20])
    vel = np.float64((1, 0, 0))
    setpoint = np.array((7, -2, 20), np.float64)
    setpt = np.zeros(3, np.float64)
    setpt[:] = setpoint[:]
    reached = False
    while not rospy.is_shutdown():
        setpt[0] = swift_drone.drone_position[0]
        v = abs(setpoint[0]-swift_drone.drone_position[0])
        if v > 0.5 and not reached:
            v = min(v, 4)
            v *= 0.05
            swift_drone.pid(setpt, vel*v)
        else:
            if not reached:
                reached = True
                swift_drone.pid_reset = True
            swift_drone.pid(setpoint)
        r.sleep()

    swift_drone.disarm()
    time.sleep(1)  # to allow drone to come to rest, before recording stops
