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
import cv2
from skimage import measure
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()


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
        rospy.Subscriber("/swift/camera_rgb/image_raw",
                         Image, self.image_callback)
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

    def image_callback(self, img_msg):
        '''
        Purpose:
        Reads image from /swift/camera_rgb/image_raw and converts it to a openCV image.
        Performs image processing to detect the position of organisms if any

        Returns:
        None
        '''
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        # convert it to grayscale, and blur it
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

        # threshold the image to reveal light regions in the blurred image
        ret, thersholding = cv2.threshold(
            blurred_image, 200, 255, cv2.THRESH_BINARY)

        # perform a series of erosions and dilations to remove any small blobs of noise from the thresholded image
        kernel = np.ones((5, 5), np.uint8)
        morph = cv2.morphologyEx(thersholding, cv2.MORPH_CLOSE, kernel)
        morph = cv2.morphologyEx(morph, cv2.MORPH_OPEN, kernel)
        labeled = measure.label(morph)  # finding connected components
        # sizes of each connected component
        sizes = np.bincount(labeled.ravel())
        blob_sizes = sizes > 60  # ignoring small blobs
        blob_sizes[0] = 0  # ignoring background
        if (blob_sizes.sum() > 1):  # if 2 or more blobs are present, i.e., an alien is detected
            self.found_alien = True
            self.alien_type = "alien_"+chr(ord("a")+blob_sizes.sum()-2)
            mask = blob_sizes[labeled]  # constructing a binary mask
            # constructing mat object from binary mask
            mask = np.array(mask, dtype=np.uint8)*255
            # find the contours in the mask
            contours, hir = cv2.findContours(
                mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # finding the centroid of the centroids of all the blobs
            self.alien_xy.fill(0)
            for c in contours:
                (x, y), radius = cv2.minEnclosingCircle(c)
                self.alien_xy[0] += x
                self.alien_xy[1] += y
            self.alien_xy /= len(contours)
            # shifting origin to the center of the image
            self.alien_xy[0] -= image.shape[1]/2
            self.alien_xy[1] -= image.shape[0]/2
        else:
            self.found_alien = False

    def pid(self, setpoint):
        '''
        Purpose:
        Compute the pid offset values and publish to /drone_command
        Arguments:
        Setpoint, the coordinates that the drone should go to
        Returns:
        None
        '''
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
            self.change_in_error + self.Ki*self.sum_error

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


def discovered(alienLocations, location):
    """
    Purpose:
    To determine if a detected organism has already been discovered
    Arguments:
    alienLocations- ArrayList of all discovered alien locations
    location- Estimated location of detected organism
    Returns- Boolean. True if already discovered
    """
    for coord in alienLocations:
        # distance between discovered organism and detected organism
        dist = np.sqrt(np.sum(np.square(coord[:2]-location[:2])))
        if (dist < 5):
            rospy.loginfo(
                "Already Discovered Alien at coord:{0}".format(location[:2]))
            return True
    return False


if __name__ == '__main__':

    swift_drone = swift()
    height = 24  # normal height
    low_height = 30  # height while bringing organism to center of frame

    # Setpoints to cover to follow a path
    setPoints = np.array([[-7, -7, height], [7, -7, height], [7, -3.5, height], [-7, -3.5, height], [-7, 0, height],
                          [7, 0, height], [7, 3.5, height], [-7, 3.5, height], [-7, 7, height], [7, 7, height]], dtype=np.float64)
    i = 0   # indicator for which setpoint to aim for currently

    # publisher for astrobiolocation
    astro_bio_pub = rospy.Publisher(
        "/astrobiolocation", Biolocation, queue_size=1)
    biolocation = Biolocation()
    alienLocations = []
    r = rospy.Rate(33)

    while not rospy.is_shutdown():
        swift_drone.pid(setPoints[i])

        # if an alien has been detected such that it has not already been discovered
        if swift_drone.found_alien and not discovered(alienLocations, swift_drone.drone_position+swift_drone.alien_xy/100):

            # initial position when the organism was spotted
            x, y = swift_drone.drone_position[0], swift_drone.drone_position[1]

            # loop until organism is approximately centered, break if organism is lost
            while swift_drone.found_alien and not np.logical_and(swift_drone.alien_xy < 3, swift_drone.alien_xy > -3).all():
                rospy.loginfo("Moving towards alien")
                target = swift_drone.drone_position + swift_drone.alien_xy/100
                # coordinates to target to get closer to center
                target[2] = low_height
                if discovered(alienLocations, target):
                    break
                # fly towards estimated location
                fly_to_coord(swift_drone, r, target,
                             accuracy=1.8, zaccuracy=0.2)

            # if organism was not lost
            if swift_drone.found_alien:
                rospy.loginfo("Found Alien")
                pos = np.array(swift_drone.drone_position)

                biolocation.organism_type = swift_drone.alien_type
                biolocation.whycon_x = pos[0]
                biolocation.whycon_y = pos[1]
                biolocation.whycon_z = pos[2]
                astro_bio_pub.publish(biolocation)
                # position saved for future reference
                alienLocations.append(pos)

            # return to the path drone was following, from which it deviated to discover this organism
            fly_to_coord(swift_drone, r, (x, y, height),
                         noSleep=True, accuracy=0.5, zaccuracy=0.5)

        # if drone has come sufficintly close to setpoint, target the next setpoint
        elif np.logical_and(swift_drone.error < 0.6, swift_drone.error > -0.6).all():
            if i == 9:
                break
            i += 1
            swift_drone.pid_reset = True
        r.sleep()

    # Landing Procedure:
    fly_to_coord(swift_drone, r, (11, 11, 32), accuracy=0.5, zaccuracy=0.5)
    rospy.loginfo("done 30")
    fly_to_coord(swift_drone, r, (11, 11, 36), accuracy=0.07)
    rospy.loginfo("done 36")
    swift_drone.pid((11, 11, 37))
    swift_drone.disarm()
    time.sleep(1)  # to allow drone to come to rest, before recording stops
