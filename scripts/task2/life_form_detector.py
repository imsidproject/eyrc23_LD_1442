#!/usr/bin/env python3

'''
# Team ID:          eYRC#LD#1442
# Theme:            Luminosity Drone
# Author List:      Rupankar Podder, Sidharth Kumar Priyadarshi, Kausar Kamal
# Filename:         position_hold.py
# Functions:        None
# Global variables: None


This python file runs a ROS-node of name drone_control which holds the position of Swift-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
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

        # This corresponds to your current position of drone. This value must be updated each time in your whycon callback
        # [x,y,z]
        self.drone_position = np.zeros(3, dtype=np.float64)
        self.image = None
        self.found_alien = False
        self.alien_xy = np.zeros(3, dtype=np.float64)
        self.alien_type = None
        # [x_setpoint, y_setpoint, z_setpoint]
        # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
        # self.setpoint = np.array([[0, 0, 23], [2, 0, 23], [2, 2, 23], [
        #     2, 2, 25], [-5, 2, 25], [-5, -3, 25], [-5, -3, 21], [7, -3, 21], [7, 0, 21], [0, 0, 19]], dtype=np.float64)
        # self.setpointIndex = 0

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

        # initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
        # after tuning and computing corresponding PID parameters, change the parameters

        # roll,pitch, throttle
        self.Kp = np.array([10, 12, 10.0], dtype=np.float64)
        self.Ki = np.array([0.003, 0.001, 0.1], dtype=np.float64)  # ..
        self.Kd = np.array([200, 200, 400], dtype=np.float64)  # ..

        # -----------------------Add other required variables for pid here ----------------------------------------------
        self.error = np.zeros(3, dtype=np.float64)  # x,y,z
        self.prev_error = np.zeros(3, dtype=np.float64)  # x,y,z
        self.sum_error = np.zeros(3, dtype=np.float64)  # x,y,z
        self.change_in_error = np.zeros(
            3, dtype=np.float64)  # derivative of x,y,z error

        self.max_values = np.array(
            [2000, 2000, 2000], dtype=np.int16)  # roll,pitch, throttle
        self.min_values = np.array(
            [1000, 1000, 1000], dtype=np.int16)  # roll,pitch, throttle

        # offset roll,pitch,throttle from pid calculation
        self.pid_out = np.zeros(3, dtype=np.float64)

        # Publishing /drone_command, /alt_error, /pitch_error, /roll_error
        self.command_pub = rospy.Publisher(
            '/drone_command', swift_msgs, queue_size=1)
        self.alt_error_pub = rospy.Publisher(
            "/alt_error", Float64, queue_size=1)
        self.pitch_error_pub = rospy.Publisher(
            "/pitch_error", Float64, queue_size=1)
        self.roll_error_pub = rospy.Publisher(
            "/roll_error", Float64, queue_size=1)

        # Subscribing to /whycon/poses
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
        # --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z
        # ---------------------------------------------------------------------------------------------------------------

    def image_callback(self, img_msg):
        # Try to convert the ROS Image message to a CV2 Image
        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
            self.image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # print(self.image.shape)
            # convert it to grayscale, and blur it
            gray_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
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
            if (blob_sizes.sum() > 1):
                self.found_alien = True
                self.alien_type = "alien_"+chr(ord("a")+blob_sizes.sum()-2)
                mask = blob_sizes[labeled]  # constructing a binary mask
                # constructing mat object from binary mask
                mask = np.array(mask, dtype=np.uint8)*255
                # find the contours in the mask
                contours, hir = cv2.findContours(
                    mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                self.alien_xy.fill(0)
                for c in contours:
                    (x, y), radius = cv2.minEnclosingCircle(c)
                    self.alien_xy[0] += x
                    self.alien_xy[1] += y
                self.alien_xy /= len(contours)
                self.alien_xy[0] -= self.image.shape[1]/2
                self.alien_xy[1] -= self.image.shape[0]/2
                # rospy.loginfo(str(self.alien_xy))
            else:
                self.found_alien = False
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def pid(self, setpoint):

        self.error = self.drone_position - setpoint
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

        self.command_pub.publish(self.cmd)
        self.alt_error_pub.publish(self.error[2])
        self.pitch_error_pub.publish(self.error[1])
        self.roll_error_pub.publish(self.error[0])


def fly_to_coord(swift_drone, r, setpoint, accuracy=0.2, noSleep=False):
    setpoint = np.array(setpoint, dtype=np.float64)
    while True:
        swift_drone.pid(setpoint)
        r.sleep()
        if np.logical_and(swift_drone.error < accuracy, swift_drone.error > -accuracy).all():
            break
    if noSleep:
        swift_drone.pid(setpoint)


if __name__ == '__main__':

    swift_drone = swift()
    coords = np.array([[-7, -7, 20], [7, -7, 20], [7, -3.5, 20], [-7, -3.5, 20], [-7, 0, 20],
                      [7, 0, 20], [7, 3.5, 20], [-7, 3.5, 20], [-7, 7, 20], [7, 7, 20]], dtype=np.float64)
    i = 0
    astro_bio_pub = rospy.Publisher(
        "/astrobiolocation", Biolocation, queue_size=1)
    biolocation = Biolocation()
    r = rospy.Rate(33)

    while not rospy.is_shutdown():
        swift_drone.pid(coords[i])

        if swift_drone.found_alien:
            # x, y = swift_drone.drone_position[0], swift_drone.drone_position[1]
            while swift_drone.found_alien and not np.logical_and(swift_drone.alien_xy < 5, swift_drone.alien_xy > -5).all():
                rospy.loginfo("Moving towards alien:")
                rospy.loginfo(swift_drone.drone_position +
                              swift_drone.alien_xy/100)
                fly_to_coord(swift_drone, r, swift_drone.drone_position +
                             swift_drone.alien_xy/100, noSleep=True)
            if swift_drone.found_alien:
                rospy.loginfo("Found Alien at: ")
                rospy.loginfo(swift_drone.drone_position)
                biolocation.organism_type = swift_drone.alien_type
                biolocation.whycon_x = swift_drone.drone_position[0]
                biolocation.whycon_y = swift_drone.drone_position[1]
                biolocation.whycon_z = swift_drone.drone_position[2]
                astro_bio_pub.publish(biolocation)
                break
            # if not swift_drone.found_alien:
            #     fly_to_coord(swift_drone,r,(x,y,20),noSleep=True)
        elif np.logical_and(swift_drone.error < 0.5, swift_drone.error > -0.5).all():
            if i == 9:
                break
            i += 1
        r.sleep()
    # swift_drone.pid(np.array((10, 10, 28), dtype=np.float64))
    fly_to_coord(swift_drone, r, (11, 11, 30), 0.5)
    rospy.loginfo("done 30")
    fly_to_coord(swift_drone, r, (11, 11, 36), 0.1)
    rospy.loginfo("done 36")
    swift_drone.disarm()
