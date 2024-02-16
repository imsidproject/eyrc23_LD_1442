#!/usr/bin/env python3

"""
Controller for the drone
"""



# standard imports
import copy
import time


# third-party imports
import scipy.signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PidTune
from swift_msgs.msg import PIDError, RCMessage
from swift_msgs.srv import CommandBool

MAX_SUM_ERROR = 100


MAX_ROLL = 1600
BASE_ROLL = 1500
MIN_ROLL = 1450
SUM_ERROR_ROLL_LIMIT = 3000


# Similarly, create upper and lower limits, base value, and max sum error values for roll and pitch




PID_OUTPUT_VALUES = [[], [], []] # This will be used to store data for filtering purpose

class DroneController():
    def __init__(self,node):
        self.node= node
        
        self.rc_message = RCMessage()
 
        self.commandbool = CommandBool.Request()

        self.rc_message.rc_yaw = int(1500)

    #------------------------------------------------------------------------------------------------------------------------

    self.rc_pub = node.create_publisher(RCMessage, "/luminosity_drone/rc_command",1)
    def publish_data_to_rpi(self, roll, pitch, throttle):

        self.rc_message.rc_roll = int(roll)
        self.rc_message.rc_yaw = int(1500)
        self.rc_message.rc_throttle = int(throttle)
        self.rc_message.rc_pitch = int(pitch)
        self.rc_pub.publish(self.rc_message)
        

    def shutdown_hook(self):
        self.node.get_logger().info("Calling shutdown hook")
        self.disarm()

    # Function to arm the drone 

    def arm(self):
        self.node.get_logger().info("Calling arm service")
        service_endpoint = "/swift/cmd/arming"
        arming_service_client = self.node.create_client(CommandBool,service_endpoint)
        self.commandbool.value = True
        try:
            resp = arming_service_client.call(self.commandbool)
            return resp.success, resp.result
        except Exception as err:
            self.node.get_logger().info(err)

    # Function to disarm the drone 

    def disarm(self):
        self.node.get_logger().info("Calling disarm service")
        service_endpoint = "/swift/cmd/disarming"
        disarming_service_client = self.node.create_client(CommandBool,service_endpoint)
        disarming_service_client.wait_for_service(10.0)
        self.commandbool.value = False
        try:
            resp = disarming_service_client.call(self.commandbool)
            return resp.success, resp.result
        except Exception as err:
           self.node.get_logger().info(err)
        self.is_flying = False




def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('controller')
    node.get_logger().info(f"Node Started")
    node.get_logger().info("Entering PID controller loop")

    controller = DroneController(node)
    controller.arm()
    node.get_logger().info("Armed")

    time.sleep(10)

    controller.shutdown_hook()
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
