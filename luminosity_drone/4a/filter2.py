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
from geometry_msgs.msg import PoseArray, Pose
from pid_msg.msg import PidTune
from swift_msgs.msg import PIDError, RCMessage
from swift_msgs.srv import CommandBool




class DroneController():
    def __init__(self,node):
        self.node= node
        self.drone_position= np.zeros(3,dtype=np.float64)
        self.filtered_Coord = Pose()
        
        order = 6
        fs = 60 #sampling freq
        fc = 12 #critical freq
        nyq = 0.5 * fs
        wc = fc / nyq
        self.sos = scipy.signal.butter(N=order, Wn=3, btype='lowpass', analog=False, output='sos',fs=fs)
        self.DRONE_WHYCON_POSE=[[],[],[]]
        self.whycon_sub = node.create_subscription(PoseArray,"/whycon/poses",self.whycon_poses_callback,1)
        self.filtered_pub = node.create_publisher(Pose, "/whycon/filtered_pose2",1)
        


    def whycon_poses_callback(self, msg):
        #self.last_whycon_pose_received_at = self.node.get_clock().now().seconds_nanoseconds()[0]
        #self.whycon_pose_new = True
        # self.drone_whycon_pose_array = msg
        
        self.drone_position[0]= msg.poses[0].position.x
        self.drone_position[1]= msg.poses[0].position.y
        self.drone_position[2]= msg.poses[0].position.z
        #self.node.get_logger().info(str(self.drone_position))
        #print("start")
        self.whycon_filter()
        #print("stop")
    def whycon_filter(self):
        span = 30
        
        for index, val in enumerate(self.drone_position):
            self.DRONE_WHYCON_POSE[index].append(val)
            if len(self.DRONE_WHYCON_POSE[index]) == span:
                self.DRONE_WHYCON_POSE[index].pop(0)
            if len(self.DRONE_WHYCON_POSE[index]) != span-1:
                return
            filtered_signal = scipy.signal.sosfilt(self.sos, self.DRONE_WHYCON_POSE[index])
            if index == 0:
                self.filtered_Coord.position.x= filtered_signal[-1]
            elif index == 1:
                self.filtered_Coord.position.y = filtered_signal[-1]
            elif index == 2:
                self.filtered_Coord.position.z = filtered_signal[-1]
            
        # self.filtered_Coord.position.x = 1.0
        # self.filtered_Coord.position.y = 2.0
        # self.filtered_Coord.position.z = 3.0
        self.filtered_pub.publish(self.filtered_Coord)


    


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('filter2')
    controller= DroneController(node)
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
    main()