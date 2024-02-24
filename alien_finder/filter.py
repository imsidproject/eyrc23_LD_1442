# Team ID:          eYRC#LD#1442
# Theme:            Luminosity Drone
# Author List:      Rupankar Podder, Sidharth Kumar Priyadarshi, Kausar Kamal
# Filename:         alien_detection_node.py
# Functions:        image_callback, img_operation
# Global variables: infi
'''
This python file contains functions to perform filtering of whycon poses
Publisher:                  Subscriber:
/whycon/filtered_pose1      None
/whycon/filtered_pose2
'''


import scipy.signal
from geometry_msgs.msg import Pose
import rclpy
import numpy as np


class Filter():
    def __init__(self,node):
        '''
        Argument:
        node- used to publish filtered poses (for debugging)
        '''
        self.whycon_poses= [[],[],[]] #for filtering
        self.filter_ready= False
        # 2 filters used, filter1 is smoother, filter2 is faster
        self.filter1 = scipy.signal.butter(N=3, Wn=5, btype='lowpass', analog=False, output='sos',fs=60)
        self.filter2 = scipy.signal.butter(N=2, Wn=4, btype='lowpass', analog=False, output='sos',fs=30)

        self.span1 = 15 
        self.span2 = 10
        self.drone_position_f1 = np.array((0,0,30),dtype= np.float64)   # from filter 1
        self.drone_position_f2 = np.array((0,0,30),dtype= np.float64)   #from filter 2

        self.filtered_pub1 = node.create_publisher(Pose, "/whycon/filtered_pose1",1)
        self.filtered_pub2 = node.create_publisher(Pose, "/whycon/filtered_pose2",1)
    
    def add_to_sequence(self,x):
        '''
        Purpose:
        Add the current whycon position to the list, then perform filtering to update
        the 2 filtered positions
        Also, publish both filtered positions
        '''
        for index, val in enumerate(x):
            self.whycon_poses[index].append(val)
            if len(self.whycon_poses[index]) == self.span1:
                self.whycon_poses[index].pop(0)
            if len(self.whycon_poses[index]) != self.span1-1:
                continue
            self.filter_ready= True
            filtered_signal1 = scipy.signal.sosfilt(self.filter1, self.whycon_poses[index])
            filtered_signal2 = scipy.signal.sosfilt(self.filter2, self.whycon_poses[index][self.span1-self.span2 :])
            
            self.drone_position_f1[index] = filtered_signal1[-1]
            self.drone_position_f2[index] = filtered_signal2[-1]
        
        if self.filter_ready:
            #publish filtered whycons
            f1=Pose()
            f2=Pose()
            f1.position.x= self.drone_position_f1[0]
            f1.position.y= self.drone_position_f1[1]
            f1.position.z= self.drone_position_f1[2]
            f2.position.x= self.drone_position_f2[0]
            f2.position.y= self.drone_position_f2[1]
            f2.position.z= self.drone_position_f2[2]


            self.filtered_pub1.publish(f1)
            self.filtered_pub2.publish(f2)
    def filt1(self):
        if not self.filter_ready:
            return None
        return self.drone_position_f1
    def filt2(self):
        if not self.filter_ready:
            return None
        return self.drone_position_f2
    