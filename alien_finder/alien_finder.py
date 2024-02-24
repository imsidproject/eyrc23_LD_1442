#!/usr/bin/env python3




'''
# Team ID:          eYRC#LD#1442
# Theme:            Luminosity Drone
# Author List:      Rupankar Podder, Sidharth Kumar Priyadarshi, Kausar Kamal
# Filename:         alien_finder.py
# Functions:        start, alien_callback, whycon_poses_callback, arm, disarm, pos_pid
                    vel_pid, publish_data_to_rpi, timer_callback
# Global variables: arm, ht, search_path


This python file runs a ROS-node of name alien_finder which holds the position of Swift-Drone .
This node publishes and subsribes the following topics:

        PUBLICATIONS                        SUBSCRIPTIONS
        /swift/rc_command                   /whycon/poses
        /luminosity_drone/pid_error         /alien_info
        /luminosity_drone/pid_verror
        /astrobiolocation

                    
                                
'''


# standard imports
import copy
import time
import sys
import math
sys.path.append('~/colcon_ws/src/luminosity_drone/alien_finder/scripts')

# third-party imports
import scipy.signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import PoseArray , Pose
from pid_msg.msg import PidTune
from swift_msgs.msg import PIDError, RCMessage
from swift_msgs.srv import CommandBool
from filter import Filter
from loc_msg.msg import AlienInfo
from loc_msg.msg import Biolocation
import json_reader as json

arm= True #whether to arm the drone
ht = 23  #height to fly the drone at
#points to visit in order
search_path= ("E5","D5","C5","B5","A5","A4","B4","C4","D4","E4","E3","D3","C3","B3","A3","A2","B2","C2","D2","E2","E1","D1","C1","B1","A1")



class DroneController():
    def __init__(self,node):
        self.node= node
        self.rc_message = RCMessage()
        self.filt= Filter(node=self.node) #filter for whycon poses

        #initialising arming service 
        self.commandbool = CommandBool.Request()
        service_endpoint = "/swift/cmd/arming"
        if arm:
            self.arming_service_client = self.node.create_client(CommandBool,service_endpoint)
            while not self.arming_service_client.wait_for_service(timeout_sec=1.0):
               self.node.get_logger().info("Service not available")

        # [x,y,z]
        self.drone_position = np.array((0,0,30), dtype=np.float64)
        self.delta = 1  #number of frames since last whycon pose received

        #Position PID gains:
        # roll,pitch, throttle
        self.Kp = np.array([3, 3, 3], dtype=np.float64)
        self.Ki = np.array([0.01, 0.01, 0.01], dtype=np.float64)  
        self.Kd = np.array([30, 30, 25], dtype=np.float64)  

        self.error = np.zeros(3, dtype=np.float64)  # x,y,z
        self.prev_error = np.array((0,0,30),dtype= np.float64)  # x,y,z
        self.sum_error = np.zeros(3, dtype=np.float64)  # x,y,z
        self.change_in_error = np.zeros(
            3, dtype=np.float64)  # derivative of x,y,z error
        
        #Velocity PID gains:
        #roll, pitch, throttle
        self.Kvp = np.array([5, 5, 4], dtype=np.float64)
        self.Kvi = np.array([0.01, 0.01, 0.04], dtype=np.float64)
        self.Kvd = np.array([30, 30, 20], dtype=np.float64)

        self.vel_stpt = np.zeros(3, dtype=np.float64)
        self.vel = np.zeros(3, dtype=np.float64) #current velocity of drone
        self.vel_error = np.zeros(3, dtype=np.float64)
        self.change_in_vel_error = np.zeros(3, dtype=np.float64)
        self.sum_vel_error = np.zeros(3, dtype=np.float64)
        self.prev_vel_error = np.zeros(3, dtype=np.float64)
        self.prev_pos = np.zeros(3, dtype=np.float64) #used to calculate velocity


        self.setpt= np.float64(json.data[search_path[0]]) #current setpoint
        self.setpt[2] = ht
        self.setpt[1] +=1.3 #adjustment for distance between camera and whycon marker on drone
        self.accuracy =0.8
        self.count=0 #counter for control
        self.l_count=0 #counter for landing
        self.land= False
        self.stpti=0 #index of current setpoint among the search path
        self.led_count=0 #count of leds detected
        self.prev_led_count=0 #count of leds in last frame
        self.alien_x_px =0    #position of alien centroid
        self.alien_y_px =0    
        self.beep_count=-1    #counter to control beep and led
        self.return_2_base = False
        self.landingPt= np.float64([9.8,8.7,ht])
        self.pid_out = np.zeros(3, dtype=np.float64)

        self.max_values = np.array(
            [1800, 1800, 1700], dtype=np.int16)  # roll,pitch, throttle
        self.min_values = np.array(
            [1200, 1200, 1000], dtype=np.int16)  # roll,pitch, throttle
        self.base_values = np.array(
            [1500, 1500, 1400], dtype=np.int16)  # roll,pitch, throttle
        # offset roll,pitch,throttle from pid calculation

        # Create subscriber for WhyCon and publishers for rc_command ,PIDerror, PID Velocity error
        # Also subsribe to alien_info from alien_detection_node
        self.whycon_sub = node.create_subscription(PoseArray,"/whycon/poses",self.whycon_poses_callback,1)
        self.rc_pub = node.create_publisher(RCMessage, "/swift/rc_command",1)
        self.astro_pub = node.create_publisher(Biolocation, '/astrobiolocation',1)
        self.alien_info_sub = node.create_subscription(AlienInfo, "/alien_info", self.alien_callback,1)
        self.pid_error_pub = node.create_publisher(PIDError, "/luminosity_drone/pid_error",1)
        self.pid_verror_pub = node.create_publisher(PIDError, "/luminosity_drone/pid_verror",1)

        
    def start(self):
        '''
        Purpose:
        Arms the drone and starts flying by creating a timer that calls pid function regularly
        '''
        self.arm()
        self.timer = self.node.create_timer(0.0333, self.timer_callback) #callback to run pid every 33ms
    def alien_callback(self, msg):
        '''
        Purpose:
        Current alien position, type is received from alien_detection_node
        '''
        self.led_count = msg.alien_type
        if self.led_count>0:
            self.alien_x_px= msg.x
            self.alien_y_px= msg.y
    

    def whycon_poses_callback(self, msg):
        '''
        Purpose:
        Apply filters and set drone position value
        '''
        self.whycon_pose_new = True
        self.drone_position[0]= msg.poses[0].position.x
        self.drone_position[1]= msg.poses[0].position.y
        self.drone_position[2]= msg.poses[0].position.z
        self.filt.add_to_sequence(self.drone_position)
        try:
            #Set z axis to value from slower but smoother filter
            self.drone_position[2]= self.filt.filt1()[2]
            self.drone_position[:2]= self.filt.filt2()[:2]
        except:
            pass


    def arm(self):
        '''
        Purpose: Arms the drone, only if global variable arm is set
        '''
        self.node.get_logger().info("Calling arm service")

        if arm:
            self.commandbool.value = True
            self.future = self.arming_service_client.call_async(self.commandbool)
            rclpy.spin_until_future_complete(self.node,self.future)
            if self.future.result().success:
                self.node.get_logger().info("Arm Successful")
            else:
                self.node.get_logger().info("Arm Unsuccessful")

    def disarm(self):
        '''
        Purpose: Disarms the drone, only if global variable arm is set
        '''
        self.node.get_logger().info("Calling disarm service")

        if arm:
            self.commandbool.value = False
            self.future = self.arming_service_client.call_async(self.commandbool)
            rclpy.spin_until_future_complete(self.node,self.future)
            if self.future.result().success:
                self.node.get_logger().info("Disarm Successful")

    def pos_pid(self, setpoint):
        '''
        Purpose:
        Computes the velocity setpoint and calls vel_pid()
        Arguments:
        Position setpoint(NDArray)
        Returns:
        None
        '''
    
        self.error[:] = setpoint - self.drone_position
        self.change_in_error[:] = self.error-self.prev_error
        # if self.pid_reset:
        #     # if pid_reset was triggered, set derivative to 0 for all axes, and integral to 0 for x,y only
        #     self.change_in_error.fill(0)
        #     self.sum_error[:2].fill(0)
        #     self.pid_reset = False
        # else:
        #     self.change_in_error = self.error-self.prev_error

        # updating the integral of error
        self.sum_error[:] += self.error
        # calculating pid
        self.vel_stpt[:] = self.Kp*self.error + self.Kd * \
            (self.change_in_error - self.vel) + self.Ki*self.sum_error
        self.vel_pid(self.vel_stpt)

        # Updating previous Error
        self.prev_error[:] = self.error[:]
        # publishing error along all 3 axes
        self.pid_error_pub.publish(
            PIDError(
                roll_error=float(self.error[0]),
                pitch_error=float(self.error[1]),
                throttle_error=float(self.error[2]),
                yaw_error=-0.0,
                zero_error=0.0,
            )
        )
        
       
    def vel_pid(self, vel_setpoint):
        '''
        Purpose:
        Compute roll,pitch,throttle and call publish_data_to_rpi
        Argument:
        velocity setpoint(NDArray)
        '''

        self.vel[:] = self.drone_position - self.prev_pos
        self.prev_pos[:] = self.drone_position
        self.vel_error[:] = vel_setpoint-self.vel
        self.change_in_vel_error[:] = self.vel_error - self.prev_vel_error
        self.sum_vel_error[:] += self.vel_error
        self.prev_vel_error[:] = self.vel_error
        self.pid_out[:] = self.Kvp*self.vel_error + self.Kvd * \
            self.change_in_vel_error + self.Kvi*self.sum_vel_error
        # Adding or subtracting the base value to get value for drone command
        self.pid_out[0] = self.base_values[0] + self.pid_out[0]
        self.pid_out[1] = self.base_values[1] - self.pid_out[1]
        self.pid_out[2] = self.base_values[2] -self.pid_out[2]
        # Note: pid_out[] now stores the value for drone command, not the offset value

        # Checking and setting the value for drone command to lie in required range
        for i in range(3):
            if (self.pid_out[i] > self.max_values[i]):
                self.pid_out[i] = self.max_values[i]
            elif (self.pid_out[i] < self.min_values[i]):
                self.pid_out[i] = self.min_values[i]

        self.publish_data_to_rpi(self.pid_out[0],self.pid_out[1],self.pid_out[2])
        self.pid_verror_pub.publish(
            PIDError(
                roll_error=float(self.vel_error[0]),
                pitch_error=float(self.vel_error[1]),
                throttle_error=float(self.vel_error[2]),
                yaw_error=-0.0,
                zero_error=0.0,
            )
        )

    def publish_data_to_rpi(self, roll, pitch, throttle):
        '''
        Purpose:
        Publishes given roll,pitch, throttle values
        Also, lowers throttle values for landing
        Also, controls buzzer and leds using beep_counter
        Arguments:
        roll, pitch, throttle values
        '''
        self.rc_message.rc_throttle = int(throttle)
        self.rc_message.rc_roll = int(roll)
        self.rc_message.rc_pitch = int(pitch)
        #landing logic:
        if self.land:
            self.l_count+=1
            self.rc_message.rc_throttle = 1480
        if self.land and self.l_count>300:
            self.rc_message.rc_throttle = 1400
        if self.land and self.l_count>350:
            self.disarm()
        
        if self.beep_count>=0:
            if (math.ceil(self.beep_count/6))%2 ==1:
                self.rc_message.aux4= 1500
                self.rc_message.aux3= 2000
                print("Beep")
            else:
                self.rc_message.aux4=2000
                self.rc_message.aux3 = 1000
                print("NoBeep")
            self.beep_count-=1

        # Send constant 1500 to rc_message.rc_yaw
        self.rc_message.rc_yaw = int(1500)
        self.rc_pub.publish(self.rc_message)

    def follow_line(self,p,q,x):
        dv = q[:2]-p[:2]
        nv = np.float64((-dv[1], dv[0]))
        nmod = np.linalg.norm(nv)
        nhat = nv/nmod
        dhat = dv/nmod
        dist = np.linalg.norm(dv) - sum((x-p)[:2]*dhat)
        if dist<2:
            self.pos_pid(q)
        else:
            l = sum((p-x)[:2]*nhat[:])
            self.error[:2] = nhat*l
            self.error[2] = q[2]-x[2]
            
            self.change_in_error = self.error- self.prev_error
            self.sum_error[:] += self.error
            self.prev_error[:] = self.error[:]
            self.vel_stpt[:] = self.Kp*self.error + self.Kd * \
                self.change_in_error + self.Ki*self.sum_error
            speed= 0.5
            self.vel_stpt[0] += speed*dhat[0]
            self.vel_stpt[1] += speed*dhat[1]

            self.vel_pid(self.vel_stpt)
        return dist
    
    def center(self):
        target= np.float64(self.drone_position)
        target[2]=ht
        target[0] += self.alien_x_px/1000
        target[1] += self.alien_y_px/1000
        print(self.alien_x_px, self.alien_y_px)
        self.pos_pid(target)

    def timer_callback(self):
        '''
        Purpose:
        Run at 30hz, call pid function
        Also, record whether last received whycon is new/old and how old
        Returns:
        None
        '''
        
        if not self.whycon_pose_new:
            
            self.delta += 1
            if self.delta>10:
                self.node.get_logger().error("Unable to detect WHYCON poses")
                print(self.delta)
        else:
            self.whycon_pose_new= False
            self.delta = 1

        #returning , landing logic
        if self.return_2_base:
            self.pos_pid(self.landingPt)
            if np.bitwise_and(self.error<0.8 , self.error>-0.8).all():
                self.land = True
            return
        
        #fly towards current setpoint
        self.pos_pid(self.setpt)
        if np.bitwise_and(self.error<0.8 , self.error>-0.8).all() and (self.alien_x_px**2 + self.alien_y_px**2) < 320000 and self.led_count == self.prev_led_count:
            self.count +=1
        else:
            self.count=0
            self.prev_led_count=self.led_count
        #if alien detection was stable, and alien (if any) was centered for >10 frames
        if self.count>10:
            print("Alien Type: ", self.led_count, "x,y:" , self.drone_position[0],self.drone_position[1]-1.3)
            self.stpti +=1
            self.setpt= np.float64(json.data[search_path[self.stpti]])
            self.setpt[2] = ht
            self.setpt[1] +=1.3 #same adjustment as before
            self.count = 0
            self.prev_led_count = -1
            if self.led_count>1 and self.led_count<=5:
                #set the beep counter
                self.beep_count= 12*(self.led_count -1) + 6
                self.return_2_base= True
                #Publish biolocation
                B= Biolocation()
                B.organism_type= 'alien_'+chr(ord("a")+self.led_count-2)
                B.whycon_x = self.drone_position[0]
                B.whycon_y = self.drone_position[1]-1.3 #same adjustment as before
                B.whycon_z = 30.0
                self.astro_pub.publish(B)


def main():
    rclpy.init()
    node = rclpy.create_node('alien_finder')
    controller = DroneController(node)
    #Make the filter ready
    while not controller.filt.filter_ready:
        rclpy.spin_once(node)
    print("Starting")
    controller.start()
    try:
        
        rclpy.spin(node)
    finally:
        controller.disarm()
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
    