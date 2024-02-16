#!/usr/bin/env python3




'''
# Team ID:          eYRC#LD#1442
# Theme:            Luminosity Drone
# Author List:      Rupankar Podder, Sidharth Kumar Priyadarshi, Kausar Kamal
# Filename:         controller.py
# Functions:        None
# Global variables: arm


This python file runs a ROS-node of name controller which holds the position of Swift-Drone .
This node publishes and subsribes the following topics:

        PUBLICATIONS                        SUBSCRIPTIONS
        /swift/rc_command                   /whycon/poses
        /luminosity_drone/pid_error         /pid_tuning_altitude
                                            /pid_tuning_roll
        /whycon/filtered_pose1              /pid_tuning_pitch
        /whycon/filtered_pose2
        /swift/rc_unbound
                    
                                
'''


# standard imports
import copy
import time

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


arm= True #whether to arm the drone

class DroneController():
    def __init__(self,node):
        self.node= node

        self.rc_message = RCMessage()
        self.whycon_poses= [[],[],[]] #for filtering
        # 2 filters used, filter1 is smoother, filter2 is faster
        self.filter1 = scipy.signal.butter(N=3, Wn=5, btype='lowpass', analog=False, output='sos',fs=60)
        self.filter2 = scipy.signal.butter(N=2, Wn=4, btype='lowpass', analog=False, output='sos',fs=30)
        self.whycon_pose_new= False #whether the last whycon pose is old/new
        self.delta = 0  #number of frames since last whycon pose received
        #filter spans
        self.span1 = 15 
        self.span2 = 10
        self.filter_ready=False #False until full span is received
        
        #initialising arming service 
        self.commandbool = CommandBool.Request()
        service_endpoint = "/swift/cmd/arming"
        if arm:
            self.arming_service_client = self.node.create_client(CommandBool,service_endpoint)
            while not self.arming_service_client.wait_for_service(timeout_sec=1.0):
               self.node.get_logger().info("Service not available")

        # [x,y,z]
        self.drone_position = np.array((0,0,30), dtype=np.float64)
        self.drone_position_f1 = np.array((0,0,30),dtype= np.float64)   # from filter 1
        self.drone_position_f2 = np.array((0,0,30),dtype= np.float64)   #from filter 2
        
        # roll,pitch, throttle
        self.Kp = np.array([8, 8, 7.28], dtype=np.float64)
        self.Ki = np.array([0.1, 0.1, 0.20], dtype=np.float64)  
        self.Kd = np.array([250, 250, 180], dtype=np.float64)  

        self.setpoints= np.array([[0, 0, 23],[2, 3, 23],[-1, 2, 25],[-3, -3, 25],[0,0,23]], dtype=np.float64)
        self.setpt_index = 0

        self.error = np.zeros(3, dtype=np.float64)  # x,y,z
        self.prev_error = np.array((0,0,30),dtype= np.float64)  # x,y,z
        self.sum_error = np.zeros(3, dtype=np.float64)  # x,y,z
        self.change_in_error = np.zeros(
            3, dtype=np.float64)  # derivative of x,y,z error
        self.max_values = np.array(
            [1800, 1800, 1700], dtype=np.int16)  # roll,pitch, throttle
        self.min_values = np.array(
            [1200, 1200, 1000], dtype=np.int16)  # roll,pitch, throttle
        self.base_values = np.array(
            [1500, 1500, 1400], dtype=np.int16)  # roll,pitch, throttle
        # offset roll,pitch,throttle from pid calculation
        self.pid_out = np.zeros(3, dtype=np.float64)
        self.pid_reset = False

        self.count=0
        self.land= False
        
        # Create subscriber for WhyCon , pidtune and publishers for rc_command ,PIDerror
        # Also some additional publishers for debugging/tuning
        self.whycon_sub = node.create_subscription(PoseArray,"/whycon/poses",self.whycon_poses_callback,1)
        self.rc_pub = node.create_publisher(RCMessage, "/swift/rc_command",1)
        self.rc_unbound_pub = node.create_publisher(Pose, "/swift/rc_unbound",1)

        self.pid_alt = node.create_subscription(PidTune,"/pid_tuning_altitude",self.pid_tune_throttle_callback,1)
        self.pid_roll = node.create_subscription(PidTune,"/pid_tuning_roll",self.pid_tune_roll_callback,1)
        self.pid_pitch = node.create_subscription(PidTune,"/pid_tuning_pitch",self.pid_tune_pitch_callback,1)
        
        self.pid_error_pub = node.create_publisher(PIDError, "/luminosity_drone/pid_error",1)
        self.filtered_pub1 = node.create_publisher(Pose, "/whycon/filtered_pose1",1)
        self.filtered_pub2 = node.create_publisher(Pose, "/whycon/filtered_pose2",1)
        
        
        self.arm()
        self.timer = node.create_timer(0.0333, self.timer_callback) #callback to run pid every 33ms

    def whycon_poses_callback(self, msg):
        '''
        Purpose:
        Set drone_position[] value, apply both kinds of filter, and update value 
        on drone_position_f1[] and drone_position_f2[]
        '''
        self.whycon_pose_new = True
        self.drone_position[0]= msg.poses[0].position.x
        self.drone_position[1]= msg.poses[0].position.y
        self.drone_position[2]= msg.poses[0].position.z
        

        for index, val in enumerate(self.drone_position):
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
        
            
            

    def pid_tune_throttle_callback(self, msg):
        self.Kp[2] = msg.kp * 0.01
        self.Ki[2] = msg.ki * 0.0001
        self.Kd[2] = msg.kd * 0.1

    def pid_tune_pitch_callback(self, msg):
        self.Kp[1] = msg.kp * 0.01
        self.Ki[1] = msg.ki * 0.0001
        self.Kd[1] = msg.kd * 0.1

    def pid_tune_roll_callback(self, msg):
        self.Kp[0] = msg.kp * 0.01
        self.Ki[0] = msg.ki * 0.0001
        self.Kd[0] = msg.kd * 0.1

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

    def pid(self, setpoint):
        '''
        Purpose:
        Compute the pid offset values and publish to /rc_command by calling publish_data_to_rpi
        Arguments:
        Setpoint, the coordinates that the drone should go to
        Returns:
        None
        '''

        #using smoother filter f1 for z, faster filter f2 for x,y
        self.error = self.drone_position_f1 - setpoint
        self.error[:2]= self.drone_position_f2[:2] - setpoint[:2]
        self.change_in_error = self.drone_position_f1- setpoint -self.prev_error
        self.change_in_error[:2] = self.drone_position_f2[:2]- setpoint[:2] -self.prev_error[:2]
        self.prev_error[:] = self.drone_position_f1- setpoint
        self.prev_error[:2] = self.drone_position_f2[:2]- setpoint[:2]

        
        if self.pid_reset:
            # if pid_reset was triggered, set derivative to 0 for all axes, and integral to 0 for x,y only
            self.change_in_error.fill(0)
            self.pid_reset = False
        
        # updating the integral of error
        self.sum_error += self.error
        # calculating pid
        self.pid_out = self.Kp*self.error + self.Kd * \
            self.change_in_error + self.Ki*self.sum_error

        # Adding or subtracting the avg value 1500 to get value for drone command
        self.pid_out[0] = self.base_values[0] - self.pid_out[0]
        self.pid_out[1] = self.base_values[1] + self.pid_out[1]
        self.pid_out[2] += self.base_values[2]
        # Note: pid_out[] now stores the value for drone command, not the offset value

        p= Pose()
        p.position.x = self.pid_out[0]
        p.position.y = self.pid_out[1]
        p.position.z = self.pid_out[2]
        self.rc_unbound_pub.publish(p)

        # Checking and setting the value for drone command to lie in required range [1000,2000]
        for i in range(3):
            if (self.pid_out[i] > self.max_values[i]):
                self.node.get_logger().info("max"+str(i))
                self.pid_out[i] = self.max_values[i]
            elif (self.pid_out[i] < self.min_values[i]):
                self.pid_out[i] = self.min_values[i]

        self.publish_data_to_rpi(self.pid_out[0],self.pid_out[1],self.pid_out[2])
        

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
    def publish_data_to_rpi(self, roll, pitch, throttle):
        '''
        Purpose:
        Publishes given roll,pitch, throttle values
        Also, lowers throttle values for landing
        '''
        self.rc_message.rc_throttle = int(throttle)
        self.rc_message.rc_roll = int(roll)
        self.rc_message.rc_pitch = int(pitch)
        #landing logic:
        if self.land:
            self.count+=1
            self.rc_message.rc_throttle = 1480
        if self.land and self.count>300:
            self.rc_message.rc_throttle = 1400
        if self.land and self.count>350:
            self.disarm()
        
        # Send constant 1500 to rc_message.rc_yaw
        self.rc_message.rc_yaw = int(1500)
        self.rc_pub.publish(self.rc_message)

    def timer_callback(self):
        '''
        Purpose:
        Run at 30hz, call pid function
        Also, record whether last received whycon is new/old and how old
        Returns:
        None
        '''
        if not self.whycon_pose_new:
            self.node.get_logger().error("Unable to detect WHYCON poses")
            self.delta += 1
        else:
            self.whycon_pose_new= False
            self.delta = 1
        if self.filter_ready:
            d= self.setpoints[self.setpt_index]- self.drone_position
            
            if( self.setpt_index<=3 and np.logical_and(d<0.8 , d>-0.8).all()):
                self.setpt_index+=1
                self.pid_reset=True
            if self.setpt_index==4 and np.logical_and(d<0.8 , d>-0.8).all():
                self.land = 1
            self.pid(self.setpoints[self.setpt_index])
            print(self.setpoints[self.setpt_index],self.drone_position)

def main():
    rclpy.init()
    node = rclpy.create_node('waypoint_controller')
    controller = DroneController(node)
    print("Starting")
    try:
        rclpy.spin(node)
    finally:
        controller.disarm()
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
    