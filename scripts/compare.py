#!/bin/python3
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import rospy
from geometry_msgs.msg import PoseArray
import time
from luminosity_drone.msg import AlienDetection, Biolocation,Coordinate
from std_msgs.msg import Bool
drone_position=[0,0,0]
k=0
j=0
l=37
count=0


def equals(cor1,cor2) ->bool:
    if cor1 ==cor2: return True 
    if count==0:
        if cor1[0]==int(cor2[0]) :
            if cor1[1]==int(cor2[1]) :
                if cor1[2]==int(cor2[2]) :
                    return True
    else:
        if cor1[0]==int(cor2[0]) :
            if cor1[1]==int(cor2[1]) :
                #if cor1[2]==int(cor2[2]) :
                    return True
                
def whycon_callback(msg):
        global drone_position
        drone_position[0] = msg.poses[0].position.x
        drone_position[1] = msg.poses[0].position.y
        drone_position[2] = msg.poses[0].position.z
def get_next_position(msg):
    global j,k,l
    k=msg.x
    j=msg.y
    l=msg.z
    rospy.loginfo(f"Initial coordinate Published {k}, {j}, ................................")

                
if __name__ == '__main__':
    rospy.init_node("compare_position")
    rospy.Subscriber('whycon/poses', PoseArray, whycon_callback)

    rospy.Subscriber('nextCoordinate', Coordinate,get_next_position,queue_size=10)
    
    is_arrived_pub=rospy.Publisher('isArrived',Bool,queue_size=10)
    r = rospy.Rate(33)
    while not rospy.is_shutdown():
        whyco=drone_position
            
        if equals([k,j,l],whyco):
            count+=1
            is_arrived_pub.publish(True)    
            rospy.loginfo(f"Arrived ............................................................................")
            
        else:
            is_arrived_pub.publish(False)
            rospy.loginfo(f"comparing: {whyco} +{k}, {j}, {l}")

            
        r.sleep()    
            
    rospy.spin()
                
            