#!/bin/python3
from position_hold import swift
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import rospy
from geometry_msgs.msg import PoseArray
import time
from std_msgs.msg import Bool
from luminosity_drone.msg import AlienDetection, Biolocation,Coordinate
k=0
j=0
l=37
swift_drone=None
is_alien_meet=False
alien_type=0
centroid_x=-1
centroid_y=-1
is_center_of_frame =False
is_going_to_research_station=False
def center_of_frame(msg):
    global is_center_of_frame
    is_center_of_frame=msg.data

def alien_meet(msg):
    global is_alien_meet
    is_alien_meet=msg.data

def alien_detection(data):
    global alien_type, centroid_x, centroid_y
    alien_type=data.alien_type
    centroid_x=data.centroid_x
    centroid_y=data.centroid_y
def get_alien_type(type) -> str:
    if type ==2:
        return 'alien_a'
    elif type ==3:
        return "alien_b" 
    
        return "alien_c"
    
    
def get_next_position(msg):
    global j,k,l
    k=msg.x
    j=msg.y
    l=msg.z
    rospy.loginfo(f"coordinate Published {k}, {j}, ................................")

    swift.sets(swift_drone,[k,j,l])  
if __name__ == '__main__':
    swift_drone = swift()
    #publish data on this topic when alien comes into center of frame
    rospy.Subscriber('centerOfFrame',Bool,center_of_frame,queue_size=10)
    
    rospy.Subscriber('alien_data', AlienDetection,alien_detection,queue_size=10)
    rospy.Subscriber('nextCoordinate', Coordinate,get_next_position,queue_size=10)
    alien_data_pub=rospy.Publisher('astrobiolocation',Biolocation,queue_size=10)

    rospy.Subscriber('alienMeet', Bool,alien_meet,queue_size=10)
    is_other_pid_active_pub=rospy.Publisher('otherPidActive',Bool,queue_size=10)
    r = rospy.Rate(33)
    while not rospy.is_shutdown():
        if not is_going_to_research_station:
            if not is_center_of_frame:
                if not is_alien_meet:    
                    swift_drone.pid()
                    rospy.loginfo(f"stable")
                else:
                    #comment this when working on center_of_frame_pid()
                    swift_drone.pid()
                    rospy.loginfo(f"other pid at work")
                    #swift_drone.center_of_frame_pid(swift_drone,centroid_x,centroid_y) 
                    #is_other_pid_active_pub.publish(True)         
            else:
                swift_drone.center_of_frame_pid(swift_drone,centroid_x,centroid_y) 
                alien_info=Biolocation()
                alien_info.deserialize_numpy=get_alien_type(alien_type)
                alien_info.whycon_x=drone_position[0]
                alien_info.whycon_y=drone_position[1]
                alien_info.whycon_z=drone_position[2]
                alien_data_pub.publish(alien_info)
                is_going_to_research_station=True
        else:
            swift.sets(swift_drone,[11,11,37]) 
            swift_drone.pid()
            rospy.loginfo(f"Going to research station") 
        r.sleep()
    rospy.spin()    
