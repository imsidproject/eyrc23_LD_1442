#!/bin/python3
from position_hold import swift
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import rospy
from geometry_msgs.msg import PoseArray
import time
from luminosity_drone.msg import AlienDetection, Biolocation



drone_position=[0,0,0]
alien_type=0
centroid_x=-1
centroid_y=-1
frame=None

def alien_detection(data):
    global alien_type, centroid_x, centroid_y
    alien_type=data.alien_type
    centroid_x=data.centroid_x
    centroid_y=data.centroid_y
    rospy.loginfo(f"alien data recieved :-{str(alien_type)}, {str(centroid_x)}, {str(centroid_y)}]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]")





    
    
def equals(cor1,cor2) ->bool:
    if cor1 ==cor2: return True 
    if cor1[0]==int(cor2[0]) or cor1[0]==int(cor2[0]+1):
         if cor1[1]==int(cor2[1]) or cor1[1]==int(cor2[1]+1):
             if cor1[2]==int(cor2[2]) or cor1[2]==int(cor2[2]+1):
                return True
    return False     
def whycon_callback(msg):
        global drone_position
        drone_position[0] = msg.poses[0].position.x
        drone_position[1] = msg.poses[0].position.y
        drone_position[2] = msg.poses[0].position.z
def get_alien_type(type) -> str:
    if type ==2:
        return 'alien_a'
    elif type ==3:
        return "alien_b" 
    
        return "alien_c"                  
    
if __name__ == '__main__':
    
    rospy.Subscriber('whycon/poses', PoseArray, whycon_callback)
    rospy.Subscriber('alien_data', AlienDetection,alien_detection,queue_size=10)
    
    
    swift_drone = swift()
    alien_data_pub=rospy.Publisher('astrobiolocation',Biolocation,queue_size=10)
    r = rospy.Rate(33)
    j=0
    k=0
    is_change=False
    is_sec=False
    swift.sets(swift_drone,[k,j,10])
    whyco=[0,0,0]
    is_alien_meet=False
    is_going_to_research_station=False
        
    while not rospy.is_shutdown():
        
        
            whyco=drone_position
            
            while not equals([k,j,10],whyco) :
                
                rospy.loginfo(whyco)
                if not is_going_to_research_station:
                    if alien_type>1:
                        is_alien_meet=True
                        while True:
                            swift.center_of_frame_pid(swift_drone,centroid_x,centroid_y) 
                            swift_drone.pid() 
                            rospy.loginfo(f"{j} {k} 000000000000000000000000000")
                            swift.sets(swift_drone,drone_position)
                            #Subscribe a topic to break the loop 
                    if is_alien_meet:
                        break
                    
                rospy.loginfo(f"{j} {k} 000000000000000000000000000")
                swift_drone.pid() 
                whyco=drone_position
                r.sleep()
            if is_alien_meet:
                alien_info=Biolocation()
                alien_info.deserialize_numpy=get_alien_type(alien_type)
                alien_info.whycon_x=drone_position[0]
                alien_info.whycon_y=drone_position[1]
                alien_info.whycon_z=drone_position[2]
                alien_data_pub.publish(alien_info)
                is_going_to_research_station=True
                swift.sets(swift_drone,[11,11,37]) 
                
                
            rospy.loginfo(f"{j} {k}+''''''''''''''''''''''''''''''''''''''''''''''''''''''")
            if not is_alien_meet:
                if is_change and not is_sec:
                    j-=1
                    if j==-10:
                        is_sec=True
                        
                if not is_change:
                    j+=1
                    if j==10:
                        is_change=True
                swift.sets(swift_drone,[k,j,10])        
            
        
        
    

