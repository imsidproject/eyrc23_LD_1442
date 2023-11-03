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

arrived=False
alien_type=0
centroid_x=-1
centroid_y=-1
frame=None
is_other_pid_active=False
def other_pid_active(msg):
    global is_other_pid_active
    is_other_pid_active = msg.data
    
def alien_detection(data):
    global alien_type, centroid_x, centroid_y
    alien_type=data.alien_type
    centroid_x=data.centroid_x
    centroid_y=data.centroid_y
    rospy.loginfo(f"alien data recieved :-{str(alien_type)}, {str(centroid_x)}, {str(centroid_y)}]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]")



def is_arrived(msg):
    global arrived 
    arrived=msg.data
    

if __name__ == '__main__':
    rospy.init_node("path_decider")
    rospy.Subscriber('alien_data', AlienDetection,alien_detection,queue_size=10)
    rospy.Subscriber('otherPidActive',Bool,other_pid_active,queue_size=10)
    is_alien_meet_pub=rospy.Publisher('alienMeet',Bool,queue_size=10)

    next_coordinate_pub=rospy.Publisher('nextCoordinate',Coordinate,queue_size=10)
    r = rospy.Rate(33)
    rospy.Subscriber('isArrived',Bool,is_arrived,queue_size=10)
    j=0
    k=0
    is_change=False
    is_going_neg_x=True
    is_going_pos_x=False
    is_apppopriate_pos=False
    count=0
    is_alien_meet=False
    

        
    while not rospy.is_shutdown():
        if not is_other_pid_active:
            
            if not is_alien_meet:
                if arrived:                
                        #rospy.loginfo("Not arrived till now")
                    
                    arrived=False
                    rospy.loginfo("Arrived................................")
                    count+=1
                    if alien_type>1:
                        is_alien_meet=True
                        continue
                    #path planning 

                    if is_going_pos_x and is_apppopriate_pos:
                            if is_change :
                                if j==-10:
                                        k+=2
                                        is_change=False
                                if j!=-10:
                                    j-=10
                            if not is_change:
                                if j==10:
                                        is_change=True 
                                        k+=2 
                                if j!=10:    
                                    j+=10
                            if k==10 and j==10:
                                is_going_neg_x=False
                                is_going_pos_x=False
                        

                                        
                    if not is_going_neg_x and not is_apppopriate_pos:
                        k=0
                        is_apppopriate_pos=True



                    if is_going_neg_x:
                            if is_change :
                                if j==-10:
                                        k-=2
                                        is_change=False
                                        
                                if j!=-10:
                                    j-=10
                                    
                                                    
                            if not is_change:
                                if j==10:
                                        is_change=True 
                                        k-=2 
                                
                                if j!=10:           
                                    j+=10
                                    
                            if k==-10 and j==10:
                                is_going_neg_x=False
                                is_going_pos_x=True
                                is_change=False

                    # if is_change and not is_sec:
                    #     j-=10
                    #     if j==-10:
                    #         is_sec=True
                            
                    # if not is_change:
                    #     j+=10
                    #     if j==10:
                    #         is_change=True
                    coordinate=Coordinate()
                    coordinate.x=k
                    coordinate.y=j
                    coordinate.z=10        
                    next_coordinate_pub.publish(coordinate)    
                    rospy.loginfo(f"Next coordinate Published {k}, {j}................................")
                else:
                    
                        init_coordinate=Coordinate()
                        init_coordinate.x=k
                        init_coordinate.y=j
                        init_coordinate.z=27
                        next_coordinate_pub.publish(init_coordinate)    
                        rospy.loginfo(f"Initial coordinate Published {k}, {j}, ................................")   
            else:
                is_alien_meet_pub.publish(True)
                init_coordinate=Coordinate()
                init_coordinate.x=k
                init_coordinate.y=j
                init_coordinate.z=27
                next_coordinate_pub.publish(init_coordinate)    
                rospy.loginfo(f"Stable when alien met {k}, {j}, ................................")  
        else:
            rospy.loginfo(f"Other pid Active")  

        r.sleep()
    rospy.spin()    
                
            