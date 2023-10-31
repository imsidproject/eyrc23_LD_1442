#!/bin/python3
import numpy as np
from sensor_msgs.msg import Image
import cv2
from luminosity_drone.msg import AlienDetection
from cv_bridge import CvBridge 
import rospy
from imutils import contours
from skimage import measure

# load the image,

rate=None
pub =None
def get_frame(data):
    br = CvBridge()
    if data !=None:
        current_frame = br.imgmsg_to_cv2(data)
        rospy.loginfo("frame recieved")
        detect_alien(current_frame)
    

def get_Center(contour):
    '''
    Purpose:
    ___
    Used as key to sort contours.
    Returns the x coordinate of centroid of a contour

    Input Args:
    ___
    OpenCV contour (matlike)

    Returns:
    ___
    x-coordinate of centroid
    '''
    (x, y), r = cv2.minEnclosingCircle(contour)
    return x
# convert it to grayscale, and blur it
def detect_alien(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

    # threshold the image to reveal light regions in the blurred image
    ret, thersholding = cv2.threshold(blurred_image, 200, 255, cv2.THRESH_BINARY)

    # perform a series of erosions and dilations to remove any small blobs of noise from the thresholded image
    kernel = np.ones((5, 5), np.uint8)
    morph = cv2.morphologyEx(thersholding, cv2.MORPH_CLOSE, kernel)
    morph = cv2.morphologyEx(morph, cv2.MORPH_OPEN, kernel)


    # perform a connected component analysis on the thresholded image, then initialize a mask to store only the "large" components
    # loop over the unique components
    # if this is the background label, ignore it
    # otherwise, construct the label mask and count the number of pixels
    # if the number of pixels in the component is sufficiently large, then add it to our mask of "large blobs"


    labeled = measure.label(morph)  # finding connected components
    sizes = np.bincount(labeled.ravel())  # sizes of each connected component
    blob_sizes = sizes > 60  # ignoring small blobs
    blob_sizes[0] = 0  # ignoring background
    mask = blob_sizes[labeled]  # constructing a binary mask
    # constructing mat object from binary mask
    mask = np.array(mask, dtype=np.uint8)*255

    # find the contours in the mask, then sort them from left to right
    contours, hir = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    sorted_ctrs = sorted(contours, key=get_Center)

    # loop over the contours
    # Initialize lists to store centroid coordinates and area
    areas = []
    centroids = []
    for c in sorted_ctrs:
        (x, y), radius = cv2.minEnclosingCircle(c)
        # Calculate the area of the contour
        area = cv2.contourArea(c) + cv2.arcLength(c, True)/2.0
        # Append centrid coordinates and area to the respective lists
        areas.append(area)
        centroids.append((x, y))
        # Draw the bright spot on the image
        cv2.drawContours(image, [c], 0, (0, 0, 255), 3)
    alien_detection=AlienDetection()
    alien_detection.alien_type=0
    alien_detection.centroid_x=-1
    alien_detection.centroid_y=-1
    while not rospy.is_shutdown():
        alien_detection.alien_type=len(centroids)
        if len(centroids) !=0:
            alien_detection.centroid_x=centroids[0][0]
            alien_detection.centroid_y=centroids[0][1]
        pub.publish(alien_detection)
        rospy.loginfo("data publised")
        rate.sleep()
        

if __name__ == '__main__':
    rospy.init_node('image_analysis')
    rospy.Subscriber('/swift/camera_rgb/image_raw', Image, get_frame)   
    pub=rospy.Publisher('alien_data',AlienDetection,queue_size=10)
    rate=rospy.Rate(10)
    rospy.spin()

    