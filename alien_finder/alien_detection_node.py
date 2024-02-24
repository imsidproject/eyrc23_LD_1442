#!/usr/bin/env python3

'''
# Team ID:          eYRC#LD#1442
# Theme:            Luminosity Drone
# Author List:      Rupankar Podder, Sidharth Kumar Priyadarshi, Kausar Kamal
# Filename:         alien_detection_node.py
# Functions:        image_callback, img_operation
# Global variables: infi

This python file runs a ROS-node of name alien_detection that performs
image processing on the camera feed from the drone and publishes information about
the detected aliens
        Publications            Subscriptions
        /alien_info             /video_frames

'''


# import the necessary packages
from imutils import contours
from skimage import measure
import numpy as np
import cv2
from loc_msg.msg import AlienInfo
import rclpy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

infi = 1000000
class AlienDetection():
    def __init__(self, node):
        self.node=node
        self.bridge=CvBridge()
        self.img_sub = node.create_subscription(Image, '/video_frames',self.image_callback,1)
        self.alien_info_pub=node.create_publisher(AlienInfo,"/alien_info",1)
        self.alien_msg=AlienInfo()

    def image_callback(self, img):
        image = self.bridge.imgmsg_to_cv2(img, "passthrough")
        cv2.imshow("",image)
        cv2.waitKey(1)
        self.img_operation(image)
    
    def img_operation(self, image):
        
        # convert it to grayscale, and blur it
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

        # threshold the image to reveal light regions in the blurred image
        ret, thersholding = cv2.threshold(blurred_image, 150, 255, cv2.THRESH_BINARY)

        # perform a series of erosions and dilations to remove any small blobs of noise from the thresholded image
        kernel = np.ones((3, 3), np.uint8)
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


        # find the contours in the mask
        contours, hir = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # loop over the contours
        # Initialize lists to store centroid coordinates
        centroids = []
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            blob = mask[y:y+h, x:x+w]  # ROI
            ctrs, hir = cv2.findContours(blob, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # perform erosion until only the main contour remains in the ROI
            while len(ctrs) > 1:
                blob = cv2.erode(blob, kernel, iterations=1,
                                borderType=cv2.BORDER_CONSTANT, borderValue=0)
                ctrs, hir = cv2.findContours(
                    blob, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            # perform erosion for separation of joined blobs
            while True:
                blob = cv2.erode(blob, kernel, iterations=1,
                                borderType=cv2.BORDER_CONSTANT, borderValue=0)
                c, hir = cv2.findContours(blob, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

                # if the blobs are disappearing, or the first blob is very small, then stop
                if len(c)==0 or len(c) < len(ctrs) or cv2.contourArea(c[0]) < 10:
                    break
                ctrs = c

            # Append centroid coordinates
            for i in ctrs:
                (cx,cy),r= cv2.minEnclosingCircle(i)
                # M = cv2.moments(i)
                # cx = M['m10']/M['m00']
                # cy = M['m01']/M['m00']
                centroids.append((x+cx, y+cy))
        distances = np.empty(len(centroids), dtype=np.float64)
        for i in range(len(centroids)):
            if centroids[i] == (-1, -1):
                continue  # led already considered

            size = 1
            x = centroids[i][0]
            y = centroids[i][1]
            distances[i] = infi

            for j in range(i+1, len(centroids)):
                if centroids[j] == (-1, -1):
                    distances[j] = infi  # led already considered
                else:
                    distances[j] = np.linalg.norm(
                        np.array(centroids[i]) - np.array(centroids[j]))

            # set maxdistance as a multiple of smallest distance
            # there should be at least 2 leds in a cluster
            if np.min == infi:
                break
            max_dist = 4*np.min(distances)

            # finding centroid of cluster
            for j in range(i+1, len(centroids)):
                if distances[j] <= max_dist:
                    x += centroids[j][0]
                    y += centroids[j][1]
                    size += 1
                    # not to be considered in any other cluster
                    centroids[j] = (-1, -1)
            x /= size
            y /= size

            if size >= 1 and size<=5:
                self.alien_msg.x=x- 320 #shifting origin
                self.alien_msg.y=y- 240
                self.alien_msg.alien_type=size
                self.alien_info_pub.publish(self.alien_msg)
                return
                
        #if no alien found
        self.alien_msg.x=0.0
        self.alien_msg.y=0.0
        self.alien_msg.alien_type=0
        self.alien_info_pub.publish(self.alien_msg)


        

if __name__=='__main__':
    rclpy.init()
    node = rclpy.create_node('alien_detection')
    alien_detection = AlienDetection(node)
    rclpy.spin(node)
    rclpy.shutdown()    

