from sensor_msgs.msg import Image
import rospy
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
rospy.init_node("readCamera")


def image_callback(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Show the converted image
    cv.imshow("", cv_image)
    cv.waitKey(3)


 # Initalize a subscriber to the "" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber(
    "/swift/camera_rgb/image_raw", Image, image_callback)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()
