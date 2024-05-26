#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import rospy

def image(msg):
	cv2Img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")	 
	cv2.imshow("name", cv2Img)
	#cv2.waitKey()
	k = cv2.waitKey(6) & 0xFF


bridge = CvBridge()
#rospy.init_node('image_processing')
rospy.init_node('image_processing')

image_topic = "/camera/image/compressed"
rospy.Subscriber(image_topic, CompressedImage, image)

#cv2Img = bridge.compressed_imgmsg_to_cv2(image, desired_encoding="bgr8")

#cv2.imshow(cv2Img)

rospy.spin()
