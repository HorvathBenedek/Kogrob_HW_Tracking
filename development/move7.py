#!/usr/bin/env python3

#import cv2
#from cv_bridge import CvBridge, CvBridgeError
#from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
import rospy
#try:
#    from queue import Queue
#except ImportError:
#    from Queue import Queue
#import threading
import numpy as np

#rospy.init_node('turtlesim_draw_circle') # Init the node with name "turtlesim_draw_circle"
rospy.init_node('line_follower') # Init the node with name "turtlesim_draw_circle"                         


pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

cmd_vel = Twist()
cmd_vel.linear.x = 0
cmd_vel.angular.z = 0.1

#pub.publish(cmd_vel)

while not rospy.is_shutdown(): # Run the node until Ctrl-C is pressed

    pub.publish(cmd_vel)           # Publishing twist message on topic "/turtle1/cmd_vel"
    #msg.linear.x += 0.005     # Uncomment this line to draw a spiral instead of circle
    rospy.sleep(1)   
#rospy.spin()
#rospy.spin()
