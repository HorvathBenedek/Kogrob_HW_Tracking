#!/usr/bin/env python3
#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
import rospy
try:
    from queue import Queue
except ImportError:
    from Queue import Queue
import threading
import numpy as np

class BufferQueue(Queue):
    """Slight modification of the standard Queue that discards the oldest item
    when adding an item and the queue is full.
    """
    def put(self, item, *args, **kwargs):
        # The base implementation, for reference:
        # https://github.com/python/cpython/blob/2.7/Lib/Queue.py#L107
        # https://github.com/python/cpython/blob/3.8/Lib/queue.py#L121
        with self.mutex:
            if self.maxsize > 0 and self._qsize() == self.maxsize:
                self._get()
            self._put(item)
            self.unfinished_tasks += 1
            self.not_empty.notify()

class ImageRecognition(threading.Thread):
    """
    Thread that displays and processes the current image
    It is its own thread so that all display can be done
    in one thread to overcome imshow limitations and
    https://github.com/ros-perception/image_pipeline/issues/85
    """
    def __init__(self, queue):
        threading.Thread.__init__(self)
        self.queue = queue
        self.image = None
        self.robotcontrol = RobotMotion()

    def run(self):
        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800,600)

        while True:
            self.image = self.queue.get()

            # Process the current image
            # self.processImage(self.image)
            mask, contour, crosshair = self.processImage(self.image)
            contoured_image = self.AddContourMask(self.image, contour)
            cv2.imshow("contour", contoured_image)

            cv2.imshow("frame", self.image)
            self.robotcontrol.MoveRobot(0, 0, 0.2)

            # Check for 'q' key to exit
            k = cv2.waitKey(6) & 0xFF
            if k in [27, ord('q')]:
                # Stop every motion
                self.robotcontrol.MoveRobot(0, 0, 0)
                # Quit
                rospy.signal_shutdown('Quit')

    def processImage(self, img):

        rows,cols = img.shape[:2]
        H,L,S = self.convert2hls(img)

        # apply a polygon mask to filter out simulation's bright sky
        L_masked, mask = self.applyPolygonMask(L)

        # For light line on dark background in simulation:
        lightnessMask = self.thresholdBinary(L, (50, 255))

        stackedMask = np.dstack((lightnessMask, lightnessMask, lightnessMask))
        contourMask = stackedMask.copy()
        crosshairMask = stackedMask.copy()

        # return value of findContours depends on OpenCV version
        (contours,hierarchy) = cv2.findContours(lightnessMask.copy(), 1, cv2.CHAIN_APPROX_NONE)

        # overlay mask on lightness image to show masked area on the small picture
        # lightnessMask = cv2.addWeighted(mask,0.2,lightnessMask,0.8,0)

        # Find the biggest contour (if detected)
        if len(contours) > 0:

            biggest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(biggest_contour)

            # Make sure that "m00" won't cause ZeroDivisionError: float division by zero
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0

            # Show contour and centroid
            cv2.drawContours(contourMask, biggest_contour, -1, (0,255,0), 10)
            cv2.circle(contourMask, (cx, cy), 20, (0, 0, 255), -1)

            # Show crosshair and difference from middle point
            cv2.line(crosshairMask,(cx,0),(cx,rows),(0,0,255),10)
            cv2.line(crosshairMask,(0,cy),(cols,cy),(0,0,255),10)
            cv2.line(crosshairMask,(int(cols/2),0),(int(cols/2),rows),(255,0,0),10)

            # Chase the ball
#            if abs(cols/2 - cx) > 20:
#                self.cmd_vel.linear.x = 0.05
#                if cols/2 > cx:
#                    self.cmd_vel.angular.z = 0.15
#                else:
#                    self.cmd_vel.angular.z = -0.15
#
#            else:
#                self.cmd_vel.linear.x = 0.1
#                self.cmd_vel.angular.z = 0
#
#        else:
#            self.cmd_vel.linear.x = 0
#            self.cmd_vel.angular.z = 0
#
#        # Publish cmd_vel
#        pub.publish(self.cmd_vel)
#
#        # Return processed frames
        return lightnessMask, contourMask, crosshairMask

    # convert to HLS color space
    def convert2hls(self, img):
        hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        H = hls[:, :, 0]
        L = hls[:, :, 1]
        S = hls[:, :, 2]

        return H, L, S

    # apply a trapezoid polygon mask, size is hardcoded for 640x480px
    def applyPolygonMask(self, img):
        mask = np.zeros_like(img)
        ignore_mask_color = 255
        imshape = img.shape
        vertices = np.array([[(0,imshape[0]),(200, 200), (440, 200), (imshape[1],imshape[0])]], dtype=np.int32)
        cv2.fillPoly(mask, vertices, ignore_mask_color)
        masked_image = cv2.bitwise_and(img, mask)

        return masked_image, mask

    # Apply threshold and result a binary image
    def thresholdBinary(self, img, thresh=(200, 255)):
        binary = np.zeros_like(img)
        binary[(img >= thresh[0]) & (img <= thresh[1])] = 1

        return binary*255


    def AddContourMask(self, img, contour):
        if len(contour.shape) == 2:
            contour = np.stack((contour, contour, contour))
        #img = img + contour
        img = contour
        return img


def queueMonocular(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        #cv2Img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8") # in case of non-compressed image stream only
        cv2Img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        qMono.put(cv2Img)

class RobotMotion():
    def __init__(self):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0
        self.cmd_vel.linear.y = 0
        self.cmd_vel.angular.z = 0

    def MoveRobot(self, vx, vy, vphi):
        self.cmd_vel.linear.x = vx
        self.cmd_vel.linear.y = vy
        self.cmd_vel.angular.z = vphi
        self.publisher.publish(self.cmd_vel)
	#rospy.sleep(1)

#Main program
queueSize = 1
qMono = BufferQueue(queueSize)

bridge = CvBridge()

image_topic = "/camera/image/compressed"

rospy.Subscriber(image_topic, CompressedImage, queueMonocular)

rospy.init_node('OwnProject')

imagerec = ImageRecognition(qMono)
imagerec.setDaemon(True)
imagerec.start()

#robotmotion = RobotMotion()

#while not rospy.is_shutdown():
#    robotmotion.MoveRobot(0.1, 0.1, 1)
#    rospy.sleep(1)


rospy.spin()
