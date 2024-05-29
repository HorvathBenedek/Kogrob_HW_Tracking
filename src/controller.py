#!/usr/bin/python3

# ROS
import rospy
from geometry_msgs.msg import Twist
from kogrob_tracking.srv import *
from math import sqrt

class Controller:

    def __init__(self) -> None:
        self.move = Twist()
        self.freeze = Twist()
        self.cmd_publisher = rospy.Publisher('/follower/cmd_vel', Twist, queue_size=100)
        self.angular_error = 0
        self.distance_error = 0

        # Parameters for P-controller
        self.angular_vel_coef = 0.01
        self.linear_vel_coef = 0.9
        self.safe_distance = 0.3  # Desired distance from the target in meters

        # Known bounding box height at 1 meter (for example)
        self.known_height_at_1m = 40.0  # This needs to be calibrated with actual measurements

        # Create a service proxy for human detection service
        rospy.wait_for_service('detection')
        self.detection = rospy.ServiceProxy('detection', Detection)

    def find_errors(self, response) -> None:
        box_x = response.box_x
        box_y = response.box_y
        box_width = response.box_width
        box_height = response.box_height
        image_width = response.image_width
        image_height = response.image_height

        box_center_x = box_x + (box_width / 2)
        img_center_x = image_width / 2

        if box_center_x > img_center_x:
            direction = -1
        else:
            direction = 1

        self.angular_error = direction * abs(img_center_x - box_center_x)
        
        # Estimate distance based on the bounding box height
        self.distance_error = self.estimate_distance(box_height)

    def estimate_distance(self, box_height: float) -> float:
        # Simple linear model: distance = known_height_at_1m / box_height
        # Adjust the 1.0 factor to calibrate for actual distances
        if box_height > 0:
            estimated_distance = self.known_height_at_1m / box_height
        else:
            estimated_distance = float('inf')  # If height is zero, set distance to infinity

        return self.safe_distance - estimated_distance

    def run(self) -> None:
        try:
            while not rospy.is_shutdown():
                label = "person"
                response = self.detection(label)

                if response.in_sight_of_robot:
                    self.find_errors(response)
                    
                    # Angular velocity control
                    self.move.angular.z = self.angular_vel_coef * self.angular_error

                    # Linear velocity control
                    if self.distance_error < 0:
                        self.move.linear.x = self.linear_vel_coef * abs(self.distance_error)  # Move backward if too close
                    else:
                        self.move.linear.x = -self.linear_vel_coef * self.distance_error  # Move forward if too far

                    self.cmd_publisher.publish(self.move)
                else:
                    self.move.linear.x = 0  # Stop linear movement when target is lost
                    self.move.angular.z = 0.3  # Rotate to search for the target
                    self.cmd_publisher.publish(self.move)

        except rospy.exceptions.ROSInterruptException:
            pass

if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    controller = Controller()
    controller.run()