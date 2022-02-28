#!/usr/bin/env python2

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyController:
    SCAN_TOPIC = rospy.get_param("safety_controller/scan_topic")
    INCOMING_DRIVE_TOPIC = "/vesc/high_level/ackermann_cmd_mux/output"
    SAFETY_TOPIC = "/vesc/low_level/ackermann_cmd_mux/input/safety"

    def __init__(self):
        self.scan_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.obstacle_detector)
        self.drive_sub = rospy.Subscriber(self.INCOMING_DRIVE_TOPIC, AckermannDriveStamped, self.speed_analyzer)
        self.safety_pub = rospy.Publisher(self.SAFETY_TOPIC, AckermannDriveStamped, queue_size = 10)

        self.straight_ahead_distance = np.inf

    def obstacle_detector(self, scan):
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, num=len(scan.ranges))
        cartesian = np.stack( (np.multiply(ranges,np.cos(angles)), \
            np.multiply(ranges,np.sin(angles))), axis=1)
        middle_index = len(ranges)/2
        self.straight_ahead_distance = cartesian[middle_index,0]
        
    def speed_analyzer(self, drive_instructions):
        speed = drive_instructions.drive.speed
        steering_angle = drive_instructions.drive.steering_angle
        steering_angle_velocity = drive_instructions.drive.steering_angle_velocity
        if not can_car_make_turn(speed, steering_angle, steering_angle_velocity, self.straight_ahead_distance):
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.
            self.safety_pub.publish(msg)
        

def can_car_make_turn(speed, steering_angle, steering_angle_velocity, distance):
    """
    Return True if car can make turn, return False if not
    Could also add an automatic stop if distance is less than a certain value
    """
    time_until_straight_collision = distance / speed
    if steering_angle_velocity * time_until_straight_collision < abs(steering_angle): # steering_angle as proxy for slope of wall
        return False
    else:
        return True

if __name__ == "__main__":
    rospy.init_node('safety_controller')
    safety_controller = SafetyController()
    rospy.spin()