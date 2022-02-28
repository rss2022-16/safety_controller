#!/usr/bin/env python2

"""
Safety controller for RSS bot
Subscribes to wall follower output, as well as scan data, and publishes to a safety topic

What are we checking?
- Time since last message from wall follower and laser scan
- Will the car bump into an obstacle soon
    - Modeling the car as a circle of radius L (=.15m)
    - "soon" = a set time interval (Using linearized dynamics - maybe a bad assumption?)
    - "obstacle" = a nearby lidar point
"""

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyController:

    def __init__(self, namespace):

        # Parameters
        self.DT = .1
        self.scan_topic = rospy.get_param("~scan_topic")
        self.drive_in_topic = rospy.get_param("~drive_in_topic")
        self.drive_out_topic = rospy.get_param("~drive_out_topic")
        self.drive_comm_timeout = rospy.Duration(rospy.get_param("~drive_comm_timeout"))
        self.scan_comm_timeout = rospy.Duration(rospy.get_param("~scan_comm_timeout"))
        self.collision_interval = rospy.get_param("~collision_interval")
        self.length = rospy.get_param("~length")

        # Pub and sub
        self.drive_pub = rospy.Publisher(self.drive_out_topic, AckermannDriveStamped, queue_size=10)
        self.drive_sub = rospy.Subscriber(self.drive_in_topic, AckermannDriveStamped, self.drive_cb)
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_cb)

        # Class vars
        self.cmd = AckermannDriveStamped()
        self.cmd.header.stamp = rospy.Time.now()
        self.scan = LaserScan()
        self.scan.header.stamp = rospy.Time.now()

        # Callbacks and shutdown behavior
        rospy.Timer(rospy.Duration(self.DT), self.cb)
        rospy.on_shutdown(self.hook)


    def scan_cb(self, scan):
        """
        Records LaserScan data
        """
        self.scan = scan
        

    def drive_cb(self, cmd):
        """
        Records commanded velocity and steering angle from wall follower
        """
        self.cmd = cmd


    def cb(self, event):
        """
        Main safety callback
        """
        if rospy.Time.now() - self.cmd.header.stamp >= self.drive_comm_timeout:
            self.send_drive(0, 0)
            rospy.logwarn("Haven't received a message from drive topic in a while. Stopping...")
            return

        if rospy.Time.now() - self.scan.header.stamp >= self.scan_comm_timeout:
            self.send_drive(0, 0)
            rospy.logwarn("Haven't received a message from scan topic in a while. Stopping...")
            return

        if self.collision_detected():
            self.send_drive(0, 0)
            rospy.logwarn("Detected incoming collsion. Stopping...")
            return


    def collision_detected(self):
        """
        Checks if we are expecting to run into an obstacle within a set time frame
        """

        # Truncate scan data to nearby & ahead points
        ranges = np.array(self.scan.ranges)
        angles = np.linspace(self.scan.angle_min, self.scan.angle_max, num=len(self.scan.ranges))

        ceiling = self.cmd.drive.speed * self.collision_interval + self.length
        far_points = [i for i in range(len(ranges)) if ranges[i] >= ceiling or abs(angles[i]) >= np.pi/2]
        ranges = np.delete(ranges, far_points, axis=0)
        angles = np.delete(angles, far_points, axis=0)

        # Convert to x,y
        xy = np.stack( (np.multiply(ranges,np.cos(angles)), np.multiply(ranges,np.sin(angles))), axis=1)

        # Calculate future car location -- Linearize (Is this a good assumption?)
        carpoint = [self.cmd.drive.speed * self.collision_interval, 0]

        # # Calculate future car location -- Non-linearly (In progress)
        # arclength = self.cmd.drive.speed * self.collision_interval
        # if np.tan(eta) != 0:
        #     radius = L / np.tan(eta)
        #     theta = arclength / radius
        # else:
        #     radius = 'inf'
        
        # Check if there's a collision
        for p in xy:
            # Bubble method
            # if np.sqrt((p[0] - carpoint[0])**2 + (p[1] - carpoint[1])**2) <= self.length:
            # Rectangle method
            if p[0] >= 0 and p[0] <= (carpoint + self.length) and abs(p[1]) <= self.length:
                return True

        return False


    def hook(self):
        """
        Safe shutdown behavior. Send 0 commands to vehicle
        """
        self.send_drive(0, 0)


    def send_drive(self, v, eta):
        """
        Helper function -- sends v, eta to the car
        """
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.drive.steering_angle = eta
        msg.drive.speed = v
        self.drive_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node('safety_controller')
    ns = rospy.get_namespace()
    node = SafetyController(ns)
    rospy.spin()