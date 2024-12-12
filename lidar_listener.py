#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String  # To publish alarm messages

class DoorSurveillance360:
    def __init__(self):
        rospy.init_node('door_surveillance', anonymous=True)

        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.alarm_publisher = rospy.Publisher('/alarm_trigger', String, queue_size=10)

        self.previous_scan = None  # Store the previous LIDAR scan
        self.distance_change_threshold = 0.1  # Threshold for detecting movement (meters)

        rospy.loginfo("TurtleBot surveillance system initialized. Monitoring 360-degree environment...")

    def scan_callback(self, msg):
        # Check if we have a previous scan to compare
        if self.previous_scan is not None:
            # Loop through the ranges to detect significant changes
            movement_detected = False
            for i, (prev, curr) in enumerate(zip(self.previous_scan, msg.ranges)):
                if abs(curr - prev) > self.distance_change_threshold:
                    angle = msg.angle_min + i * msg.angle_increment
                    rospy.logwarn(f"Movement detected at angle {angle:.2f} radians, distance: {curr:.2f} meters!")
                    self.alarm_publisher.publish(f"Movement detected at angle {angle:.2f}, distance: {curr:.2f} meters!")
                    movement_detected = True
                    break  # Exit early to minimize performance impact

            if not movement_detected:
                rospy.loginfo("No significant movement detected.")

        # Update the previous scan for the next callback
        self.previous_scan = list(msg.ranges)  # Store a copy of the current scan

if __name__ == '__main__':
    try:
        DoorSurveillance360()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

