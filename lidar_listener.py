#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

class LidarListener:
    def __init__(self):
        rospy.init_node('lidar_listener', anonymous=True)
        
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        min_distance = min(msg.ranges) 
        min_angle = msg.angle_min + msg.ranges.index(min_distance) * msg.angle_increment

        rospy.loginfo(f'Closest obstacle at {min_distance:.2f} meters, angle: {min_angle:.2f} radians')

        if min_distance < 0.4:
            rospy.logwarn('Obstacle too close!')

if __name__ == '__main__':
    try:
        LidarListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
