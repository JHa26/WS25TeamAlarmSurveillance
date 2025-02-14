#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading

class MotionDetector:
    def __init__(self):
        rospy.init_node('motion_detector', anonymous=True)
        
        # Subscribe to the camera image topic
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()
        
        # Initialize variables for movement detection
        self.previous_frame = None
        self.process_interval = 1.0  # Increased time in seconds between processing frames
        self.background_update_interval = 2  # Background frame update interval in seconds
        self.threshold_display_interval = 2  # Interval to update the threshold display window in seconds
        self.last_processed_time = time.time()
        self.last_background_update_time = time.time()
        self.last_threshold_display_time = time.time()
        
        # For threading display updates
        self.frame_to_show = None
        self.threshold_to_show = None
        self.display_thread = threading.Thread(target=self.display_frames)
        self.display_thread.daemon = True  # Daemonize thread so it exits when the main program does
        self.display_thread.start()

    def image_callback(self, data):
        # Throttle frame processing based on the time interval
        current_time = time.time()
        if current_time - self.last_processed_time < self.process_interval:
            return  # Skip this frame

        # Convert the ROS Image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # Resize the frame to reduce resolution and processing load
        frame = cv2.resize(frame, (320, 240))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        # Update background frame only at specified intervals
        if self.previous_frame is None or current_time - self.last_background_update_time > self.background_update_interval:
            self.previous_frame = gray
            self.last_background_update_time = current_time
            return

        # Calculate the difference between the current and previous frame
        frame_delta = cv2.absdiff(self.previous_frame, gray)
        _, thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)

        # Dilate the thresholded image and find contours
        thresh = cv2.dilate(thresh, None, iterations=2)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw rectangles around detected movements
        for contour in contours:
            if cv2.contourArea(contour) < 500:  # Ignore small movements
                continue
            (x, y, w, h) = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Update frames for display in the separate thread
        self.frame_to_show = frame
        if current_time - self.last_threshold_display_time > self.threshold_display_interval:
            # Resize the threshold image for efficiency
            self.threshold_to_show = cv2.resize(thresh, (160, 120))
            self.last_threshold_display_time = current_time

        # Update the last processed time
        self.last_processed_time = current_time

    def display_frames(self):
        # Display frames in a separate thread to avoid blocking the main thread
        while not rospy.is_shutdown():
            if self.frame_to_show is not None:
                cv2.imshow("Camera Feed", self.frame_to_show)
            if self.threshold_to_show is not None:
                cv2.imshow("Threshold", self.threshold_to_show)
            cv2.waitKey(1)  # Wait for a brief moment to handle display updates

if __name__ == '__main__':
    try:
        MotionDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
