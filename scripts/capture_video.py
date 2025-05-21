#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def video_publisher():
    # Initialise a ROS node
    rospy.init_node('video_publisher', anonymous=True)

    # TODO Create a publisher in the /operator/image topic.
    pub = rospy.Publisher('/operator/image', Image, queue_size=10)

    # TODO Set up video capture from webcam (or from video)
    # Initialize capture using webcam
    cap = cv2.VideoCapture(0)
    # Log if webcam is not available
    if not cap.isOpened():
        rospy.logerr("Cannot open video source")
        return

    # Create an instance of CvBridge to convert OpenCV images to ROS messages.
    bridge = CvBridge()

    # Define the publication rate (e.g., 10 Hz).
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        # TODO Capture a frame from the webcam
        ret, frame = cap.read()
        # Log ifthe frame is not captured
        if not ret:
            rospy.logwarn("Failed to capture frame")
            break
                
        # TODO Convert OpenCV frame to ROS message
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # TODO Post the message in the topic
        pub.publish(msg)

        # Waiting to meet the publication rate
        rate.sleep()

    # When you're done, release the catch
    cap.release()

if __name__ == '__main__':
    try:
        video_publisher()
    except rospy.ROSInterruptException:
        pass
