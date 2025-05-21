#!/usr/bin/env python3

# Importar las librerías necesarias
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import ackermann_msgs.msg

# TODO Declare the mediapipe pose detector to be used
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False,
                       max_num_hands=2,
                       min_detection_confidence=0.7,
                       min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

# Control message publisher
ackermann_command_publisher = None

#Operator image processing
def image_callback(msg):
    global ackermann_command_publisher

    bridge = CvBridge()
    try:
        # Convert ROS image to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # TODO Processing the image with MediaPipe
    image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    result = hands.process(image_rgb)

    # TODO Recognise the gesture by means of some classification from the landmarks.
    # Initialize gesture variable, that will define the steering of the robot
    speed = 0
    steer = 0
    # The first hand detected will indicate the speed
    if result.multi_hand_landmarks:
        # Get hand landmarks
        hand_landmarks = result.multi_hand_landmarks[0]
        h, w, _ = cv_image.shape
        # create a landmarks matrix
        landmarks = [(int(lm.x * w), int(lm.y * h)) for lm in hand_landmarks.landmark]
        # Save the height of the wrist of the hand
        wrist_y = landmarks[0][1]
        # save the height of the hand index and middle finger
        index_finger_tip_y = landmarks[8][1]
        middle_finger_tip_y = landmarks[12][1]
        # If the finger is over the wrist, move forward
        if index_finger_tip_y < wrist_y:
            # use middle and index finger to move faster or slower
            if index_finger_tip_y > middle_finger_tip_y:
                speed = 1
            else:
                speed = 0.5
        # If the finger is under the wrist, move backwards
        else:
            speed = - 0.5
        # TODO Draw landsmarks on the image
        mp_drawing.draw_landmarks(cv_image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        # If a second hand is detected,it will indicate the steer
        if len(result.multi_hand_landmarks) == 2:
            # First hand detected indicates the steereing angle
            print(len(result.multi_hand_landmarks))
            hand_landmarks = result.multi_hand_landmarks[1]
            h, w, _ = cv_image.shape
            # create a landmarks matrix
            landmarks = [(int(lm.x * w), int(lm.y * h)) for lm in hand_landmarks.landmark]
            # Save the coordinates of the wrist of the hand
            wrist_x = landmarks[0][0]
            # save the coordinate x of the thumb
            thumb_tip_x = landmarks[4][0]
            # look if the thumb is at the left or right of the wrist
            if thumb_tip_x < wrist_x:
                steer = 0.5
            elif thumb_tip_x > wrist_x:
                steer = -0.5
            else:
                steer = 0
            # TODO Draw landsmarks on the image
            mp_drawing.draw_landmarks(cv_image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            

    # Display image with detected landmarks/gestures
    cv2.imshow("Hand pose Estimation", cv_image)
    cv2.waitKey(1)

    # TODO Interpret the obtained gesture and send the ackermann control command.
    cmd = ackermann_msgs.msg.AckermannDrive()
    # Define the speed and steer depending on the gesture
    cmd.speed = speed
    cmd.steering_angle = steer
    # Publish the speed and steer
    ackermann_command_publisher.publish(cmd)

def main():
    global ackermann_command_publisher
    rospy.init_node('pose_estimation', anonymous=True)
    rospy.Subscriber("/operator/image", Image, image_callback)
    ## Publisher definition
    ackermann_command_publisher = rospy.Publisher(
            "/blue/preorder_ackermann_cmd",
            ackermann_msgs.msg.AckermannDrive,
            queue_size=10,
        )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
