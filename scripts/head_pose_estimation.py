#!/usr/bin/env python3
# this figures out which way the person is looking
# so the robot can approach from the front (like the paper says)

import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray


class HeadPoseEstimator:
    def __init__(self):
        rospy.init_node('head_pose_estimator', anonymous=True)

        self.bridge = CvBridge()

        # look at camera
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.process_face)

        # publish which way they're looking (yaw, pitch, roll)
        self.gaze_pub = rospy.Publisher('/person_gaze', Float32MultiArray, queue_size=10)

        # load face detector
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )

        # these are approximate facial feature points (simplified)
        # in real implementation you'd use a proper CNN model
        self.model_points = np.array([
            (0.0, 0.0, 0.0),  # Nose tip
            (0.0, -0.1, -0.1),  # Chin
            (-0.1, 0.0, -0.1),  # Left eye left corner
            (0.1, 0.0, -0.1),  # Right eye right corner
            (-0.05, -0.05, -0.1),  # Left mouth corner
            (0.05, -0.05, -0.1)  # Right mouth corner
        ], dtype=np.float64)

        print("Head pose estimator ready - figuring out where people are looking")

    def estimate_head_pose(self, face_img, face_rect):
        # this is a simplified version of head pose estimation
        # real implementation would use a CNN like in the paper

        # this is used PURELY for testing for now

        # simulate looking slightly towards the robot
        yaw = random.uniform(-15, 15)  # looking left/right
        pitch = random.uniform(-10, 10)  # looking up/down
        roll = random.uniform(-5, 5)  # head tilt

        return yaw, pitch, roll

    def process_face(self, msg):
        try:
            # get image
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # find faces
            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

            if len(faces) == 0:
                return

            # take biggest face
            (x, y, w, h) = max(faces, key=lambda r: r[2] * r[3])
            face_roi = img[y:y + h, x:x + w]

            if face_roi.size == 0:
                return

            # figure out which way they're looking
            yaw, pitch, roll = self.estimate_head_pose(face_roi, (x, y, w, h))

            # publish angles
            gaze_msg = Float32MultiArray()
            gaze_msg.data = [yaw, pitch, roll]
            self.gaze_pub.publish(gaze_msg)

            # draw on image for debugging (optional)
            if yaw > 10:
                direction = "looking right"
            elif yaw < -10:
                direction = "looking left"
            else:
                direction = "looking forward"

            print(f"Person is {direction} (yaw={yaw:.1f}°)")

        except Exception as e:
            print(f"head pose error: {e}")


if __name__ == '__main__':
    import random  # only for TESTING PURPOSES !!!

    estimator = HeadPoseEstimator()
    rospy.spin()