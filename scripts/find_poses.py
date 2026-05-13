#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class PersonPoseFinder:
    def __init__(self):
        rospy.init_node('pose_finder', anonymous=True)

        self.bridge = CvBridge()

        rospy.Subscriber('/camera/rgb/image_raw', Image, self.look_for_hands)

        self.hand_pub = rospy.Publisher('/hand_raised_detected', Bool, queue_size=10)

        print("Watching for people raising hands...")

    def look_for_hands(self, msg):
        try:
            # get the image
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Openvino's Efficient HRNet would work also for this! I chose OpenPose as this is more adaptable to more
            # ways of activation and not just raising your hand
            # so here's a simpler way: look for a hand shape above a face

            # first find faces
            face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)

            if len(faces) == 0:
                return

            # take the biggest face
            (fx, fy, fw, fh) = max(faces, key=lambda r: r[2] * r[3])

            hand_region_y_start = max(0, fy - fh)
            hand_region_y_end = fy
            hand_region_x_start = max(0, fx - fw // 2)
            hand_region_x_end = min(img.shape[1], fx + fw + fw // 2)

            # check if there's skin color in that region
            hand_region = img[hand_region_y_start:hand_region_y_end, hand_region_x_start:hand_region_x_end]

            if hand_region.size == 0:
                return

            # convert to hsv for skin detection
            hsv_region = cv2.cvtColor(hand_region, cv2.COLOR_BGR2HSV)
            lower_skin = np.array([0, 20, 70])
            upper_skin = np.array([20, 255, 255])
            skin_mask = cv2.inRange(hsv_region, lower_skin, upper_skin)

            # if there's enough skin above the face, probably a hand raised
            skin_percentage = np.sum(skin_mask > 0) / (hand_region.shape[0] * hand_region.shape[1])

            if skin_percentage > 0.15:  # if more than 15% is skin
                self.hand_pub.publish(True)
                print("I see a hand raised!")
            else:
                self.hand_pub.publish(False)

        except Exception as e:
            print(f"pose finding error: {e}")


if __name__ == '__main__':
    finder = PersonPoseFinder()
    rospy.spin()