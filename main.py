#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import message_filters


class MyRobotMapper:
    def __init__(self):
        rospy.init_node('my_robot_mapper', anonymous=True)

        # astra camera specs
        self.camera_fov_h = 60.0 * 3.14159 / 180.0  # 60 degrees to radians
        self.camera_fov_v = 49.5 * 3.14159 / 180.0  # 49.5 degrees to radians
        self.img_w = 640
        self.img_h = 480

        # where is my robot right now?
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_angle = 0.0  # which way it's facing

        self.hand_up = False

        self.bridge = CvBridge()

        # get camera images (rgb and depth at the same time)
        rgb_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)

        # sync them up so they match
        self.ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 0.1)
        self.ts.registerCallback(self.got_images)

        # listen to where the robot is (from SLAM stuff)
        rospy.Subscriber('/robot_pose', PoseStamped, self.update_robot_pose)

        # listen for hand gesture
        rospy.Subscriber('/hand_raised_detected', Bool, self.hand_detected)

        # send out where I found the person
        self.person_pub = rospy.Publisher('/person_location', PointStamped, queue_size=10)

        # send out markers for visualization
        self.marker_pub = rospy.Publisher('/person_markers', MarkerArray, queue_size=10)

        print("Ready to find people!")

    def update_robot_pose(self, msg):
        # get robot's position from the message
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y

        # convert quaternion to angle (yaw)
        q = msg.pose.orientation
        # this math converts quaternion to euler angles
        self.robot_angle = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def hand_detected(self, msg):
        self.hand_up = msg.data
        if self.hand_up:
            print("Someone raised their hand! Looking for them...")

    def calculate_xy_from_pixels(self, px, py, depth):
        # THESE ARE EQUATIONS 1 AND 2 FROM THE PAPER
        # x = (2*x'/w') * d * tan(β/2)
        # y = (2*y'/h') * d * tan(α/2)

        # pixel to meter conversion
        x_meters = (2.0 * px / self.img_w) * depth * math.tan(self.camera_fov_h / 2.0)
        y_meters = (2.0 * py / self.img_h) * depth * math.tan(self.camera_fov_v / 2.0)

        return x_meters, y_meters

    def transform_to_map(self, x_rel, y_rel):
        # EQUATIONS 3, 4, 5, 6, 7
        # rotating coordinates to match the map

        # AC is the straight line distance (hypotenuse)
        ac_dist = math.sqrt(x_rel * x_rel + y_rel * y_rel)

        # angle calculations
        arctan_stuff = math.atan2(x_rel, y_rel)
        angle_90_minus = 3.14159 / 2 - self.robot_angle

        # relative coordinates in map frame
        total_angle = angle_90_minus - arctan_stuff
        x_relative_map = math.sin(total_angle) * ac_dist
        y_relative_map = math.cos(total_angle) * ac_dist

        # add robot's current position to get absolute coordinates
        global_x = self.robot_x + x_relative_map
        global_y = self.robot_y + y_relative_map

        return global_x, global_y

    def find_person_in_image(self, rgb_image):
        # i'm supposed to use OpenVINO EfficientHRNet here
        # but that's complicated so for now let's just find the brightest spot?
        # wait that's a bad idea

        # actually let's just look for skin color as a simple way
        # convert to HSV color space
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        # skin color range (roughly)
        lower_skin = np.array([0, 20, 70], dtype=np.uint8)
        upper_skin = np.array([20, 255, 255], dtype=np.uint8)

        skin_mask = cv2.inRange(hsv, lower_skin, upper_skin)

        # find the largest skin area
        contours, _ = cv2.findContours(skin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return cx, cy

        # if no skin found, just use center
        return self.img_w // 2, self.img_h // 2

    def draw_circle_around_person(self, center_x, center_y, radius=0.5):
        # Eq. 8!
        # make points around the person that the robot can drive to

        circle_points = []
        num_points = 8  # 8 points around the circle

        for i in range(num_points):
            angle = 2 * 3.14159 * i / num_points
            px = center_x + radius * math.cos(angle)
            py = center_y + radius * math.sin(angle)
            circle_points.append((px, py))

        return circle_points

    def got_images(self, rgb_msg, depth_msg):
        # this runs every time we get new camera images

        if not self.hand_up:
            return

        try:
            # convert ros images to opencv to better visualize for the user
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

            person_px, person_py = self.find_person_in_image(rgb_img)

            depth_value = depth_img[person_py, person_px] / 1000.0

            x_rel, y_rel = self.calculate_xy_from_pixels(person_px, person_py, depth_value)

            global_x, global_y = self.transform_to_map(x_rel, y_rel)

            print(f"Found person at: ({global_x:.2f}, {global_y:.2f})")

            person_point = PointStamped()
            person_point.header.frame_id = "map"
            person_point.header.stamp = rospy.Time.now()
            person_point.point.x = global_x
            person_point.point.y = global_y
            self.person_pub.publish(person_point)

            approach_points = self.draw_circle_around_person(global_x, global_y)

            # make markers so I can see them in rviz
            markers = MarkerArray()
            for i, (px, py) in enumerate(approach_points):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.id = i
                marker.type = 1  # sphere
                marker.action = Marker.ADD
                marker.pose.position.x = px
                marker.pose.position.y = py
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                markers.markers.append(marker)

            self.marker_pub.publish(markers)

            self.hand_up = False

        except Exception as e:
            print(f"Oops something went wrong: {e}")


if __name__ == '__main__':
    try:
        mapper = MyRobotMapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass