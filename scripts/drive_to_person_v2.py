#!/usr/bin/env python3
# this version actually uses head pose to pick where to drive
# like the paper says: approach from where they're looking

import rospy
import math
from geometry_msgs.msg import PointStamped, Twist
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32MultiArray
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class SmartDriveToPerson:
    def __init__(self):
        rospy.init_node('smart_drive_to_person', anonymous=True)

        # listen for person location
        rospy.Subscriber('/person_location', PointStamped, self.person_found)

        # listen for circle points
        rospy.Subscriber('/person_markers', MarkerArray, self.got_circle_points)

        # listen for which way they're looking
        rospy.Subscriber('/person_gaze', Float32MultiArray, self.got_gaze)

        # send movement commands
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # store data
        self.person_x = None
        self.person_y = None
        self.circle_spots = []
        self.gaze_yaw = 0.0  # looking left/right
        self.gaze_pitch = 0.0  # looking up/down

        # for moving
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        print("Smart driver ready - using gaze to decide approach angle!")

    def got_gaze(self, msg):
        # save which way they're looking
        if len(msg.data) >= 3:
            self.gaze_yaw = msg.data[0]  # left/right
            self.gaze_pitch = msg.data[1]  # up/down
            # roll is msg.data[2] but we don't use it much

    def person_found(self, msg):
        self.person_x = msg.point.x
        self.person_y = msg.point.y
        print(f"Person at: ({self.person_x:.2f}, {self.person_y:.2f})")

        rospy.sleep(0.5)
        best_spot = self.pick_best_approach_spot()

        if best_spot:
            self.drive_to_spot(best_spot)

    def got_circle_points(self, msg):
        self.circle_spots = []
        for marker in msg.markers:
            self.circle_spots.append((marker.pose.position.x, marker.pose.position.y))
        print(f"Got {len(self.circle_spots)} approach spots")

    def pick_best_approach_spot(self):
        # THIS IS THE COOL PART - using gaze to pick approach angle
        # from the paper: "robot determines user's gaze direction and selects most appropriate point"

        if not self.circle_spots or self.person_x is None:
            return None

        # the person is looking in direction of gaze_yaw
        # we want to approach from where they're looking (so they see us coming)
        # angle that the person is facing (0 = forward/positive y direction)
        facing_angle = math.radians(self.gaze_yaw)  # convert to radians

        # we want to approach from the direction they're looking
        # so angle from person to robot should be facing_angle + pi (opposite)
        desired_angle = facing_angle + math.pi

        best_spot = None
        best_score = -999999

        for (px, py) in self.circle_spots:
            # calculate angle from person to this spot
            dx = px - self.person_x
            dy = py - self.person_y
            spot_angle = math.atan2(dy, dx)

            # how close is this to our desired approach angle?
            angle_diff = abs(spot_angle - desired_angle)
            # make it wrap around correctly
            angle_diff = min(angle_diff, 2 * math.pi - angle_diff)

            # score = negative angle difference (smaller diff is better)
            score = -angle_diff

            print(f"  Spot at ({px:.2f}, {py:.2f}) - angle diff: {math.degrees(angle_diff):.1f}°")

            if score > best_score:
                best_score = score
                best_spot = (px, py)

        if best_spot:
            print(f"Choosing approach from front (angle match: {math.degrees(-best_score):.1f}°)")

        return best_spot

    def drive_to_spot(self, spot):
        print(f"Driving to ({spot[0]:.2f}, {spot[1]:.2f})")

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = spot[0]
        goal.target_pose.pose.position.y = spot[1]
        goal.target_pose.pose.orientation.w = 1.0

        self.move_client.send_goal(goal)

        # wait up to 30 seconds
        finished = self.move_client.wait_for_result(rospy.Duration(30))

        if finished and self.move_client.get_state() == 3:
            print("Made it to the person!")
        else:
            print("Something went wrong, couldn't get there")


if __name__ == '__main__':
    driver = SmartDriveToPerson()
    rospy.spin()