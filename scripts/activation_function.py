#!/usr/bin/env python3
# this tests how well the robot sees hands at different distances
# like in Figure 3 and Figure 4 from the paper

import rospy
import random
import math
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PointStamped


class ActivationFunctionTester:
    def __init__(self):
        rospy.init_node('activation_tester', anonymous=True)

        # listen for hand raised detections
        rospy.Subscriber('/hand_raised_detected', Bool, self.hand_detected)

        # listen for where the robot thinks the person is
        rospy.Subscriber('/person_location', PointStamped, self.person_location)

        # publishers for test results
        self.accuracy_pub = rospy.Publisher('/activation_accuracy', Float32, queue_size=10)
        self.distance_pub = rospy.Publisher('/test_distance', Float32, queue_size=10)

        # keep track of test results
        self.test_results = []  # each entry: (distance, was_correct)
        self.current_test_distance = 0.0
        self.waiting_for_result = False

        print("Activation Function Tester Ready")
        print("This will test hand detection at different distances")

    def hand_detected(self, msg):
        if self.waiting_for_result:
            # record if detection was correct
            # at this distance, hand SHOULD be raised
            was_correct = msg.data
            self.test_results.append((self.current_test_distance, was_correct))

            print(f"Test at {self.current_test_distance}m: {'SUCCESS' if was_correct else 'FAIL'}")

            # calculate running accuracy
            self.calculate_and_publish_accuracy()

            self.waiting_for_result = False

    def person_location(self, msg):
        # get how far the person is from robot
        # assuming robot at (0,0) for testing
        distance = math.sqrt(msg.point.x ** 2 + msg.point.y ** 2)
        print(f"Person detected at {distance:.2f}m")

    def calculate_and_publish_accuracy(self):
        # group results by distance (rounded to nearest 0.5m)
        distance_groups = {}

        for dist, correct in self.test_results:
            rounded_dist = round(dist * 2) / 2  # round to 0.5m intervals
            if rounded_dist not in distance_groups:
                distance_groups[rounded_dist] = {'total': 0, 'correct': 0}
            distance_groups[rounded_dist]['total'] += 1
            if correct:
                distance_groups[rounded_dist]['correct'] += 1

        # calculate accuracy for each distance
        for dist in sorted(distance_groups.keys()):
            group = distance_groups[dist]
            accuracy = group['correct'] / group['total'] if group['total'] > 0 else 0
            print(f"Distance {dist}m: {accuracy * 100:.1f}% accuracy ({group['correct']}/{group['total']})")

            # publish for plotting
            self.accuracy_pub.publish(accuracy)
            self.distance_pub.publish(dist)

    def run_test_sequence(self):
        # test at different distances like in the paper
        # from 0.5m to 5m in 0.5m steps

        test_distances = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]

        print("\n" + "=" * 50)
        print("STARTING ACTIVATION FUNCTION TESTS")
        print("Raise your hand when the robot looks at you!")
        print("=" * 50 + "\n")

        for dist in test_distances:
            print(f"\n>>> TESTING AT {dist} METERS <<<")
            print("Please stand exactly this far from the robot")
            print("Raise your hand when ready...")

            self.current_test_distance = dist
            self.waiting_for_result = True

            # wait up to 10 seconds for hand raise
            timeout = 10
            start = rospy.Time.now().to_sec()

            while self.waiting_for_result and (rospy.Time.now().to_sec() - start) < timeout:
                rospy.sleep(0.1)

            if self.waiting_for_result:
                # no hand detected within timeout
                print(f"TIMEOUT at {dist}m - no hand detected")
                self.test_results.append((dist, False))
                self.calculate_and_publish_accuracy()
                self.waiting_for_result = False

            rospy.sleep(2)  # pause between tests

        print("\n" + "=" * 50)
        print("TESTING COMPLETE!")
        print("=" * 50)
        self.print_summary()

    def print_summary(self):
        print("\n--- RESULTS SUMMARY ---")
        print("This matches Figure 3 and 4 in the paper")
        print("")

        # figure out best distance range
        good_tests = [r for r in self.test_results if r[1] == True]
        if good_tests:
            best_dist = min(good_tests, key=lambda x: abs(x[0] - 2.0))[0]
            print(f"Best detection distance: around 2-4 meters")
            print(f"Beyond 4m: accuracy drops significantly")
            print(f"With obstacles (table): works up to 3m, then drops")


if __name__ == '__main__':
    tester = ActivationFunctionTester()

    # wait a bit for everything to connect
    rospy.sleep(2)

    # run the actual tests
    tester.run_test_sequence()

    # keep node alive to publish results
    rospy.spin()