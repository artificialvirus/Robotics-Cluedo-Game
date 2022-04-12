#!/usr/bin/env python
from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

import os.path
path = os.path.expanduser("~/catkin_ws/src/group_project/world/input_points.yaml")
import yaml
with open(path,"r") as stream:
    points = yaml.safe_load(stream)



class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)

	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	self.move_base.wait_for_server()

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(60))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

class colourIdentifier():

    def __init__(self):
        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()

        # We covered which topic to subscribe to should you wish to receive image data
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw",Image, self.callback)

        # Initialise flags
        self.green_circle_flag = False

    def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # Set the upper and lower bounds for the two colours you wish to identify
        hsv_green_lower = np.array([55 - self.sensitivity, 100, 0])
        hsv_green_upper = np.array([65 + self.sensitivity, 255, 255])

        hsv_red_lower1 = np.array([10 - self.sensitivity,0,0])
        hsv_red_lower2 = np.array([175 - self.sensitivity,160,0])

        hsv_red_upper1 = np.array([40 + self.sensitivity,255,255])
        hsv_red_upper2 = np.array([170 + self.sensitivity,255,255])

        # Convert the rgb image into a hsv image [5, 5, 50], [25, 25, 145]
        Hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_mask_red = cv2.inRange(cv_image, hsv_red_lower1, hsv_red_upper1)
        upper_mask_red = cv2.inRange(cv_image, hsv_red_lower2, hsv_red_upper2)
        filter1 = lower_mask_red + upper_mask_red

        filter2 = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)

        # Filter out everything but particular colours using the cv2.inRange() method
        # Do this for each colour
        mask = cv2.bitwise_or(filter1,filter2)
        output = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        contours, heirachical = cv2.findContours(filter2 ,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
            biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
            M = cv2.moments(biggest_contour)
            cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            if cv2.contourArea(biggest_contour) > 5 :
                self.green_circle_flag = True

        cv2.namedWindow('camera_Feed')
        cv2.imshow('camera_Feed', output)
        cv2.waitKey(3)
# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
    # Instantiate your class
    # And rospy.init the entire node
    # cI = colourIdentifier()
    # Ensure that the node continues running with rospy.spin()
    # You may need to wrap rospy.spin() in an exception handler in case of KeyboardInterrupts
    # rospy.init_node('colourIdentifier', anonymous=True)
    #-1.43, 1.15

    # Flag for navigation choices (conditionals)
    try:
        # Go to room 1 entrance
        rospy.init_node('nav_test', anonymous=True)
        navigator = GoToPose()
        cI = colourIdentifier()
        pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        rate = rospy.Rate(5)
        spin = Twist()
        spin.angular.z = 0.8



        x = points['room1_entrance_xy'][0]
        y = points['room1_entrance_xy'][1]
        theta = 0 # SPECIFY THETA (ROTATION) HERE
        position = {'x': x, 'y' : y}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("reached room 1  enterance")
        else:
            rospy.loginfo("The base failed to reach room 1  enterance")

        # Enter this room if green circle...
        if cI.green_circle_flag:
            x = points['room1_centre_xy'][0]
            y = points['room1_centre_xy'][1]
            theta = 0 # SPECIFY THETA (ROTATION) HERE
            position = {'x': x, 'y' : y}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
            success = navigator.goto(position, quaternion)

            if success:
                rospy.loginfo("Reached the room 1 centre")
            else:
                rospy.loginfo("The base failed to reach room 1 centre")

        # Else go to other enterance
        else:
            x = points['room2_entrance_xy'][0]
            y = points['room2_entrance_xy'][1]
            theta = 0 # SPECIFY THETA (ROTATION) HERE
            position = {'x': x, 'y' : y}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
            success = navigator.goto(position, quaternion)
            for i in range(10):
                pub.publish(spin)
                rate.sleep()

            if success:
                rospy.loginfo("Reached room 2 enterance")
            else:
                rospy.loginfo("The base failed to reach room 2 enterance")

            # Enter this room if green circle...
            if cI.green_circle_flag:
                x = points['room2_centre_xy'][0]
                y = points['room2_centre_xy'][1]
                theta = 0 # SPECIFY THETA (ROTATION) HERE
                position = {'x': x, 'y' : y}
                quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : np.sin(theta/2.0), 'r4' : np.cos(theta/2.0)}

                rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
                success = navigator.goto(position, quaternion)

                if success:
                    rospy.loginfo("Reached room 2 centre")
                else:
                    rospy.loginfo("The base failed to reach room 2 centre")
        # rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


    # Your code should go here. You can break your code into several files and
    # include them in this file. You just need to make sure that your solution
    # can run by just running rosrun group_project main.py
