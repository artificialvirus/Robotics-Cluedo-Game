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

from collections import namedtuple, Counter


flann_index_kdtr = 1
flann_index_lsh = 6

min_mtchs = 10

flann_prms = dict(algorithm = flann_index_lsh,
                   table_number = 6,
                   key_size = 12,
                   multi_probe_level = 1)



Template = namedtuple('Template', 'image, name, keypoints, descriptors')

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


#using flann based object detection (opencv feature detection)
#template matching
#Optionally we can try to implement yolo based detection (model traiining)
#Yolov3 or yolov5 (coco dataset (custom))
class ObjectDetection():

    def __init__(self, camera=False):
        self.bridge = CvBridge()
        self.orb = cv2.ORB(nfeatures = 1000)
        self.flann = cv2.FlannBasedMatcher(flann_prms, {})


        self.tmplts = []
        self.templates()
        self.take_picture = False
        self.detected_character = False


        if camera:
            self.subscriber = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)


    def callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
          
            if self.detected_character == True:
                cv2.imwrite(os.path.expanduser('~/catkin_ws/src/group_project/') +'cv_image.png', cv_image)
            else:
                print('character not found')


            results =  self.compareFtr(cv_image)
            if results:
                counter = Counter(results)
                print(counter)

            else:

                print("Not able to detect an image.")

            self.drawBoundaries()
        except CvBridgeError as e:
            print(e)



        self.imgIdentify(cv_image)



        cv2.namedWindow('CameraFeed')
        cv2.imshow('CameraFeed', cv_image)
        cv2.waitKey(1)



    def detection(self, image):

        keypoints, descriptors = self.orb.detectAndCompute(image, None)
        if descriptors is None:
            descriptors = []
        return keypoints, descriptors


    def templates(self):

        tmplts = {'mustard' : 'mustard.png',

                    'scarlet' : 'scarlet.png',

                     'peacock' : 'peacock.png',

                     'plum' : 'plum.png'}

        for name, filename in tmplts.iteritems():

            image = cv2.imread(os.path.expanduser('~/catkin_ws/src/group_project/cluedo_images/') + filename)

            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            self.tmplt(image.copy(), name)


    def tmplt(self, image, name):

        keypoints, descriptors = self.detection(image)
        descriptors = np.uint8(descriptors)
        self.flann.add([descriptors])


        template = Template(image=image, name=name, keypoints=keypoints, descriptors=descriptors)
        self.tmplts.append(template)




    def compareFtr(self, image):

        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self.detection(image)


        if len(keypoints) < min_mtchs:
            return []


        matches = self.flann.knnMatch(descriptors, k = 2)


        matches = [m[0] for m in matches if len(m) == 2 and m[0].distance < m[1].distance * 0.75]


        if len(matches) < min_mtchs:
            return []


        template_matches = [[] for _ in xrange(len(self.tmplts))]
        for match in matches:

            template_matches[match.imgIdx].append(match)


        detections = []


        for imgIdx, matches in enumerate(template_matches):
            if len(matches) < min_mtchs:

                continue

            template = self.tmplts[imgIdx]


            template_matches = [template.keypoints[m.trainIdx].pt for m in matches]
            image_matches = [keypoints[m.queryIdx].pt for m in matches]
            template_matches, image_matches = np.float32((template_matches, image_matches))


            homography, status = cv2.findHomography(template_matches, image_matches, cv2.RANSAC, 3.0)
            status = status.ravel() != 0


            if status.sum() < min_mtchs:
                continue

            detections.append(template.name)
        return detections

    def imgIdentify(self, image):


        detected = []
        for i in range(0, 2):
            detected = detected + self.compareFtr(image)

        if detected:
            self.detected_character = True

            counter = Counter(detected)

            file_path = os.path.expanduser('~/catkin_ws/src/group_project/')
            with open(file_path + 'cluedo_character.txt', 'w') as txt_file:
                txt_file.write("Detected: " + counter.most_common(1)[0][0])


        else:
            print("Can not find.")




    def drawBoundaries(self):

        tmplts = {'mustard' : 'mustard.png',

                    'scarlet' : 'scarlet.png',

                     'peacock' : 'peacock.png',

                     'plum' : 'plum.png'}

        for name, filename in tmplts.iteritems():
            image = cv2.imread(os.path.expanduser('~/catkin_ws/src/group_project/cluedo_images/') + filename)


        image_to_gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)


        image_to_gray = cv2.GaussianBlur(image_to_gray,(11,11),0)


        edge = cv2.Canny(image_to_gray,100,200)


        contours, hierarchy = cv2.findContours(edge.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)


        cv2.drawContours(image,contours,-1,[0,255,0],2)


        #cv2.imwrite('/home/csunix/sc19ao/catkin_ws/src/group_project/'+ 'found_character.png', image)







class colourIdentifier():

    def __init__(self):
        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()

        # We covered which topic to subscribe to should you wish to receive image data
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw",Image, self.callback)


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
            cx, cy = int(M['m10']/(M['m00']+1e-5)), int(M['m01']/(M['m00']+1e-5))
            if cv2.contourArea(biggest_contour) > 5 :
                self.green_circle_flag = True


        cv2.namedWindow('camera_Feed')
        cv2.imshow('camera_Feed', output)
        cv2.waitKey(3)
# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such

class rectangleIdentifier():

    def __init__(self):
        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()

        # We covered which topic to subscribe to should you wish to receive image data
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw",Image, self.callback)


        self.found_rectangle = False
        self.x = 0
        self.y = 0
        self.contours = []

    def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # Set the upper and lower bounds for the two colours you wish to identify
        # hsv_green_lower = np.array([55 - self.sensitivity, 100, 0])
        # hsv_green_upper = np.array([65 + self.sensitivity, 255, 255])

        hsv_yellow_lower = np.array([35-self.sensitivity,50,100])
        hsv_red_lower1 = np.array([10 - self.sensitivity,50,100])

        hsv_blue_lower = np.array([100 - self.sensitivity,30,100])
        hsv_purple_lower = np.array([140 - self.sensitivity,30,15])
                # hsv_red_lower2 = np.array([175 - self.sensitivity,160,0])
        hsv_purple_upper = np.array([150+self.sensitivity,255,255])

        hsv_blue_upper = np.array([100+self.sensitivity,255,255])

        hsv_yellow_upper = np.array([20+self.sensitivity,255,255])

        hsv_red_upper1 = np.array([-5 + self.sensitivity,255,255])
        # hsv_red_upper2 = np.array([170 + self.sensitivity,255,255])

        # Convert the rgb image into a hsv image [5, 5, 50], [25, 25, 145]
        Hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # lower_mask_red = cv2.inRange(cv_image, hsv_red_lower1, hsv_red_upper1)
        # upper_mask_red = cv2.inRange(cv_image, hsv_red_lower2, hsv_red_upper2)
        # filter1 = lower_mask_red + upper_mask_red
        filter1 = cv2.inRange(Hsv_image, hsv_red_lower1, hsv_red_upper1)
        # filter2 = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)
        filter3 = cv2.inRange(Hsv_image, hsv_yellow_lower, hsv_yellow_upper)
        filter4 = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)
        filter5 = cv2.inRange(Hsv_image, hsv_purple_lower, hsv_purple_upper)
        # Filter out everything but particular colours using the cv2.inRange() method
        # Do this for each colour
        # mask = cv2.bitwise_or(filter1,filter2)
        mask = cv2.bitwise_or(filter1, filter3)
        mask = cv2.bitwise_or(mask, filter4)
        mask = cv2.bitwise_or(mask, filter5)
    
        output = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        contours, heirachical = cv2.findContours(mask ,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
            biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
            M = cv2.moments(biggest_contour)
            cx, cy = int(M['m10']/(M['m00']+1e-5)), int(M['m01']/(M['m00']+1e-5))
            if cv2.contourArea(biggest_contour) > 5 :
                self.found_rectangle = True
                self.contour = biggest_contour
            else:
                self.found_rectangle = False
        cv2.namedWindow('camera_Feed2')
        cv2.imshow('camera_Feed2', output)
        cv2.waitKey(3)
def find_coordinates(contours):
    xy = []
    for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.009 * cv2.arcLength(cnt, True), True)
            n = approx.ravel()
            i = 0
            for j in n :
                if(i % 2 == 0):
                    x = n[i]
                    y = n[i + 1]
    
                    # String containing the co-ordinates.
                    xy.append(x)
                    xy.append(y)
                    string = str(x) + " " + str(y) 

                i = i + 1
    return xy
def main(args):
    # Instantiate your class
    # And rospy.init the entire node
    # cI = colourIdentifier()
    # Ensure that the node continues running with rospy.spin()
    # You may need to wrap rospy.spin() in an exception handler in case of KeyboardInterrupts
    # rospy.init_node('colourIdentifier', anonymous=True)
    #-1.43, 1.15

    # Flag for navigation choices (conditionals)
    #green_circle_flag = False




    try:
        # Go to room 1 entrance

        rospy.init_node('nav_test', anonymous=True)

        navigator = GoToPose()

        cI = colourIdentifier()
        # objDetec = ObjectDetection(camera=True)


        pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        rospy.loginfo('2PLEASE NOTICE THIS NOTICE')
        rate = rospy.Rate(5)
        spin = Twist()
        spin.angular.z = 0.2
        

        desired_velocity = Twist()
        desired_velocity.linear.x = 0.3
        
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
        for i in range(156):
            pub.publish(spin)
            rate.sleep()
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
                count_false = 0
                count_true = 0
                count_false_final = 0
                already_true_flag = False
                rI = rectangleIdentifier()
                while rI.found_rectangle is True:
                    pub.publish(spin)
                    rate.sleep()
                for i in range(156):
                    if rI.found_rectangle is True:
                        count_true = count_true + 1
                        count_false_final = count_false
                        already_true_flag = True
                    else:
                        if already_true_flag is False:
                            count_false = count_false + 1
                    pub.publish(spin)
                    rate.sleep()
                    print(rI.found_rectangle)
                iter = int(count_false_final + (count_true/2-2))
                for i in range(iter):
                    pub.publish(spin)
                    rate.sleep()
                
                objDetec = ObjectDetection(camera=True)
                while cv2.contourArea(rI.contour) < 10500:
                        pub.publish(desired_velocity)
                        rate.sleep()


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
                    rospy.loginfo("Reached the room 1 centre")
                    count_false = 0
                    count_true = 0
                    count_false_final = 0
                    already_true_flag = False
                    rI = rectangleIdentifier()
                    while rI.found_rectangle is True:
                        pub.publish(spin)
                        rate.sleep()
                    for i in range(156):
                        if rI.found_rectangle is True:
                            count_true = count_true + 1
                            count_false_final = count_false
                            already_true_flag = True
                        else:
                            if already_true_flag is False:
                                count_false = count_false + 1
                        pub.publish(spin)
                        rate.sleep()
                        print(rI.found_rectangle)
                    iter = int(count_false_final + (count_true/2-2))
                    for i in range(iter):
                        pub.publish(spin)
                        rate.sleep()
                
                objDetec = ObjectDetection(camera=True)
                while cv2.contourArea(rI.contour) < 10500:
                        pub.publish(desired_velocity)
                        rate.sleep()


                else:
                    rospy.loginfo("The base failed to reach room 2 centre")

    

    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


    # Your code should go here. You can break your code into several files and
    # include them in this file. You just need to make sure that your solution
    # can run by just running rosrun group_project main.py
