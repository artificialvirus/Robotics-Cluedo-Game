#!/usr/bin/env python
from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
class colourIdentifier():

    def __init__(self):
        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()

        # We covered which topic to subscribe to should you wish to receive image data
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw",Image, self.callback)


    def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # Set the upper and lower bounds for the two colours you wish to identify
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        hsv_blue_lower = np.array([110 - self.sensitivity,50, 50])
        hsv_blue_upper = np.array([120 + self.sensitivity,255, 255])
        # Convert the rgb image into a hsv image [5, 5, 50], [25, 25, 145]
        Hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Filter out everything but particular colours using the cv2.inRange() method
        # Do this for each colour
        mask = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)
        # produce an output that a human would find useful
        human = cv2.bitwise_and(Hsv_image, Hsv_image, mask=mask)
        # To combine the masks you should use the cv2.bitwise_or() method
        # You can only bitwise_or two images at once, so multiple calls are necessary for more than two colours
        mask2 = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)
        combined = cv2.bitwise_or(mask, mask2)
        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to...
        #bitwise and an image with itself and pass the mask to the mask parameter (rgb_image,rgb_image, mask=mask)
        # As opposed to performing a bitwise_and on the mask and the image.



        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        cv2.imshow("Image window", cv_image)
        cv2.imshow("HSV_Image window", Hsv_image)
        cv2.imshow("Mask2",mask2)
        cv2.imshow("Human",human)
        cv2.imshow("MaskC",combined)
        cv2.imshow("Mask",mask)


        cv2.waitKey(3)
# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
    # Instantiate your class
    # And rospy.init the entire node
    cI = colourIdentifier()
    # Ensure that the node continues running with rospy.spin()
    # You may need to wrap rospy.spin() in an exception handler in case of KeyboardInterrupts
    rospy.init_node('colourIdentifier', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

    # Your code should go here. You can break your code into several files and
    # include them in this file. You just need to make sure that your solution
    # can run by just running rosrun group_project main.py
