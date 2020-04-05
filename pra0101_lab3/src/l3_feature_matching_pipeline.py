#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("/output/image", Image, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image", Image, self.callback)
        self.first_image = True
        self.kp_cur = None
        self.kp_next = None
        self.des_cur = None
        self.des_next = None
        self.img_cur = None
        self.img_next = None
        self.img_harris_corner = None
        self.harris_kp = None

    def callback(self, data):
        try:
            # TODO convert the ROS image to OpenCV format
            self.img_next =
        except CvBridgeError as e:
            print(e)

        # Find the keypoints with harris corner detector
        # Convert to grayscale
        gray =

        # Extract features
        self.harris_kp =

        # Dilate the result
        dst =

        # Threshold for an optimal value, it may vary depending on the image.
        self.img_harris_corner = self.img_next.copy()
        self.img_harris_corner[dst > 0.01 * dst.max()] = [0, 0, 255]

        cv2.imshow('Harris Corner Detection', self.img_harris_corner)
        cv2.waitKey(3)

        # compute feature descriptors
        # Initiate ORB detector
        self.kp_next, self.des_next =

        # visualize the features
        img_next_feat =

        cv2.imshow("Monocular image with features", img_next_feat)
        cv2.waitKey(3)

        # create BFMatcher object

        # Match descriptors.
        if not self.first_image:
            matches =

            # Sort them in the order of their distance.
            matches =

            # Specify number of matches to draw and visualize it
            nmatch2draw = 10
            img_m =

            cv2.imshow("Feature Matching Window", img_m)
            cv2.waitKey(3)

        # Store old image and keypoints etc.
        self.kp_cur = self.kp_next
        self.des_cur = self.des_next
        self.img_cur = self.img_next.copy()
        self.first_image = False

        cv2.imshow("New Image Window", self.img_next)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.img_next, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
