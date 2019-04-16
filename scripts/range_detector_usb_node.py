#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import roslib
roslib.load_manifest('turtlebot3_camera')

import sys
import rospy
import os
import cv2
import imutils

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import CameraInfo

from camera_tutorials.msg import IntList

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import numpy as np

class range_detector_node:
    def __init__(self, filter, gamma, alpha, beta):
        self.filter = filter
        self.gamma = float(gamma)
        self.alpha = float(alpha)
        self.beta = float(beta)

        if self.gamma > 0:
            self.gamma = float(self.gamma / 100)
        else:
            self.gamma = 0.01

        # TODO:
        # if self.ksize <= 3:
        #     self.ksize = 3
        # else:
        #     self.ksize = self.ksize

        if self.alpha > 0:
            self.alpha = float(self.alpha / 100)
        else:
            self.alpha = 0.01

        if self.beta > 0:
            self.beta = float(self.beta / 100)
        else:
            self.beta = 0.01

        """ Initializing your ROS Node """
        rospy.init_node('range_detector_node', anonymous=True)

        """ Give the OpenCV display window a name """
        self.cv_window_name = "Range Detector Node"

        """ Create the cv_bridge object """
        self.bridge = CvBridge()

        """ Subscribe to the raw camera image topic """
        # self.imgRaw_sub = rospy.Subscriber("/camPi/image_raw", Image, self.callback)
        self.imgRaw_sub = rospy.Subscriber("/camUSB/image_raw", Image, self.callback)

        """ Publish as color range topic """
        self.rangeColor_pub = rospy.Publisher("/color_range", IntList, queue_size=10)

    def callback(self, data):
        """ Convert the raw image to OpenCV format """
        self.cvtImage(data)

        rospy.loginfo(self.gamma)

        """ Determine the range_filter using the setup_trackbars() helper function """
        self.setup_trackbars()

        """ Convert the image colorspace """
        self.cvtColorspace()

        """ Extract the require color value """
        self.get_trackbar_values()

        """ Publish Data """
        self.pubData()

        """ Threshold the image """
        self.imgThresh()

        """ Refresh the image on the screen """
        self.displayImg()

    """ Convert the raw image to OpenCV format """
    def cvtImage(self, data):
        try:
            """ Convert the raw image to OpenCV format """
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.adjust_gamma()
            self.sharpImg()

        except CvBridgeError as e:
            print(e)

    def adjust_gamma(self):
        """ build a lookup table mapping the pixel values [0, 255] to their adjusted gamma values """
        self.invGamma = 1.0 / self.gamma

        self.table = np.array([((i / 255.0) ** self.invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")

        """ apply gamma correction using the lookup table """
        self.adjusted = cv2.LUT(self.cv_image, self.table)

    def sharpImg(self):
        """ apply guassian blur on src image """
        self.blurred = cv2.GaussianBlur(self.adjusted, (5, 5), cv2.BORDER_DEFAULT)
        self.sharpen = cv2.addWeighted(self.adjusted, self.alpha, self.blurred, -self.beta, self.gamma)

        """ copying sharpen image"""
        self.cv_image = self.sharpen

    def setup_trackbars(self):
        self.cv_window_trackbar = "Trackbars"
        cv2.namedWindow(self.cv_window_trackbar, 0)

        for i in ["MIN", "MAX"]:
            v = 0 if i == "MIN" else 255

            for j in self.filter.upper():
                cv2.createTrackbar("%s_%s" % (j, i), self.cv_window_trackbar, v, 255, self.callback_trackbars)

    def callback_trackbars(self, value):
        pass

    def get_trackbar_values(self):
        self.values = []

        for i in ["MIN", "MAX"]:
            for j in self.filter.upper():
                v = cv2.getTrackbarPos("%s_%s" % (j, i), self.cv_window_trackbar)
                self.values.append(v)

        # return values
        rospy.loginfo(self.values)

    """ Convert the image colorspace """
    def cvtColorspace(self):
        self.frame_to_thresh = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

    """ Refresh the image on the screen """
    def displayImg(self):
        cv2.imshow(self.cv_window_name, self.cv_image)
        cv2.waitKey(1)

    def imgThresh(self):
        self.thresh = cv2.inRange(self.frame_to_thresh, (self.values[0], self.values[1], self.values[2]), (self.values[3], self.values[4], self.values[5]))
        self.cv_image = cv2.bitwise_and(self.cv_image, self.cv_image, mask=self.thresh)

    def pubData(self):
        msg = IntList()
        msg.v1_min = self.values[0]
        msg.v2_min = self.values[1]
        msg.v3_min = self.values[2]
        msg.v1_max = self.values[3]
        msg.v2_max = self.values[4]
        msg.v3_max = self.values[5]
        self.rangeColor_pub.publish(msg)

def usage():
    print("Please specify type of color filter:")
    print("1. RGB")
    print("2. HSV")
    print("%s [COLOR_FILTER]" % sys.argv[0])

def main(args):
    vn = range_detector_node(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Range detector node [OFFLINE]...")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    if len(sys.argv) < 1:
        print(usage())
        sys.exit(1)
    else:
        print("Range detector node [ONLINE]...")
        main(sys.argv)
