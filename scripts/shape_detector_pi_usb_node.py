#!/usr/bin/env python

from __future__ import print_function
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

class shape_detector_node:
    def __init__(self):

        """ Initializing your ROS Node """
        rospy.init_node('shape_detector_node', anonymous=True)

        """ Give the OpenCV display window a name """
        self.cv_window_name = "Shape Detector Node"
        self.cv_window_thresh = "Shape Detector Node (THRESH)"

        """ Create the cv_bridge object """
        self.bridge = CvBridge()

        """ Subscribe to the raw camera image topic """
        self.imgRaw_sub = rospy.Subscriber("/camPi_USB/image_raw", Image, self.callback)

        """ Subscribe to the camera info topic """
        self.imgRaw_sub = rospy.Subscriber("/camPi_USB/camera_info", CameraInfo, self.getCameraInfo)

    def callback(self, data):
        """ Convert the raw image to OpenCV format """
        self.cvtImage(data)

        """ load the image and resize it to a smaller factor so that the shapes can be approximated better """
        self.imgProcessing()

        # self.indentifyContour()

        """ Refresh the image on the screen """
        self.displayImg()

    """ Getting camera info -- width, height, and etc """
    def getCameraInfo(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height

    """ Convert the raw image to OpenCV format """
    def cvtImage(self, data):
        try:
            """ Convert the raw image to OpenCV format """
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            """ OTIONAL -- image-rotate """
            # self.cv_image = imutils.rotate(self.cv_image, angle=180)

        except CvBridgeError as e:
            print(e)

    """ load the image and resize it to a smaller factor so that the shapes can be approximated better """
    def imgProcessing(self):
        """ load the image and resize it to a smaller factor so that the shapes can be approximated better """
        self.resized = imutils.resize(self.cv_image, width=300)
        self.ratio = self.image_height / float(self.resized.shape[0])
        rospy.loginfo(self.ratio)

        """ convert the resized image to grayscale, blur it slightly, and threshold it """
        self.gray = cv2.cvtColor(self.resized, cv2.COLOR_BGR2GRAY)
        self.blurred = cv2.GaussianBlur(self.gray, (5, 5), 0)

        # find contours of a binary image
        self.thresh = cv2.threshold(self.blurred, 127, 255, cv2.THRESH_BINARY)[1]

        # self.frame_to_thresh = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        # self.thresh = cv2.inRange(self.frame_to_thresh, (29, 115, 84), (45, 187, 155))
        # self.cv_image = cv2.bitwise_not(self.cv_image, self.cv_image, mask=self.thresh)

        """ find contours in the thresholded image and initialize the shape detector """
        # self.cnts = cv2.findContours(self.thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # to save memory; else cv2.CHAIN_APPROX_NONE
        # self.cnts = imutils.grab_contours(self.cnts)

    """ identify each of the contours """
    def indentifyContour(self):
        # loop over the contours
        for self.c in self.cnts:
            # compute the center of the contour, then detect the name of the shape using only the contour
            self.M = cv2.moments(self.c)
            self.cX = int((self.M["m10"] / self.M["m00"]) * self.ratio)
            self.cY = int((self.M["m01"] / self.M["m00"]) * self.ratio)

            self.detect()

            # multiply the contour (x, y)-coordinates by the resize ratio, then draw the contours and the name of the shape on the image
            self.c = self.c.astype("float")
            self.c *= self.ratio
            self.c = self.c.astype("int")
            cv2.drawContours(self.cv_image, [self.c], -1, (0, 255, 0), 2) # draw all contours found
            cv2.putText(self.cv_image, self.shape, (self.cX, self.cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    def detect(self):
        """ initialize the shape name and approximate the contour """
        self.shape = "unidentified"
        self.peri = cv2.arcLength(self.c, True) # calculates a contour perimeter or a curve length.
        self.approx = cv2.approxPolyDP(self.c, 0.04 * self.peri, True)  # approximates a polygonal curve(s) with the specified precision

        """ if the shape is a triangle, it will have 3 vertices """
        if len(self.approx) == 3:
            self.shape = "triangle"

        # """ if the shape has 4 vertices, it is either a square or a rectangle """
        elif len(self.approx) == 4:
            """ compute the bounding box of the contour and use the bounding box to compute the aspect ratio """
            (self.x, self.y, self.w, self.h) = cv2.boundingRect(self.approx)
            self.ar = self.w / float(self.h)

            """ a square will have an aspect ratio that is approximately equal to one, otherwise, the shape is a rectangle """
            self.shape = "square" if self.ar >= 0.95 and self.ar <= 1.05 else "rectangle"

        # """ if the shape is a pentagon, it will have 5 vertices """
        elif len(self.approx) == 5:
            self.shape = "pentagon"

        # """ otherwise, we assume the shape is a circle """
        else:
            self.shape = "circle"

    """ Refresh the image on the screen """
    def displayImg(self):
        cv2.imshow(self.cv_window_name, self.cv_image)
        cv2.imshow(self.cv_window_thresh, self.thresh)
        cv2.waitKey(1)

def usage():
    print("%s" % sys.argv[0])

def main(args):
    vn = shape_detector_node()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shape detector node [OFFLINE]...")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    if len(sys.argv) == 1:
        print("Shape detector node [ONLINE]...")
        main(sys.argv)
    else:
        print(usage())
        sys.exit(1)
