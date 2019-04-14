#!/usr/bin/env python

""" UPDATED VERSION OF test_vision_node.py """

from __future__ import print_function
import roslib
roslib.load_manifest('camera_tutorials')

import sys
import rospy
import cv2
import imutils

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

class test_vision_node:
    def __init__(self):

        """  Initializing your ROS Node """
        rospy.init_node('test_vision_node', anonymous=True)

        """ Give the OpenCV display window a name """
        self.cv_window_name = "Test Vision Node"

        """ Create the cv_bridge object """
        self.bridge = CvBridge()

        """ Subscribe to the raw camera image topic """
        self.imgRaw_sub = rospy.Subscriber("/camPi/image_raw", Image, self.callback)

        """ Subscribe to the camera info topic """
        self.imgRaw_sub = rospy.Subscriber("/camPi/camera_info", CameraInfo, self.getCameraInfo)

    def callback(self,data):
        """ Convert the raw image to OpenCV format """
        self.cvtImage(data)

        """ Overlay some text onto the image display """
        self.textInfo()

        """ Refresh the image on the screen """
        self.displayImg()

    """ Get the width and height of the image """
    def getCameraInfo(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height

    """ Convert the raw image to OpenCV format """
    def cvtImage(self, data):
        try:
            """ Convert the raw image to OpenCV format """
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            """ OTIONAL -- image-rotate """
            self.cv_image = imutils.rotate(self.cv_image, angle=180)

        except CvBridgeError as e:
            print(e)

    """ Overlay some text onto the image display """
    def textInfo(self):
        img = self.cv_image

        text = "Sample"
        org = (10, self.image_height - 10)

        fontFace = cv2.FONT_HERSHEY_DUPLEX
        fontScale = 0.5
        color = (255, 255, 255)
        thickness = 1
        lineType = cv2.LINE_AA
        bottomLeftOrigin = False # if True (text upside down)

        text1 = "(%d, %d)" % (self.image_width, self.image_height)
        org1 = (self.image_width - 100, self.image_height - 10)

        cv2.putText(img, text, org, fontFace, fontScale, color, thickness, lineType, bottomLeftOrigin)
        cv2.putText(img, text1, org1, fontFace, fontScale, color, thickness, lineType, bottomLeftOrigin)

    """ Refresh the image on the screen """
    def displayImg(self):
        cv2.imshow(self.cv_window_name, self.cv_image)
        cv2.waitKey(1)

def usage():
    print("%s" % sys.argv[0])

def main(args):
    vn = test_vision_node()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Test vision node [OFFLINE]...")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    if len(sys.argv) == 1:
        print("Test vision node [ONLINE]...")
        main(sys.argv)
    else:
        print(usage())
        sys.exit(1)
