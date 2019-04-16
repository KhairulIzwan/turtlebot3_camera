#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
import roslib
roslib.load_manifest('turtlebot3_camera')

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

import numpy as np

class gammaCalibrate_node:
    def __init__(self):

        """  Initializing your ROS Node """
        rospy.init_node('gammaCalibrate_node', anonymous=True)

        """ Give the OpenCV display window a name """
        self.cv_window_name = "Test Vision Node"

        """ Create the cv_bridge object """
        self.bridge = CvBridge()

        """ Subscribe to the raw camera image topic """
        self.imgRaw_sub = rospy.Subscriber("/camUSB/image_raw", Image, self.callback)

        """ Subscribe to the camera info topic """
        self.imgRaw_sub = rospy.Subscriber("/camUSB/camera_info", CameraInfo, self.getCameraInfo)

        """ Publish as the opencv image topic """
        self.imgCV_pub = rospy.Publisher("/usb_opencv_img", Image, queue_size=10)

    def callback(self,data):
        """ Convert the raw image to OpenCV format """
        self.cvtImage(data)

        """
        ADDITIONAL:
        """
        self.setup_trackbars()

        self.get_trackbar_values()

        self.adjust_gamma()

        """ Overlay some text onto the image display """
        self.textInfo()

        """ Refresh the image on the screen """
        self.displayImg()

        """ Publish converted Image """
        self.publishImg()

    """ Get the width and height of the image """
    def getCameraInfo(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height

    """ Convert the raw image to OpenCV format """
    def cvtImage(self, data):
        try:
            """ Convert the raw image to OpenCV format """
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            """ Duplicate orginal image """
            self.cv_image_copy = self.cv_image.copy()

        except CvBridgeError as e:
            print(e)

    def setup_trackbars(self):
        self.cv_window_trackbar = "Trackbars"
        cv2.namedWindow(self.cv_window_trackbar, 0)
        cv2.createTrackbar("Gamma", self.cv_window_trackbar, 0, 100, self.callback_trackbars)

    def callback_trackbars(self, value):
        pass

    def get_trackbar_values(self):
        self.gamma = cv2.getTrackbarPos("Gamma", self.cv_window_trackbar)

        if self.gamma > 0:
            self.gamma = float(self.gamma / 100)
        else:
            self.gamma = 0.01

    def adjust_gamma(self):
        """ build a lookup table mapping the pixel values [0, 255] to their adjusted gamma values """
        self.invGamma = 1.0 / self.gamma

        self.table = np.array([((i / 255.0) ** self.invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")

        """ apply gamma correction using the lookup table """
        self.adjusted = cv2.LUT(self.cv_image, self.table)

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
        cv2.imshow(self.cv_window_name, np.hstack((self.cv_image, self.adjusted)))
        cv2.waitKey(1)

    """ coverting the uint8 OpenCV image to ROS image data """
    """ Publisher.publish() -- explicit way """
    def publishImg(self):
        try:
            self.imgCV_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image_copy, "bgr8"))
        except CvBridgeError as e:
            print(e)

def usage():
    print("%s" % sys.argv[0])

def main(args):
    vn = gammaCalibrate_node()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Gamma Calibration node [OFFLINE]...")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    if len(sys.argv) < 1:
        print(usage())
        sys.exit(1)
    else:
        print("Gamma Calibration node [ONLINE]...")
        main(sys.argv)
