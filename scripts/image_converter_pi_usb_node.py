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

class image_converter_node:
    def __init__(self, gamma, alpha, beta):
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

        """  Initializing your ROS Node """
        rospy.init_node('image_converter_node', anonymous=True)

        rospy.on_shutdown(self.shutdown)

        """ Give the OpenCV display window a name """
        self.cv_window_name = "Test Vision Node (Pi-USB)"

        """ Create the cv_bridge object """
        self.bridge = CvBridge()

        """ Subscribe to the raw camera image topic """
        self.imgRaw_sub = rospy.Subscriber("/camPi_USB/image_raw", Image, self.callback)

        """ Subscribe to the camera info topic """
        self.imgRaw_sub = rospy.Subscriber("/camPi_USB/camera_info", CameraInfo, self.getCameraInfo)

        """ Publish as the opencv image topic """
        self.imgCV_pub = rospy.Publisher("/pi_usb_opencv_img", Image, queue_size=10)

    def callback(self,data):
        """ Convert the raw image to OpenCV format """
        self.cvtImage(data)

        """ Overlay some text onto the image display """
        self.textInfo()

        """ Refresh the image on the screen """
        self.displayImg()

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

            self.adjust_gamma()
            self.sharpImg()

            """ OTIONAL -- image-rotate """
            # self.cv_image = imutils.rotate(self.cv_image, angle=180)
            self.cv_image_copy = self.cv_image.copy()

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

    """ coverting the uint8 OpenCV image to ROS image data """
    """ Publisher.publish() -- explicit way """
    def publishImg(self):
        try:
            self.imgCV_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image_copy, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def shutdown(self):
        try:
            rospy.loginfo("Image converter node [OFFLINE]...")

        finally:
            cv2.destroyAllWindows()
            
def usage():
    print("%s" % sys.argv[0])

def main(args):
    vn = image_converter_node(sys.argv[1], sys.argv[2], sys.argv[3])

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Image converter node [OFFLINE]...")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    if len(sys.argv) < 1:
        print(usage())
        sys.exit(1)
    else:
        print("Image converter node [ONLINE]...")
        main(sys.argv)
