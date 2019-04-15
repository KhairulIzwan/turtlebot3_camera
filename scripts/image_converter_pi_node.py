#!/usr/bin/env python

""" UPDATED VERSION OF image_converter_node.py """

from __future__ import print_function
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

class image_converter_node:
    def __init__(self):

        """  Initializing your ROS Node """
        rospy.init_node('image_converter_node', anonymous=True)

        """ Give the OpenCV display window a name """
        self.cv_window_name = "Test Vision Node (Pi-Cam)"

        """ Create the cv_bridge object """
        self.bridge = CvBridge()

        """ Subscribe to the raw camera image topic """
        self.imgRaw_sub = rospy.Subscriber("/camPi/image_raw", Image, self.callback)

        """ Subscribe to the camera info topic """
        self.imgRaw_sub = rospy.Subscriber("/camPi/camera_info", CameraInfo, self.getCameraInfo)

        """ Publish as the opencv image topic """
        self.imgCV_pub = rospy.Publisher("/pi_opencv_img", Image, queue_size=10)

    def callback(self,data):
        """ Convert the raw image to OpenCV format """
        self.cvtImage(data)

        """ Overlay some text onto the image display """
        self.textInfo()

        """ Publish converted image """
        self.publishImg()

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
            self.cv_image_copy = self.cv_image.copy()

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
    vn = image_converter_node()

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
