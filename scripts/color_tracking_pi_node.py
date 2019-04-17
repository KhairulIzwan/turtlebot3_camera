#!/usr/bin/env python

# single color tracking (color)

from __future__ import print_function
from __future__ import division

import roslib
roslib.load_manifest('camera_tutorials')

import sys
import rospy
import cv2
import numpy as np
import imutils

from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import CameraInfo

from camera_tutorials.msg import IntList
from camera_tutorials.msg import detailROI

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

from collections import deque
"""  a list-like data structure with super fast appends and pops to maintain a list of the past N (x, y)-locations of the ball in our video stream. Maintaining such a queue allows us to draw the "contrail" of the ball as its being tracked """

import os

class color_tracking_node:
    def __init__(self, names, v1_min, v2_min, v3_min, v1_max, v2_max, v3_max):
        self.names = names

        self.v1_min = v1_min
        self.v2_min = v2_min
        self.v3_min = v3_min
        self.v1_max = v1_max
        self.v2_max = v2_max
        self.v3_max = v3_max

        """ Initializing your ROS Node """
        rospy.init_node('color_tracking_node_' + self.names, anonymous=True)

        """ Give the OpenCV display window a name """
        self.cv_window_name = self.names + " Ball"

        """ initialize the list of tracked points, the frame counter, and the coordinate deltas """
        self.pts = deque(maxlen=32)
        self.counter = 0
        (self.dX, self.dY) = (0, 0)
        self.direction = ""

        """ Give the camera driver a moment to come up """
        rospy.sleep(1)

        """ Create the cv_bridge object """
        self.bridge = CvBridge()

        """ Subscribe to the raw camera image topic """
        self.imgRaw_sub = rospy.Subscriber("/pi_opencv_img", Image, self.callback)

        """ Subscribe to the camera info topic """
        self.imgInfo_sub = rospy.Subscriber("/camUSB/camera_info", CameraInfo, self.getCameraInfo)

        # TODO: Need to replace this; read from files
        """ define the lower and upper boundaries of the colors in the HSV color space """
        self.lower = {str(self.names):(int(self.v1_min), int(self.v2_min), int(self.v3_min))}
        self.upper = {str(self.names):(int(self.v1_max), int(self.v2_max), int(self.v3_max))}

        # TODO: Need to replace this; read from files
        """ define standard colors for circle around the object """
        self.colors = {"Green":(0,255,0), "Red":(0,0,255)}

        """ Publish roi topic """
        self.imgROI_pub = rospy.Publisher("/roi_" + self.names, detailROI, queue_size=10)

    def callback(self, data):
        """ Convert the raw image to OpenCV format using the cvtImage() helper function """
        self.cvtImage(data)

        """ Apply image processing using imgProcessing() helper function """
        self.imgProcessing()

        """ Apply ball tracking using colorDetection() helper function """
        self.colorDetection()

        """ Visualize the tracking echo"""
        self.ptsTrack()

        """ Refresh the image on the screen """
        self.displayImg()

    """ Convert the raw image to OpenCV format """
    def cvtImage(self, data):
        try:
            """ Convert the raw image to OpenCV format """
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)

    """ resize the frame, blur it, and convert it to the HSV color space """
    def imgProcessing(self):
        if (self.image_width > 320):
            self.cv_image = imutils.resize(self.cv_image, width = 320)
        else:
            pass

        """ optional -- image-mirrored """
        # self.cv_image = cv2.flip(self.cv_image, 1)

        self.blurred = cv2.GaussianBlur(self.cv_image, (11, 11), 0)
        self.hsv = cv2.cvtColor(self.blurred, cv2.COLOR_BGR2HSV)

    """ Get the width and height of the image """
    def getCameraInfo(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height

    def colorDetection(self):
        self.mask = cv2.inRange(self.hsv, self.lower[self.names], self.upper[self.names])
        self.mask = cv2.erode(self.mask, None, iterations=2)
        self.mask = cv2.dilate(self.mask, None, iterations=2)

        """ find contours in the mask and initialize the current (x, y) center of the ball """
        self.cnts = cv2.findContours(self.mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        self.center = None

        """ only proceed if at least one contour was found """
        if len(self.cnts) > 0:
            """ find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid """
            self.c = max(self.cnts, key=cv2.contourArea)
            ((self.x, self.y), self.radius) = cv2.minEnclosingCircle(self.c)

            """ Image moments help you to calculate some features like center of mass of the object, area of the object etc """
            """ http://en.wikipedia.org/wiki/Image_moment """
            self.M = cv2.moments(self.c)

            """ Centroid is given by the relations, Cx=M10/M00 and Cy=M01/M00 """
            self.center = (int(self.M["m10"] / self.M["m00"]), int(self.M["m01"] / self.M["m00"]))

            """ Straight Bounding Rectangle """
            self.xBox, self.yBox, self.w, self.h = cv2.boundingRect(self.c)

            """ RegionOfInterest """
            # roi = RegionOfInterest()
            # roi.x_offset = self.xBox
            # roi.y_offset = self.yBox
            # roi.width = self.w
            # roi.height = self.h

            """ Detail RegionOfInterest """
            # roi = detailROI()
            # roi.colorName = self.names
            # roi.offsetX = self.xBox
            # roi.offsetY = self.yBox
            # roi.width = self.w
            # roi.height = self.h
            # roi.x = self.x
            # roi.y = self.y
            # roi.radius = self.radius
            #
            # self.imgROI_pub.publish(roi)

            """ only proceed if the radius meets a minimum size. Correct this value for your obect's size """
            if self.radius > 10:

                """ Detail RegionOfInterest """
                roi = detailROI()
                roi.colorName = self.names
                roi.offsetX = self.xBox
                roi.offsetY = self.yBox
                roi.width = self.w
                roi.height = self.h
                roi.x = self.x
                roi.y = self.y
                roi.radius = self.radius

                self.imgROI_pub.publish(roi)

                """ draw the circle and centroid on the frame, then update the list of tracked points """
                cv2.circle(self.cv_image, (int(self.x), int(self.y)), int(self.radius), self.colors[self.names], 2)
                cv2.circle(self.cv_image, (int(self.x), int(self.y)), 5, self.colors[self.names], -1) # center points
                # cv2.circle(self.cv_image, (int(COG_y), int(COG_x)), 5, self.colors["green"], -1) # center points
                cv2.putText(self.cv_image, self.names + " ball", (int(self.x - self.radius),int(self.y - self.radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.colors[self.names],2)

                cv2.putText(self.cv_image, self.names + " Colored Ball Detected!", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, self.colors[self.names], 1)

                # rospy.loginfo([self.names + " Colored Ball Detected!"])

                """ Update the points queue """
                self.pts.appendleft((int(self.x), int(self.y)))

                """ Dataset collections - For ML/AI purpose only """
                # self.do_saveImage()

            else:
                cv2.putText(self.cv_image, "No Colored Ball Detected!", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
                # rospy.logwarn(["No Colored Ball Detected!"])
                # pass
        else:
            cv2.putText(self.cv_image, "No Colored Ball Detected!", (10, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
            # rospy.logwarn(["No Colored Ball Detected!"])
            # pass

    def ptsTrack(self):
        """ loop over the set of tracked points """
        for i in range(1, len(self.pts)):

            """ if either of the tracked points are None, ignore them """
            if self.pts[i - 1] is None or self.pts[i] is None:
                continue

            """ check to see if enough points have been accumulated in the buffer """
            if self.counter >= 10 and i == 10 and self.pts[i-10] is not None:
                """ compute the difference between the x and y coordinates and re-initialize the direction text variables """
                self.dX = self.pts[-10][0] - self.pts[i][0]
                self.dY = self.pts[-10][1] - self.pts[i][1]
                (self.dirX, self.dirY) = ("", "")

                """ ensure there is significant movement in the x-direction """
                if np.abs(self.dX) > 20:
                    self.dirX = "East" if np.sign(self.dX) == 1 else "West"

                """ ensure there is significant movement in the y-direction """
                if np.abs(self.dY) > 20:
                    self.dirY = "North" if np.sign(self.dY) == 1 else "South"

                """ handle when both directions are non-empty """
                if self.dirX != "" and self.dirY != "":
                    self.direction = "{}-{}".format(self.dirY, self.dirX)

                    """ otherwise, only one direction is non-empty """
                else:
                    self.direction = self.dirX if self.dirX != "" else self.dirY

            """ otherwise, compute the thickness of the line and draw the connecting lines """
            thickness = int(np.sqrt(32 / float(i + 1)) * 2.5)
            cv2.line(self.cv_image, self.pts[i - 1], self.pts[i], (255, 255, 255), thickness)

        """ show the movement deltas and the direction of movement on the frame """
        cv2.putText(self.cv_image, self.direction, (10, self.cv_image.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        cv2.putText(self.cv_image, "dx: {}, dy: {}".format(self.dX, self.dY), (10, self.cv_image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
        cv2.putText(self.cv_image, "counter: {}".format(self.counter), (10, self.cv_image.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

        self.counter += 1

        """ Limit the dataset to save """
        # if self.counter > 100:
        #     rospy.signal_shutdown('Quit')

    # TODO: In order to apply ML -- Need to collect data
    def do_saveImage(self):
        """ define the name of the directory to be created (change accordingly) """
        path = "/home/khairulizwan/catkin_ws/src/ROS-Camera/camera_tutorials/dataset/ball_green"

        try:
            os.makedirs(path)
        except OSError:
            print ("Creation of the directory %s failed" % path)
            pass
        else:
            print ("Successfully created the directory %s " % path)
            pass

        img_no = "{:0>5d}".format(self.counter)
        filename = path + "/dataset_ball_green_" + str(img_no) +".png"

        # filename = "dataset/ball_green/dataset_ball_green_" + str(img_no) +".png"
        rospy.loginfo(filename)
        # cv2.imwrite(filename, self.cv_image_target)

    """ Refresh the image on the screen """
    def displayImg(self):
        cv2.imshow(self.cv_window_name, self.cv_image)
        cv2.waitKey(1)

    def shutdown(self):
        try:
            rospy.loginfo("Color tracking node [OFFLINE]...")

        finally:
            cv2.destroyAllWindows()

def usage():
    print("Please specify:")
    print("%s [Color Name] [v1_min] [v2_min] [v3_min] [v1_max] [v2_max] [v3_max]" % sys.argv[0])

def main(args):
    vn = color_tracking_node(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7])

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Color tracking node [OFFLINE]...")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    if len(sys.argv) < 1:
        print(usage())
        sys.exit(1)
    else:
        print("Color tracking node [ONLINE]...")
        main(sys.argv)
