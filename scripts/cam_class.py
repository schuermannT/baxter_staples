#!/usr/bin/env python
#-*- coding:utf-8 -*-
import argparse
import sys
import time
import rospy
import numpy
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import baxter_interface
from copy import deepcopy

class Cam(object):
    def __init__(self, limb, verbose):
        sub_cam = "/cameras/{}_hand_camera/image".format(limb)
        self._limb = limb
        self.controller = baxter_interface.CameraController("{}_hand_camera".format(limb))
        self.controller.resolution = (640, 400)
        self.sub = rospy.Subscriber(sub_cam, Image, self.show_callback)
        self.bridge = CvBridge()
        self._image_update = True
        self._verbose = verbose

    def show_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg)
            if self._image_update:
                self._img = deepcopy(img)
                cv.imshow("snapshot", self._img)
                self._image_update = False
            cv.imshow(self._limb, img)
            cv.waitKey(1)
        except CvBridgeError as e:
            print("Bridge-Error: {}".format(e))