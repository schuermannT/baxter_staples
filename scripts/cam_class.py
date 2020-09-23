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
    def __init__(self, limb, verbose, arm_z=5.0):
        sub_cam = "/cameras/{}_hand_camera/image".format(limb)
        self._limb = limb
        self.controller = baxter_interface.CameraController("{}_hand_camera".format(limb))
        self.controller.resolution = (640, 400)
        self.bridge = CvBridge()
        self.update_snapshot = False
        self._verbose = verbose
        self._init = True
        self.sub = rospy.Subscriber(sub_cam, Image, self.show_callback)
        self.arm_z = arm_z

    def show_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg)
            if self.arm_z < 5.0:
                self.draw_action_point(img)
            if self.update_snapshot:
                self._snapshot = deepcopy(img)
                self.update_snapshot = False
                cv.imshow("snapshot", self._snapshot)
                self._update_snapshot_window = False
                if self._init:
                    cv.setMouseCallback("snapshot", self.onMouse)
                    self._init = False
            cv.imshow(self._limb, img)
            cv.waitKey(1)
        except CvBridgeError as e:
            print("Bridge-Error: {}".format(e))

    def onMouse(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            print("{},{}".format(x,y))

    def draw_action_point(self, img):
        display_y = (163.15*self.arm_z)+162.13
        display_x = (-47.03*self.arm_z)+379.19
        cv.circle(img, (int(display_x),int(display_y)), 2, (0,0,255), -1)

    def update_z(self, z):
        self.arm_z = z