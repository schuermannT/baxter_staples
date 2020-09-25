#!/usr/bin/env python
#-*- coding:utf-8 -*-
import argparse
import sys
import time
import rospy
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import baxter_interface
from copy import deepcopy


class Cam(object):
    def __init__(self, limb, verbose, arm_z=5.0, windowed=False):
        sub_cam = "/cameras/{}_hand_camera/image".format(limb)
        self._limb = limb
        self.controller = baxter_interface.CameraController("{}_hand_camera".format(limb))
        self.controller.resolution = (640, 400)
        self.bridge = CvBridge()
        self.update_snapshot = True
        self._verbose = verbose
        self._init = True
        self.sub = rospy.Subscriber(sub_cam, Image, self.show_callback)
        self.arm_z = arm_z
        self.windowed = windowed
        self._img = None
        self.ix = 0


    def show_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg)
            if self.update_snapshot:
                self._snapshot = deepcopy(img)
                self.update_snapshot = False
                #cv.imshow("snapshot", self._snapshot)
                self._update_snapshot_window = False
                if self._init:
                    #cv.setMouseCallback("snapshot", self.onMouse)
                    self._img = deepcopy(img)
                    self._init = False
            if self.arm_z < 5.0:
                action_point = self.get_action_point()
                cv.circle(img, action_point, 2, (0,0,255), -1)
            cv.imshow("live", img)
            cv.imshow("snapshot", self._snapshot)
            cv.imshow("input image", self._img)
            cv.waitKey(1)
        except CvBridgeError as e:
            print("Bridge-Error: {}".format(e))

    def onMouse(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            print("{},{}".format(x,y))

    def get_action_point(self):
        display_y = (163.15*self.arm_z)+162.13
        display_x = (-47.03*self.arm_z)+379.19
        if self.windowed:
            display_x -= 280
        return (int(display_x),int(display_y))

    def action_point(self, z):
        display_y = (163.15*z)+162.13
        display_x = (-47.03*z)+379.19
        if self.windowed:
            display_x -= 280
        return (int(display_x),int(display_y))

    def update_z(self, z):
        self.arm_z = z

    def set_img(self, img):
        self._img = img

    def write_img(self, img):
        cv.imwrite("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/pic_{}.jpg".format(self.ix), img)
        self.ix+=1
 