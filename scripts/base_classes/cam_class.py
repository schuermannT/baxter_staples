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
                self._update_snapshot_window = False
            if self.arm_z < 5.0:
                action_point = self.get_action_point()
                cv.circle(img, action_point, 2, (0,0,255), -1)
            cv.imshow("live", img)
            cv.imshow("snapshot", self._snapshot)
            if self._init:
                cv.setMouseCallback("snapshot", self.onMouse)
                self._img = deepcopy(img)
                self._init = False
            cv.imshow("input image", self._img)
            cv.waitKey(1)
        except CvBridgeError as e:
            print("Bridge-Error: {}".format(e))

    def onMouse(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            print("{},{}".format(x,y))

    def get_action_point(self):
        display_y = 9842*np.float_power(self.arm_z, 3)+1071.3*np.square(self.arm_z)+224.15*self.arm_z+169.02 #<- polynomische Trendlinie 3. Ordnung; lineare Trendlinie: (362.83*self.arm_z)+177.55
        display_x = -4815.8*np.float_power(self.arm_z, 3)-734.51*np.square(self.arm_z)-76.228*self.arm_z+377.23 #<- polynomische Trendlinie 3. Ordnung; lineare Trendlinie:(-102.02*self.arm_z)+374.46
        if self.windowed:
            display_x -= 280
        return (int(display_x),int(display_y))

    def action_point(self, z):
        display_y = (362.83*z)+177.55       #Errechnet durch Mittelwert bei 3 Messdurchläufen und Trendlinie über diese (siehe Greiferhöhe zu Aktionspunkt.xlsx). Trifft nur auf Stifthalterung_MKII zu
        display_x = (-102.02*z)+374.46
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
 