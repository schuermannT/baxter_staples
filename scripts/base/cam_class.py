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
    """
    Class to control one hand camera of Baxter
    """
    def __init__(self, limb, verbose, arm_z=5.0):
        """
        Constructor for the Cam class.

            Parameters:
                limb:       The limb of Baxter whichs camera shall be managed; Options: 'left', 'right'
                verbose:    True -> print verbose information; False -> dont print verbose information; Default: False
                arm_z:      z value of the cameras arm in meter (needed for action point prediction); Default: 5.0
        """
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
        self.windowed = False
        self._img = None
        self.ix = 0


    def show_callback(self, msg):
        """
        Callback method for the image stream of the hand camera.

        Displays the live image stream of the cam, the current snapshot and the current highlight.

            Parameters:
                msg:    The image message to be displayed
        """
        try:
            img = self.bridge.imgmsg_to_cv2(msg)
            if self.update_snapshot:
                self._snapshot = deepcopy(img)
                self.update_snapshot = False
                self._update_snapshot_window = False
            if self.arm_z < 5.0:
                action_point = self.get_action_point()
                cv.circle(img, action_point, 2, (0,0,255), -1)
            cv.imshow("Live", img)
            cv.imshow("Snapshot", self._snapshot)
            if self._init:
                cv.setMouseCallback("Snapshot", self.onMouse)
                self._img = deepcopy(img)
                self._init = False
            cv.imshow("Highlight", self._img)
            cv.waitKey(1)
        except CvBridgeError as e:
            print("Bridge-Error: {}".format(e))

    def onMouse(self, event, x, y, flags, param):
        """
        Eventhandler for clicking on the 'Snapshot' image.

        Prints the pixel coordinates of the point on which the mouse clicked on the 'Snapshot' display.

            Parameters:
                event:  The triggering event type
                x:      The x coordinate at which the event got triggered
                y:      The y coordinate at which the event got triggered
                flags:  Flags that come with the event
                param:  Optional parameters that belong to the triggering event
        """
        if event == cv.EVENT_LBUTTONDOWN:
            print("{},{}".format(x,y))

    def get_action_point(self):
        """
        Calculates the action point depending on the current z value of the arm.

        For further information on the used calculations please see "Metallentfernung an Dokumenten durch den Forschungsroboter Baxter" by "Timo Schürmann".
        """
        display_y = 2748.1*np.float_power(self.arm_z, 3)-789.76*np.square(self.arm_z)+144.88*self.arm_z+166.8  #third order polynomial trend line 
        display_x = -1719*np.float_power(self.arm_z, 3)+200.79*np.square(self.arm_z)-8.1584*self.arm_z+374.83  #third order polynomial trend line 
        if self.windowed:
            display_x -= 280
        return (int(display_x),int(display_y))

    def distance_to_point(self, point):
    """
    Calculates the distance between the action point and a given point in meter and split into x and y.

    For more information on the used formula please see "Metallentfernung an Dokumenten durch den Forschungsroboter Baxter" by "Timo Schürmann".
    
        Parameters:
            img:                    Image to be masked
            gripper_action_point:   Action point of the used end effector
            arm_z:                  Current z value of the used end effector
        Return:
            distance:               Calculated distance in format: (x, y)
    """
        factor = -9530.9 * self.arm_z + 1949.7
        gripper_action_point = self.get_action_point()
        distance_x = (point[0] - gripper_action_point[0]) / factor
        distance_y = (-(point[1] - gripper_action_point[1]) / factor) - 0.004
        return (distance_x, distance_y)

    def update_z(self, z):
        """
        Saves the given z value for the calculation of the action point.

            Parameters:
                z:  Current z value of the gripper
        """
        self.arm_z = z

    def set_highlight(self, img):
        """
        Saves the given image as the new highlight to be displayed.

            Parameters:
                img:    Image to be displayed as highlight
        """
        self._img = img

    def write_img(self, img):
        """
        Saves the given image to a folder.
        Each method call will increase the index to be used in the naming of the file

            Parameters:
                img:    Image to be saved as file
        """
        cv.imwrite("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/bilder/08/pic_{}.jpg".format(self.ix), img)
        self.ix+=1
 