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
from copy import deepcopy

import arm_class
import baxter_interface

cmd_funcs = {
    "u" : "cam.update_snapshot"
}

class Handcam(object):
    def __init__(self, limb, verbose):
        sub_cam = "/cameras/{}_hand_camera/image".format(limb)
        self._limb = limb
        self.sub = rospy.Subscriber(sub_cam, Image, self.show_callback)
        self.bridge = CvBridge()
        self._img
        self._image_update = True
        self._verbose = verbose
        print("Getting robot state...")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        if not self._init_state:
            print("Enabling robot...")
            self._rs.enable()
        else:
            print("Robot already enabled...")

    def show_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg)
            if self._image_update:
                self._img = deepcopy(img)
                self._image_update = False
            cv.imshow(self._limb, img)
            cv.waitKey(1)
        except CvBridgeError as e:
            print("Bridge-Error: {}".format(e))

    def update_snapshot(self):
        self._image_update = True
        time.sleep(0.5)
        if not self._image_update:
            cv.imshow("Current Snapshot", self._img)
            return "snapshot updated and shown."


def main():
    # Argument Parsing
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    parser.add_argument(
        '-v', '--verbose',
        action='store_const',
        const=True,
        default=False,
        help="displays debug information (default = False)"
    )
    parser.add_argument(
        '-l', '--limb',
        choices=['left', 'right'],
        #required=True,
        default='left',
        help='the limb to run the measurements on'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    #Init
    rospy.init_node("lut_interface", anonymous = True)
    time.sleep(0.5)
    print("--- Ctrl-D stops the program ---")
    print("Init started...")
    arm = arm_class.Arm(args.limb, args.verbose)
    print("Init finished...")
    cam = Handcam(args.limb, args.verbose)

    while(True):
        cmd = raw_input("Enter command")
        func = cmd_funcs.get(cmd)
        if func is None:
            print("no such command.\n available commands:")
            for cf in cmd_funcs.keys():
                print cf
        else:
            print func()

if __name__ == "__main__":
    main()