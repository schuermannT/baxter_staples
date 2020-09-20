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
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion
)

sys.path.append("/home/user/schuermann_BA/ros_ws/src/baxter_staples/scripts/cv_scripts")

import arm_class
import cam_class
import detector
import baxter_interface

pose = Pose(
    position=Point(
        x=0.430,
        y=0.110,
        z=-0.200,
    ),
    orientation=Quaternion(
        x=-0.000,
        y=0.999,
        z=0.000,
        w=0.000,
    ),
)

paper_pose = Pose(
    position=Point(
        x=0.610,
        y=0.285,
        z=0.100
    ),
    orientation=Quaternion(
        x=-0.700,
        y=0.700,
        z=0.000,
        w=0.000
    )
)

""" Paper1280:
position: 
  x: 0.576751688976
  y: 0.330536543527
  z: -0.07204134598
orientation: 
  x: -0.703464896814
  y: 0.710467625684
  z: 0.0117420328291
  w: -0.0153302469344 

Paper640:
position: 
  x: 0.639653824451
  y: 0.333314267116
  z: -0.0556015894701
orientation: 
  x: -0.698709839035
  y: 0.715198454438
  z: -0.00254755593687
  w: 0.0170071033134"""

commands = {
    'update'    : "updates and shows the snapshot",
    'write'     : "saves the current snapshot as file",
    'window'    : "resizes the camera resolution to a specific field",
    'exposure'  : "change the exposure of the camera",
    'pose'      : "get the arms current pose",
    'gain'      : "change the cameras gain",
    'etc'       : "and many more..."
}

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
    cam = cam_class.Cam(args.limb, args.verbose)
    #cam.controller.resolution = (1280, 800)

    """ raw_input("Press Enter to grab pen...")
    time.sleep(5)
    arm._gripper.close()
    arm.set_neutral(False) """

    while(True):
        commando = raw_input("Enter command\n")
        if commando == 'update':
            cam._image_update = True
        elif commando == "write":
            cv.imwrite("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/rename_me.jpg", cam._img)
        elif commando == "window":
            cam.controller.resolution = (320, 200)
            cam.controller.window = (600, 200)
        elif commando == "exposure":
            cmd_param = raw_input("Enter value (0-100): ")
            cam.controller.exposure = int(cmd_param)
        elif commando == "pose":
            cmd_param = raw_input("get, go or rotate? ")
            if cmd_param == "get":
                print(arm._current_pose)
            elif cmd_param == "go":
                arm.set_neutral()
                arm.move_to_pose(arm_class.alter_pose_inc(deepcopy(paper_pose), posz=0.1))
                arm.move_precise(paper_pose)
            elif cmd_param == "rotate":
                cmd_param = raw_input("enter rotation: ")
                arm.move_to_pose(arm_class.alter_pose_inc(arm._current_pose, args.verbose, orx=float(cmd_param)+arm._current_pose.orientation.x))
        elif commando == "gain":
            cmd_param = raw_input("Enter value (0-79): ")
            cam.controller.gain = int(cmd_param)
        elif commando == "balance":
            cmd_param = raw_input("Enter key and value (red, green, blue) (0-4095): ").split(" ")
            if cmd_param[0] == "red":
                cam.controller.white_balace_red = int(cmd_param[1])
            if cmd_param[0] == "green":
                cam.controller.white_balace_green = int(cmd_param[1])
            if cmd_param[0] == "blue":
                cam.controller.white_balace_blue = int(cmd_param[1])
        elif commando == "find":
            detector.detect_staple(detector.detect_paper(deepcopy(cam._img)))
        elif commando == "quit":
            print("resetting camera settings")
            cam.controller.resolution = (640, 400)
            cam.controller.exposure = cam.controller.CONTROL_AUTO
            cam.controller.gain = cam.controller.CONTROL_AUTO
            cam.controller.white_balace_red = cam.controller.CONTROL_AUTO
            cam.controller.white_balace_green = cam.controller.CONTROL_AUTO
            cam.controller.white_balace_blue = cam.controller.CONTROL_AUTO
            arm.move_precise(arm_class.alter_pose_inc(arm._current_pose, posz=0.1))
            arm.simple_failsafe()
            break
        else:
            print("no such command\navailable commands:")
            for c in commands.items():
                print("{} - {}".format(c[0], c[1]))
        

if __name__ == "__main__":
    main()