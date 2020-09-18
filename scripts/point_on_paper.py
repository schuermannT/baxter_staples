#!/usr/bin/env python
#-*- coding:utf-8 -*-
import argparse
import sys
import time
import rospy
import numpy
import cv2 as cv
from copy import deepcopy
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion
)

import arm_class
import cam_class
import baxter_interface

pose = Pose(
    position=Point(
        x=0.500,
        y=0.200,
        z=-0.120,
    ),
    orientation=Quaternion(
        x=-0.000,
        y=0.999,
        z=0.000,
        w=0.000,
    ),
)

def main():
    # Argument Parsing
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    parser.add_argument(
        '-v', '--verbose',
        action='store_const',
        const=True,
        default=True,
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
    rospy.init_node("point_on_paper", anonymous = True)
    time.sleep(0.5)
    print("Init started...")
    arm = arm_class.Arm(args.limb, args.verbose)
    cam = cam_class.Cam(args.limb, args.verbose)
    print("Init finished...")

    arm.set_neutral()
    raw_input("Press Enter to grab pen...")
    for i in range(3):
        print("gripping in: {}".format(3-i))
        time.sleep(1)
    arm._gripper.close()
    #high pose
    if not arm.move_to_pose(arm_class.alter_pose_inc(pose, args.verbose, posz=0.12)):
        arm.simple_failsafe()
        return False
    if not arm.move_direct(pose):
        arm.simple_failsafe()
        return False
    if not arm.move_to_pose(arm_class.alter_pose_inc(pose, args.verbose, posz=-0.01)):
        arm.simple_failsafe()
        return False
    if not arm.move_direct(arm_class.alter_pose_inc(pose, args.verbose, posz=0.12)):
        arm.simple_failsafe()
        return False
    
    #exit strategy
    arm.set_neutral(False)
    raw_input("please remove pen...")
    arm.simple_failsafe(True)


if __name__ == "__main__":
    main()