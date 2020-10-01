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

sys.path.append("/home/user/schuermann_BA/ros_ws/src/baxter_staples/scripts/base_classes")

import arm_class
import cam_class
import baxter_interface

pose = Pose(
    position=Point(
        x=0.500,
        y=0.200,
        z=-0.190,
    ),
    orientation=Quaternion(
        x=-0.700,
        y=0.700,
        z=0.000,
        w=0.000
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
    rospy.init_node("pen_and_paper", anonymous = True)
    time.sleep(0.5)
    print("Init started...")
    #cam = cam_class.Cam(args.limb, args.verbose)
    arm = arm_class.Arm(args.limb, args.verbose, True)
    print("Init finished...")

    arm.set_neutral(False)
    raw_input("Press Enter to grab pen...")
    for i in range(3):
        print("gripping in: {}".format(3-i))
        time.sleep(1)
    arm._gripper.close()
    #high pose
    if not arm.move_to_pose(arm_class.alter_pose_inc(pose, args.verbose, posz=0.12)):
        arm.simple_failsafe()
        return False
    raw_input("mark point")
    if not arm.move_direct(pose):
        arm.simple_failsafe()
        return False
    if not arm.move_to_pose(arm_class.alter_pose_inc(pose, args.verbose, posz=-0.01)):
        arm.simple_failsafe()
        return False
    print(arm._current_pose)
    for i in range(20):
        if not arm.move_to_pose(arm_class.alter_pose_inc(pose, args.verbose, posz=(i*0.02))):
            arm.simple_failsafe()
            return False
        time.sleep(1)
        arm.cam.update_snapshot = True
        print(arm._current_pose)
        raw_input("{} - ".format(i+1))
    
    #exit strategy
    arm.set_neutral(False)
    raw_input("please remove pen...")
    arm.simple_failsafe(True)


if __name__ == "__main__":
    main()