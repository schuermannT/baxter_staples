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

sys.path.append("/home/user/schuermann_BA/ros_ws/src/baxter_staples/scripts/base_classes")

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

approvement_height = -0.15
marking_height = -0.2

commands = {
    'update'    : "updates and shows the snapshot",
    'write'     : "saves the current snapshot as file",
    'window'    : "resizes the camera resolution to a specific field",
    'exposure'  : "change the exposure of the camera",
    'pose'      : "get the arms current pose",
    'gain'      : "change the cameras gain",
    'etc'       : "and many more..."
}

def mark_staple(arm, box_cnt):
    print("marking staple")
    arm.cam.set_img(detector.draw_cnt_on_img(box_cnt, arm.cam._snapshot))
    time.sleep(2)
    point_distance = detector.distance_to_point(box_cnt[0], arm.cam.action_point(arm._current_pose.position.z), arm._current_pose.position.z)
    markingpose = arm_class.alter_pose_inc(arm._current_pose, posx=point_distance[0], posy=point_distance[1])
    markingpose = arm_class.alter_pose_abs(markingpose, arm._verbose, posz=marking_height)
    print("current pose:\n{}\nmarkingpose:\n{}\n".format(arm._current_pose, markingpose))
    if not arm.move_direct(markingpose):
        return False
    retreat = arm_class.alter_pose_inc(arm._current_pose, posz=0.1)
    if not arm.move_to_pose(retreat):
        return False
    return True

def approve_matches(arm, matches, match_pose):
    prove_poses = list()
    contour_img = deepcopy(arm.cam._snapshot)
    for m in matches:
        arm.cam.set_img(detector.draw_cnt_on_img(m[1], contour_img))
        time.sleep(1)
        staple_distance = detector.distance_to_point(m[1][0], arm.cam.get_action_point(), arm._current_pose.position.z)
        prove_poses.append(arm_class.alter_pose_inc(arm._current_pose, posx=staple_distance[0], posy=staple_distance[1]+0.01))
    for p in prove_poses:
        if not arm.move_to_pose(arm_class.alter_pose_abs(p, arm._verbose, posz=approvement_height + 0.1)):
            return False
        if not arm.move_precise(arm_class.alter_pose_abs(p, arm._verbose, posz=approvement_height)):
            return False
        if not arm.cam.windowed:
            window(arm)
        else:
            arm.cam.update_snapshot = True
        time.sleep(1)
        small_roi(arm)
        time.sleep(1)
        success, found_match, cnt_img = detector.detect_staple(deepcopy(arm.cam._snapshot))
        """ inp = raw_input("write?")
        if inp == "w":
            arm.cam.write_img(arm.cam._snapshot) """
        if success:
            arm.cam.set_img(cnt_img)
            print("found it")
            if not mark_staple(arm, found_match):
                return False
            return True
        else:
            if not arm.move_to_pose(arm_class.alter_pose_inc(arm._current_pose, arm._verbose, posz=0.1)):
                return False   
    print("no staple found")

def window(arm):
    if not arm.cam.windowed:
        arm.cam.controller.resolution = (320, 200)
        arm.cam.controller.window = (600, 200)
        arm.cam.windowed = True
        arm.cam.update_snapshot = True
    else:
        arm.cam.controller.window = (320, 200)
        arm.cam.controller.resolution = (640,400)
        arm.cam.windowed = False
        arm.cam.update_snapshot = True

def small_roi(arm):
    arm.cam._snapshot = detector.mask_window(deepcopy(arm.cam._snapshot), arm.cam.get_action_point())

def full_run(arm):
    if arm.cam.windowed:
        window(arm)
    arm.move_to_pose(paper_pose)
    time.sleep(1)
    arm.cam.update_snapshot = True
    time.sleep(0.2)
    paper_success, only_rim, paper_cnt = detector.detect_paper(deepcopy(arm.cam._snapshot))
    if paper_success:
        arm.cam.set_img(only_rim)
        time.sleep(2)
        staple_success, matches = detector.detect_staple(only_rim)
        if staple_success:
            approve_matches(arm, matches, deepcopy(arm._current_pose))
    else:
        print("No document detectable. Please rearrange it on the workplate")

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
    arm = arm_class.Arm(args.limb, args.verbose, True)
    print("Init finished...")

    while(True):
        commando = raw_input("Enter command\n")
        if commando == 'update':
            arm.cam.update_snapshot = True
        elif commando == "write":
            cv.imwrite("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_test_images/rename_me.jpg", arm.cam._img)
        elif commando == "window":
            window(arm)
        elif commando == "exposure":
            cmd_param = raw_input("Enter value (0-100): ")
            arm.cam.controller.exposure = int(cmd_param)
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
            elif cmd_param == "neutral":
                arm.set_neutral()
            elif cmd_param == "approach":
                arm.move_direct(arm_class.alter_pose_abs(arm._current_pose, posz=-0.1))
        elif commando == "gain":
            cmd_param = raw_input("Enter value (0-79): ")
            arm.cam.controller.gain = int(cmd_param)
        elif commando == "find":
            if not arm.cam.windowed:
                paper_success, only_rim, cnt_img = detector.detect_paper(deepcopy(arm.cam._snapshot))
                if paper_success:
                    arm.cam.set_img(cnt_img)
                    staple_success, matches = detector.detect_staple(only_rim)
                    if staple_success:
                        approve_matches(arm, matches, deepcopy(arm._current_pose))
            else:
                success, contour, cnt_img = detector.detect_staple(deepcopy(arm.cam._snapshot))
                if success:
                    arm.cam.set_img(cnt_img)
                    ghi_cnt = contour
                    print ghi_cnt
        elif commando == "pen":
            raw_input("Press Enter to grab pen...")
            time.sleep(5)
            arm._gripper.close()
        elif commando == "roi":
            arm.cam._snapshot = detector.mask_window(deepcopy(arm.cam._snapshot), arm.cam.get_action_point())
        elif commando == "mark":
            mark_staple(arm, ghi_cnt)
        elif commando == "run":
            full_run(arm)
        elif commando == "up":
            arm.move_to_pose(arm_class.alter_pose_inc(arm._current_pose, arm._verbose, posz=0.15))
        elif commando == "quit":
            print("resetting camera settings")
            arm.cam.controller.resolution = (640, 400)
            arm.cam.controller.exposure = arm.cam.controller.CONTROL_AUTO
            arm.cam.controller.gain = arm.cam.controller.CONTROL_AUTO
            arm.cam.controller.white_balace_red = arm.cam.controller.CONTROL_AUTO
            arm.cam.controller.white_balace_green = arm.cam.controller.CONTROL_AUTO
            arm.cam.controller.white_balace_blue = arm.cam.controller.CONTROL_AUTO
            arm.move_to_pose(arm_class.alter_pose_inc(arm._current_pose, posz=0.1))
            arm.simple_failsafe()
            break
        else:
            print("no such command\navailable commands:")
            for c in commands.items():
                print("{} - {}".format(c[0], c[1]))
        

if __name__ == "__main__":
    main()