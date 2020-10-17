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

#---------------------Constants-------------------------------------
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

pick_paper_pose = Pose(
    position=Point(
        x=0.660,
        y=-0.150,
        z=-0.180
    ),
    orientation=Quaternion(
        x=0.000,
        y=0.999,
        z=0.000,
        w=0.000
    )
)

approvement_height = -0.15
marking_height = -0.202

commands = {
    'cam'   :   "opens the menu for camera control",
    'arm'   :   "opens the menu for arm control",
    'find'  :   "tries to find a staple with current pose",
    'run'   :   "starts a full run from document provision to marking the staple",
    'quit'  :   "resets all camera settings, moves arms in neutral poses and disables robot"
}

#---------------------Program---------------------------------------
def mark_staple(arm, box_cnt):
    """
    Approaches the given contour in dependance of the arms current pose.

    For this function to work properly please dont move the arm inbetween finding the contour and calling this.

        Parameters:
            arm:        The robots limb to perform the movement
            box_cnt:    The box contour to be approached
        Return:
            True/False: False if any calculated pose is not reachable, else True
    """
    print("marking staple")
    arm.cam.set_highlight(detector.draw_cnt_on_img(box_cnt, arm.cam._snapshot))
    time.sleep(2)
    point_distance = detector.distance_to_point(box_cnt[0], arm.cam.gripper_action_point(), arm._current_pose.position.z)
    markingpose = arm_class.alter_pose_inc(arm._current_pose, posx=point_distance[0], posy=point_distance[1])
    markingpose = arm_class.alter_pose_abs(markingpose, arm._verbose, posz=marking_height)
    print("current pose:\n{}\nmarkingpose:\n{}\n".format(arm._current_pose, markingpose))
    if not arm.move_direct(markingpose):
        return False
    retreat = arm_class.alter_pose_inc(arm._current_pose, posz=0.05)
    if not arm.move_to_pose(retreat):
        return False
    return True

def approve_matches(arm, matches):
    """
    Approaches the given matches in dependency to the arms current pose and approves them in a close-up view.
    If a match gets confirmed it will be marked and the program stops.

    For this function to work properly please dont move the arm inbetween finding the matches and calling this.

        Parameters:
            arm:        The robots limb to perform the movements
            matches:    A list of box contours to be approved
        Return:
            True/False: False if any calculated pose is not reachable, else True
    """
    prove_poses = list()
    contour_img = deepcopy(arm.cam._snapshot)
    for m in matches:
        arm.cam.set_highlight(detector.draw_cnt_on_img(m[1], contour_img))
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
        if success:
            arm.cam.set_highlight(cnt_img)
            print("found it")
            if not mark_staple(arm, found_match):
                return False
            return True
        else:
            if not arm.move_to_pose(arm_class.alter_pose_inc(arm._current_pose, arm._verbose, posz=0.1)):
                return False   
    print("no staple found")

def window(arm):
    """
    Toggles between normal mode (640x400) and windowed mode (320x200).
    To check the current mode please check arm.cam.windowed.

        Parameters:
            arm:    The robots limb whose camera mode shall be toggled
    """
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
    """
    Masks the current snapshot so that it only displays a field around the action point.

        Parameters:
            arm:    The robots limb whose snapshot shall be masked             
    """
    arm.cam._snapshot = detector.mask_window(deepcopy(arm.cam._snapshot), arm.cam.get_action_point())

def find_staple(arm):
    """
    The complete walkthrough to find a staple on a provided document.

    Moves the arm to a pose to overview the complete workplate. Then detects the document and possible staples on it.
    Checks the possibilities and marks a match, if found.

        Parameters:
            arm:    The robots limb to execute the walkthrough 
    """
    if arm.cam.windowed:
        window(arm)
    arm.move_to_pose(paper_pose)
    time.sleep(1)
    arm.cam.update_snapshot = True
    time.sleep(0.2)
    paper_success, only_rim, paper_cnt = detector.detect_paper(deepcopy(arm.cam._snapshot))
    if paper_success:
        arm.cam.set_highlight(only_rim)
        time.sleep(2)
        staple_success, matches = detector.detect_staple(only_rim)
        if staple_success:
            approve_matches(arm, matches, deepcopy(arm._current_pose))
    else:
        print("No document detectable. Please rearrange it on the workplate")

def full_run(left):
    """
    The complete walkthrough with document provision, staple detection and staple marking.

        Parameters:
            left:   The arm which shall execute the detection cycle
        Return:
            True/False: False if any movement is not executable, else True
    """
    right = arm_class.Arm('right', left._verbose)
    left.set_neutral(False)
    right.set_neutral(False)
    #supply document
    if not right.pick(pick_paper_pose):
        right.simple_failsafe()
        return False
    if not right.place_paper():
        return False
    right.set_neutral()
    #detect and mark staple
    find_staple(left)
    left.set_neutral(False)
    return True


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
    args = parser.parse_args(rospy.myargv()[1:])

    #Init
    rospy.init_node("lut_interface", anonymous = True)
    time.sleep(0.5)
    print("--- Ctrl-D stops the program ---")
    print("Init started...")
    arm = arm_class.Arm('left', args.verbose, cam_wanted=True)
    print("Init finished...")

    while(True):
        cmd = raw_input("Enter command\n")
        if cmd == "cam":
            cmd_param = raw_input("possible options: update, write, window, exposure, gain, roi")
            if cmd_param == 'update':
                arm.cam.update_snapshot = True
            elif cmd_param == "write":
                cv.imwrite("/home/user/schuermann_BA/ros_ws/src/baxter_staples/cv_images/rename_me.jpg", arm.cam._img)
            elif cmd_param == "window":
                window(arm)
            elif cmd_param == "exposure":
                cmd_param = raw_input("Enter value (0-100): ")
                arm.cam.controller.exposure = int(cmd_param)
            elif cmd_param == "gain":
                cmd_param = raw_input("Enter value (0-79): ")
                arm.cam.controller.gain = int(cmd_param)
            elif cmd_param == "roi":
                arm.cam._snapshot = detector.mask_window(deepcopy(arm.cam._snapshot), arm.cam.get_action_point())
            else:
                print("no such command")
        elif cmd == "arm":
            cmd_param = raw_input("possible options: get, go, rotate, neutral, approach, retreat, pen")
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
            elif cmd_param == "retreat":
                arm.move_to_pose(arm_class.alter_pose_inc(arm._current_pose, arm._verbose, posz=0.15))
            elif cmd_param == "pen":
                raw_input("Press Enter to grab pen...")
                for i in range(5):
                    print("gripping in: {}".format(5-i))
                    time.sleep(1)
                arm._gripper.close()
            else:
                print("no such command")
        elif cmd == "find":
            if arm.cam.windowed:
                window(arm)
            find_staple(arm)
            arm.set_neutral(False)
        elif cmd == "run":
            full_run(arm)
        elif cmd == "quit":
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