#!/usr/bin/env python
#-*- coding:utf-8 -*-
import argparse
import struct
import sys
import time

import rospy

import measure_precision_lib
import arm_class

from copy import deepcopy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)


def positive_x(arm, iterations):
    print("--- {} arm: positive x").format(arm._limb_name)    
    raw_input("Press Enter to start...")
    #Initial Pose
    if arm.get_solution(arm_class.alter_pose_inc(deepcopy(measure_precision_lib.middle_pose[arm._limb_name].pose), arm._verbose, posx=-0.12)): #approach starting point with minimal joint play
        arm.move_to_solution()
    else:
        arm.simple_failsafe()
        return False
    initial_pose = deepcopy(measure_precision_lib.middle_pose[arm._limb_name].pose)
    if arm.get_solution(arm_class.alter_pose_inc(initial_pose, verbose=arm._verbose, posx=-0.1)):
        arm.move_to_solution()
    else:
        arm.simple_failsafe()
        return False
    print("--->Reached: initial pose\nStarting Measurements...")
    #Readings: ['posenumber', 'setpoint', 'actual', 'difference in x', 'difference in y']
    setpoint = list()
    actual = list()
    x_diff = list()
    y_diff = list()
    #Measuring
    for x in range(1, iterations+1):
        next_pose = arm_class.alter_pose_inc(arm_class.alter_pose_inc(deepcopy(initial_pose), arm._verbose, posx=x*0.02))
        if arm.get_solution(next_pose):
            arm.move_to_solution()
        else:
            arm.simple_failsafe()
            return False
        print("Reached Pose {}").format((int) (x))
        setpoint.append(next_pose.position.x)
        actual.append(arm._current_pose.position.x)
        x_diff.append(arm._current_pose.position.x - next_pose.position.x)
        y_diff.append(arm._current_pose.position.y - next_pose.position.y)
    print("Finished: {} arm: positive x").format(arm._limb_name)
    print("Data following:\n")
    for it in range(iterations):
        print("{},{},{},{},{}").format(it+1, setpoint[it], actual[it], x_diff[it], y_diff[it])
    print("")
    return True

def negative_x(arm, iterations):
    print("--- {} arm: negative x").format(arm._limb_name)    
    raw_input("Press Enter to start...")
    #Initial Pose
    if arm.get_solution(arm_class.alter_pose_inc(deepcopy(measure_precision_lib.middle_pose[arm._limb_name].pose), arm._verbose, posx=0.12)): #approach starting point with minimal joint play
        arm.move_to_solution()
    else:
        arm.simple_failsafe()
        return False
    initial_pose = deepcopy(measure_precision_lib.middle_pose[arm._limb_name].pose)
    if arm.get_solution(arm_class.alter_pose_inc(initial_pose, verbose=arm._verbose, posx=0.1)):
        arm.move_to_solution()
    else:
        arm.simple_failsafe()
        return False
    print("--->Reached: initial pose\nStarting Measurements...")
    #Readings: ['posenumber', 'setpoint', 'actual', 'difference in x', 'difference in y']
    setpoint = list()
    actual = list()
    x_diff = list()
    y_diff = list()
    #Measuring
    for x in range(1, iterations+1):
        next_pose = arm_class.alter_pose_inc(arm_class.alter_pose_inc(deepcopy(initial_pose), arm._verbose, posx=-(x*0.02)))
        if arm.get_solution(next_pose):
            arm.move_to_solution()
        else:
            arm.simple_failsafe()
            return False
        print("Reached Pose {}").format((int) (x))
        setpoint.append(next_pose.position.x)
        actual.append(arm._current_pose.position.x)
        x_diff.append(arm._current_pose.position.x - next_pose.position.x)
        y_diff.append(arm._current_pose.position.y - next_pose.position.y)
    print("Finished: {} arm: negative x").format(arm._limb_name)
    print("Data following:\n")
    for it in range(iterations):
        print("{},{},{},{},{}").format(it+1, setpoint[it], actual[it], x_diff[it], y_diff[it])
    print("")
    return True

def positive_y(arm, iterations):
    print("--- {} arm: positive y").format(arm._limb_name)    
    raw_input("Press Enter to start...")
    #Initial Pose
    if arm.get_solution(arm_class.alter_pose_inc(deepcopy(measure_precision_lib.middle_pose[arm._limb_name].pose), arm._verbose, posy=-0.12)): #approach starting point with minimal joint play
        arm.move_to_solution()
    else:
        arm.simple_failsafe()
        return False
    initial_pose = deepcopy(measure_precision_lib.middle_pose[arm._limb_name].pose)
    if arm.get_solution(arm_class.alter_pose_inc(initial_pose, verbose=arm._verbose, posy=-0.1)):
        arm.move_to_solution()
    else:
        arm.simple_failsafe()
        return False
    print("--->Reached: initial pose\nStarting Measurements...")
    #Readings: ['posenumber', 'setpoint', 'actual', 'difference in y', 'difference in x']
    setpoint = list()
    actual = list()
    x_diff = list()
    y_diff = list()
    #Measuring
    for x in range(1, iterations+1):
        next_pose = arm_class.alter_pose_inc(arm_class.alter_pose_inc(deepcopy(initial_pose), arm._verbose, posy=x*0.02))
        if arm.get_solution(next_pose):
            arm.move_to_solution()
        else:
            arm.simple_failsafe()
            return False
        print("Reached Pose {}").format((int) (x))
        setpoint.append(next_pose.position.y)
        actual.append(arm._current_pose.position.y)
        x_diff.append(arm._current_pose.position.x - next_pose.position.x)
        y_diff.append(arm._current_pose.position.y - next_pose.position.y)
    print("Finished: {} arm: positive y").format(arm._limb_name)
    print("Data following:\n")
    for it in range(iterations):
        print("{},{},{},{},{}").format(it+1, setpoint[it], actual[it], y_diff[it], x_diff[it])
    print("")
    return True

def negative_y(arm, iterations):
    print("--- {} arm: negative y").format(arm._limb_name)    
    raw_input("Press Enter to start...")
    #Initial Pose
    if arm.get_solution(arm_class.alter_pose_inc(deepcopy(measure_precision_lib.middle_pose[arm._limb_name].pose), arm._verbose, posy=0.12)): #approach starting point with minimal joint play
        arm.move_to_solution()
    else:
        arm.simple_failsafe()
        return False
    initial_pose = deepcopy(measure_precision_lib.middle_pose[arm._limb_name].pose)
    if arm.get_solution(arm_class.alter_pose_inc(initial_pose, verbose=arm._verbose, posy=0.1)):
        arm.move_to_solution()
    else:
        arm.simple_failsafe()
        return False
    print("--->Reached: initial pose\nStarting Measurements...")
    #Readings: ['posenumber', 'setpoint', 'actual', 'difference in y', 'difference in x']
    setpoint = list()
    actual = list()
    x_diff = list()
    y_diff = list()
    #Measuring
    for x in range(1, iterations+1):
        next_pose = arm_class.alter_pose_inc(arm_class.alter_pose_inc(deepcopy(initial_pose), arm._verbose, posy=-(x*0.02)))
        if arm.get_solution(next_pose):
            arm.move_to_solution()
        else:
            arm.simple_failsafe()
            return False
        print("Reached Pose {}").format((int) (x))
        setpoint.append(next_pose.position.y)
        actual.append(arm._current_pose.position.y)
        x_diff.append(arm._current_pose.position.x - next_pose.position.x)
        y_diff.append(arm._current_pose.position.y - next_pose.position.y)
    print("Finished: {} arm: negative y").format(arm._limb_name)
    print("Data following:\n")
    for it in range(iterations):
        print("{},{},{},{},{}").format(it+1, setpoint[it], actual[it], y_diff[it], x_diff[it])
    print("")
    return True
    

def main():
    try:
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
            required=True,
            #default='right',
            help='the limb to run the measurements on'
        )
        args = parser.parse_args(rospy.myargv()[1:])

        #Init
        rospy.init_node("measure_precision", anonymous = True)
        time.sleep(0.5)
        print("--- Ctrl-D stops the program ---")
        print("Init started...")
        arm = arm_class.Arm(args.limb, args.verbose)
        print("Init finished...")

        #print(arm_class.convert_to_pose(arm._limb.endpoint_pose()))

        #Move to neutral Pose
        arm.set_neutral()

        #Measurements
        print("Starting measurements...")
        iterations = 10
            #Positive X
        if not positive_x(arm, iterations):
            return False
        if not negative_x(arm, iterations):
            return False
        if not positive_y(arm, iterations):
            return False
        if not negative_y(arm, iterations):
            return False

        print("\nMeasurements finished...\nExiting program...")
        arm.set_neutral()

    except rospy.ROSInterruptException as e:
        return e
    except KeyboardInterrupt as e:
        return e

if __name__ == '__main__':
    main()