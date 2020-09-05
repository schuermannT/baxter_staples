#!/usr/bin/env python
#-*- coding:utf-8 -*-
import argparse
import struct
import sys
import time

import rospy

import arm_class

from copy import deepcopy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)

#Startpose -> Alle weiteren Messposen werden in Abhängigkeit von dieser berechnet
start_pose = {
                'left': PoseStamped(
                    pose=Pose(
                        position=Point(
                            x=0.660,
                            y=0.300,
                            z=0.000,
                        ),
                        orientation=Quaternion(
                            x=-0.000,
                            y=0.999,
                            z=0.000,
                            w=0.000,
                        ),
                    ),
                ),
                'right': PoseStamped(
                    pose=Pose(
                        position=Point(
                            x=0.660,
                            y=-0.300,
                            z=0.000,
                        ),
                        orientation=Quaternion(
                            x=0.000,
                            y=0.999,
                            z=0.000,
                            w=0.000,
                        ),
                    ),
                )
            }


def positive_x(arm, step_width = 0.01, speed = 0.3):
    print("--- {} arm: positive x").format(arm._limb_name)    
    arm._limb.set_joint_position_speed(speed)
    #raw_input("Press Enter to start...")
    diff_dict = dict()
    diff_dict['x'] = list()
    diff_dict['y'] = list()
    for round_counter in range(10):
        #Initial Pose
        if arm.get_solution(arm_class.alter_pose_inc(deepcopy(start_pose[arm._limb_name].pose), arm._verbose, posx=(step_width * -2))): #approach starting point with minimal joint play
            arm.move_to_solution()
        else:
            arm.simple_failsafe()
            return False
        initial_pose = deepcopy(start_pose[arm._limb_name].pose)
        if arm.get_solution(arm_class.alter_pose_inc(initial_pose, verbose=arm._verbose, posx=-(step_width))):
            arm.move_to_solution()
        else:
            arm.simple_failsafe()
            return False
        print("--->Reached: initial pose\nStarting Measurements...")
        #Measuring
        next_pose = arm_class.alter_pose_inc(arm_class.alter_pose_inc(deepcopy(initial_pose), arm._verbose, posx=step_width))
        if arm.get_solution(next_pose):
            arm.move_to_solution()
        else:
            arm.simple_failsafe()
            return False
        diff_dict['x'].append(arm._current_pose.position.x - next_pose.position.x)
        diff_dict['y'].append(arm._current_pose.position.y - next_pose.position.y)
    return diff_dict

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
        slow_short = positive_x(arm, step_width=0.01, speed=0.1)
        slow_medium = positive_x(arm, step_width=0.02, speed=0.1)
        slow_far = positive_x(arm, step_width=0.03, speed=0.1)
        fast_short = positive_x(arm, step_width=0.01, speed=0.3)
        fast_medium = positive_x(arm, step_width=0.02, speed=0.3)
        fast_far = positive_x(arm, step_width=0.03, speed=0.3)

        print("x")
        for k in range(len(slow_short['x'])):
            print("{},{},{},{},{},{}").format(slow_short['x'][k], slow_medium['x'][k], slow_far['x'][k], fast_short['x'][k], fast_medium['x'][k], fast_far['x'][k])
        print("y")
        for k in range(len(slow_short['y'])):
            print("{},{},{},{},{},{}").format(slow_short['y'][k], slow_medium['y'][k], slow_far['y'][k], fast_short['y'][k], fast_medium['y'][k], fast_far['y'][k])

        print("\nMeasurements finished...\nExiting program...")
        arm.simple_failsafe()

    except rospy.ROSInterruptException as e:
        return e
    except KeyboardInterrupt as e:
        return e

if __name__ == '__main__':
    main()