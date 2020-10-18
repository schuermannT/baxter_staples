#!/usr/bin/env python
#-*- coding:utf-8 -*-

"""
A simple script to measure the accuracy of the robots arm movements depending on different increments and speed configurations.

The chosen arm approaches a hardcoded pose multiple times with different configurations and prints the outcome in a neat way to import it in a spreadsheet program as Microsoft Excel to analyze.
"""

import argparse
import struct
import sys
import time

import rospy

sys.path.append("/home/user/schuermann_BA/ros_ws/src/baxter_staples/scripts/base_classes")
import arm_class

from copy import deepcopy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)


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


def measure_positive_x(arm, step_width = 0.01, speed = 0.3):
    """
    Approaches one point 20 times and measures the difference between given pose and reached pose.
    
    The approaches contain two movements. The first is to minimize joint play, whereas the second is to reach the measurement pose.

        Parameter:
            arm:        The robots limb to perform the motion
            step_width: Width of the steps to be made in meter; Default: 0.01
            speed:      Speed at which the arm shall move; Default: 0.3; Range: 0.0-1.0
        Return:
            diff_dict:  Dictionary containing the measured data. Structure: diff_dict['x' : list(), 'y' : list()]
    """
    print("--- {} arm: positive x").format(arm._limb_name)    
    arm._limb.set_joint_position_speed(speed)
    diff_dict = dict()
    diff_dict['x'] = list()
    diff_dict['y'] = list()
    for round_counter in range(20):
        #Minimize joint play
        if not arm.move_to_pose(arm_class.alter_pose_inc(start_pose[arm._limb_name].pose, arm._verbose, posx=(step_width * -2))):
            arm.simple_failsafe()
            return False
        #Initial pose
        initial_pose = deepcopy(start_pose[arm._limb_name].pose)
        if not arm.move_to_pose(arm_class.alter_pose_inc(initial_pose, verbose=arm._verbose, posx=-(step_width))):
            arm.simple_failsafe()
            return False
        print("--->Reached: initial pose\nStarting Measurements...")
        #Measuring
        next_pose = arm_class.alter_pose_inc(arm_class.alter_pose_inc(initial_pose, arm._verbose, posx=step_width))
        if not arm.move_to_pose(next_pose):
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
            help='the limb to run the measurements on'
        )
        args = parser.parse_args(rospy.myargv()[1:])

        #Init
        rospy.init_node("measure_precision", anonymous = True)
        time.sleep(0.5)
        print("Init started...")
        arm = arm_class.Arm(args.limb, args.verbose)
        print("Init finished...")

        #Move to neutral Pose
        arm.set_neutral()

        #Measurements
        print("Starting measurements...")
        slow_short = measure_positive_x(arm, step_width=0.01, speed=0.1)
        slow_medium = measure_positive_x(arm, step_width=0.02, speed=0.1)
        slow_far = measure_positive_x(arm, step_width=0.03, speed=0.1)
        fast_short = measure_positive_x(arm, step_width=0.01, speed=0.3)
        fast_medium = measure_positive_x(arm, step_width=0.02, speed=0.3)
        fast_far = measure_positive_x(arm, step_width=0.03, speed=0.3)

        #Print data
        print("slow_short,slow_medium,slow_far,fast_short,fast_medium,fast_far")
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