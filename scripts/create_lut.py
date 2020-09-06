#!/usr/bin/env python
#-*- coding:utf-8 -*-
import argparse
import struct
import sys
import time

import rospy
from numpy import arange

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
                            x=0.380,
                            y=0.100,
                            z=-0.150,
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

def positive_x(arm, step_width, workspace_x=0.297, workspace_y=0.210):
    print("--- {} arm: positive x").format(arm._limb_name)    
    #raw_input("Press Enter to start...")
    out_dict = dict()
    if step_width < 0.02:
        #Initial Pose
        if arm.get_solution(arm_class.alter_pose_inc(deepcopy(start_pose[arm._limb_name].pose), arm._verbose, posx=-(step_width*2))): #approach starting point with minimal joint play
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
        next_pose = deepcopy(initial_pose)
        for y_step in arange(initial_pose.position.y, initial_pose.position.y+workspace_y, step_width):
            next_pose = arm_class.alter_pose_abs(next_pose, verbose=arm._verbose, posy=y_step)
            for x_step in arange(initial_pose.position.x, initial_pose.position.x+workspace_x, step_width):
                next_pose = arm_class.alter_pose_abs(next_pose, verbose=arm._verbose, posx=x_step)
                if arm.get_solution(next_pose):
                    arm.move_to_solution()
                else:
                    arm.simple_failsafe()
                    return False
                diff = Point(
                    x = arm._current_pose.position.x - next_pose.position.x,
                    y = arm._current_pose.position.y - next_pose.position.y,
                    z = arm._current_pose.position.z - next_pose.position.z
                )
                out_dict['x{}y{}'.format(((int)(next_pose.position.x*100)), ((int)(next_pose.position.y*100)))] = {
                    diff
                }
    else:
        next_pose = deepcopy(start_pose[arm._limb_name].pose)
        for y_step in arange(start_pose[arm._limb_name].pose.position.y, start_pose[arm._limb_name].pose.position.y+workspace_y, step_width):
            next_pose = arm_class.alter_pose_abs(next_pose, verbose=arm._verbose, posy=y_step)
            for x_step in arange(start_pose[arm._limb_name].pose.position.x, start_pose[arm._limb_name].pose.position.x+workspace_x, step_width):
                next_pose = arm_class.alter_pose_abs(next_pose, verbose=arm._verbose, posx=x_step)
                if not arm.move_precise(next_pose):
                    arm.simple_failsafe()
                    return False
                diff = Point(
                    x = arm._current_pose.position.x - next_pose.position.x,
                    y = arm._current_pose.position.y - next_pose.position.y,
                    z = arm._current_pose.position.z - next_pose.position.z
                )
                out_dict['x{}y{}'.format(((int)(next_pose.position.x*100)), ((int)(next_pose.position.y*100)))] = {
                    diff
                } 
    return out_dict

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
        lut = dict()
        lut[args.limb] = dict()
        print("Init finished...")

        #Move to neutral Pose
        arm.set_neutral()

        """ #Grab Pen
        raw_input("Press Enter to close gripper")
        time.sleep(5)
        arm._gripper.close() """

        #Measurements
        print("Starting measurements...")
        lut[args.limb]['x_pos'] = positive_x(arm, 0.03, workspace_x=0.297, workspace_y=0.210)
        if lut[args.limb]['x_pos'] is False:
            return False            

        print("\nMeasurements finished...\nExiting program...")
        arm.simple_failsafe()

        print lut

    except rospy.ROSInterruptException as e:
        return e
    except KeyboardInterrupt as e:
        return e

if __name__ == '__main__':
    main()