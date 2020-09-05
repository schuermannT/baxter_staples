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
    x_steps = (int)(workspace_x/(step_width))+1
    y_steps = (int)(workspace_y/(step_width))+1
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
    for y_step in range(y_steps):
        next_pose = deepcopy(initial_pose)
        next_pose = arm_class.alter_pose_inc(next_pose, verbose=arm._verbose, posy=step_width*(y_step+1))
        for x_step in range(x_steps):
            next_pose = arm_class.alter_pose_inc(next_pose, verbose=arm._verbose, posx=step_width)
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
            position = deepcopy(arm._current_pose.position)
            out_dict['x{}y{}'.format(x_step, y_step)] = {
                position,
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

        #print(arm_class.convert_to_pose(arm._limb.endpoint_pose()))

        #Move to neutral Pose
        arm.set_neutral()

        #Measurements
        print("Starting measurements...")
        lut[args.limb]['x_pos'] = positive_x(arm, 0.01, workspace_x=0.297, workspace_y=0.210)
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