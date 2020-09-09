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

def positive_x(arm, out_dict=dict(), step_width=0.03, workspace_x=0.297, workspace_y=0.210):
    print("--- {} arm: positive x").format(arm._limb_name)    
    #raw_input("Press Enter to start...")
    diff = Point()
    if step_width < 0.02:
        print("running this script with a step_width of <0.02 will need a tremendous amount of time and is therefore in this version not supported...")
        return False
    next_pose = deepcopy(start_pose[arm._limb_name].pose)
    for y_step in arange(start_pose[arm._limb_name].pose.position.y, start_pose[arm._limb_name].pose.position.y+workspace_y, step_width):
        next_pose = arm_class.alter_pose_abs(next_pose, verbose=arm._verbose, posy=y_step)
        for x_step in arange(start_pose[arm._limb_name].pose.position.x, start_pose[arm._limb_name].pose.position.x+workspace_x, step_width):
            pose_name = 'y{}x{}'.format(((int)(y_step*100)), ((int)(x_step*100)))
            next_pose = arm_class.alter_pose_abs(next_pose, verbose=arm._verbose, posx=x_step)
            if not arm.move_precise(next_pose):
                arm.simple_failsafe()
                return False
            diff.x = arm._current_pose.position.x - next_pose.position.x
            diff.y = arm._current_pose.position.y - next_pose.position.y
            diff.z = arm._current_pose.position.z - next_pose.position.z
            if not pose_name in out_dict.keys():
                out_dict[pose_name] = deepcopy(diff)
            else:
                out_dict[pose_name].x = (out_dict[pose_name].x + diff.x)/2
                out_dict[pose_name].y = (out_dict[pose_name].y + diff.y)/2
                out_dict[pose_name].z = (out_dict[pose_name].z + diff.z)/2
        print("row finished: {}/{}".format(y_step, start_pose[arm._limb_name].pose.position.y+workspace_y))
    return out_dict

def print_csv(lut):
    print("\n-----------------------------------------------------------------------")
    for arm in lut.keys():
        print(arm)
        for way in lut[arm].keys():
            print(way)
            for pose in lut[arm][way].keys():
                print("{},{},{},{}".format(pose, lut[arm][way][pose].x, lut[arm][way][pose].y, lut[arm][way][pose].z))

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
        lut[args.limb]['x_pos'] = dict()
        print("Init finished...")

        #Move to neutral Pose
        arm.set_neutral()

        """ #Grab Pen
        raw_input("Press Enter to close gripper")
        time.sleep(5)
        arm._gripper.close() """

        #Measurements
        print("Starting measurements...")
        for r in range(10):
            lut[args.limb]['x_pos'] = positive_x(arm=arm, out_dict=lut[args.limb]['x_pos'], step_width=0.03, workspace_x=0.297, workspace_y=0.210)
            if lut[args.limb]['x_pos'] is False:
                return False
            print("-------------------> Round: {}".format(r+1))
            print_csv(lut)            

        print("\nMeasurements finished...\nExiting program...")
        arm.simple_failsafe()

    except rospy.ROSInterruptException as e:
        return e
    except KeyboardInterrupt as e:
        return e

if __name__ == '__main__':
    main()