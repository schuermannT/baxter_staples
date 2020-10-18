#!/usr/bin/env python
#-*- coding:utf-8 -*-

"""
Script to measure the robots accuracy in a specified workspace with rectangular shape.
The default size of this workspace equals that of a DIN A4 document. 
For this the robot approaches a number of points arranged in a matrix and prints the measured data in a neat way to be importet into a spreadsheet program.
"""

import argparse
import struct
import sys
import time

import rospy
from numpy import arange

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
    """
    Measure the accuracy of the given robots limb in a workspace of given size.
    The number of measurement points depends on the relationship between workspace size and step width.

        Parameter:
            arm:            The robots limb to be measured on
            step_width:     Distance between the measurement points
            workspace_x:    Size of the workspace along the x axis
            workspace_y:    Size of the workspace along the y axis
        Return:
            out_dict:       Dictionary that contains the measured data; Structure: outdict['x{}y{}' = Point]
    """
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
                out_dict['x{}y{}'.format(((int)(next_pose.position.x*100)), ((int)(next_pose.position.y*100)))] = diff
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
                out_dict['y{}x{}'.format(((int)(next_pose.position.y*100)), ((int)(next_pose.position.x*100)))] = diff
            print("row {} finished".format(y_step))
    #Print data
    print("Pose,x_diff[mm],y_diff[mm],z_diff[mm],,")
    for k in out_dict.keys():
        print("{},{},{},{},,").format(k, out_dict[k].x*1000, out_dict[k].y*1000, out_dict[k].z*1000)
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

        #Measurements
        print("Starting measurements...")
        for k in range(10):
            print("--------------> Runde: {}".format(k))
            lut[args.limb]['x_pos'] = positive_x(arm, step_width=0.03, workspace_x=0.297, workspace_y=0.210)
            if lut[args.limb]['x_pos'] is False:
                return False            

        print("\nMeasurements finished...\nExiting program...")
        arm.simple_failsafe()

        #print lut

    except rospy.ROSInterruptException as e:
        return e
    except KeyboardInterrupt as e:
        return e

if __name__ == '__main__':
    main()