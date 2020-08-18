#!/usr/bin/env python
#-*- coding:utf-8 -*-
import argparse
import struct
import sys
import time

import rospy

import csv

import measure_precision_lib
import arm_class

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)

def positive_x(self, arm):
    print("--- {} arm: Negative X").format(arm._limb_name)
    filename = '{}_positive_x.csv'.format(arm._limb_name)
    with open(filename, "w", newline="") as csv_file:
        header = ['Pose', 'Soll', 'Ist', 'Difference']
        writer = csv.writer(csv_file, dialect='excel')
        writer.writerow(header)
            #Initial Pose
        if arm.get_solution(arm_class.alter_pose_inc(measure_precision_lib.middle_pose[arm._limb_name].pose, arm._verbose, posx=-0.2)): #approach starting point with minimal joint play
            arm.move_to_solution()
        else:
            arm.simple_failsafe()
            return False
        if arm.get_solution(arm_class.alter_pose_inc(measure_precision_lib.middle_pose[arm._limb_name].pose, verbose=arm._verbose, posx=-0.1)):
            arm.move_to_solution()
        else:
            arm.simple_failsafe()
            return False
        print("--->Reached: initial pose\nStarting Measurements...")
            #Measuring
        for x in range(0.02, 0.22, 0.02):
            next_pose = arm_class.alter_pose_inc(arm_class.alter_pose_inc(measure_precision_lib.middle_pose[arm._limb_name].pose, arm._verbose, posx=-0.1))
            if arm.get_solution(next_pose):
                arm.move_to_solution()
            else:
                arm.simple_failsafe()
                return False
            print("Reached Pose {}").format((int) (x*5))
            writer.writerow([x*5, next_pose.position.x, arm._current_pose.position.x, (arm._current_pose.position.x - next_pose.position.x)])
        print("Finished: {} arm: positive X").format(arm._limb_name)

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
        print("--- Ctrl-D stops the program ---")
        print("Init started...")
        arm = arm_class.Arm(args.limb, args.verbose)
        print("Init finished...")

        #print(arm_class.convert_to_pose(arm._limb.endpoint_pose()))

        #Move to neutral Pose
        arm.set_neutral()

        #Measurements
        print("Starting measurements...")
            #Positive X
        if not positive_x(arm):
            return False


    except rospy.ROSInterruptException as e:
        return e
    except KeyboardInterrupt as e:
        return e

if __name__ == '__main__':
    main()