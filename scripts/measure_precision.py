#!/usr/bin/env python
#-*- coding:utf-8 -*-
import argparse
import struct
import sys
import time

import rospy

import csv

import measure_precision_lib
import class_arm

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)

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
        arm = class_arm.Arm(args.limb, args.verbose)
        print("Init finished...")

        #print(class_arm.convert_to_pose(arm._limb.endpoint_pose()))

        #Move to neutral Pose
        arm.set_neutral()

        #Measurements
        print("Starting measurements...")
        
            #Right Arm
                #Positive X
        print("--- {} arm: Positive X").format(arm._limb_name)
        filename = '{}_positive_x.csv'.format(arm._limb_name)
        with open(filename, mode='wb') as csv_file:
            header = ['Pose', 'Soll', 'Ist']
            writer = csv.DictWriter(csv_file, header)
            writer.writeheader()
            """ if arm.get_solution(class_arm.alter_pose_inc(measure_precision_lib.start_poses[arm._limb_name].pose, arm._verbose, posx=-0.1)):
                arm.move_to_solution()
            else:
                arm.simple_failsafe()
                return False
            if arm.get_solution(measure_precision_lib.start_poses[arm._limb_name].pose):
                arm.move_to_solution()
            else:
                arm.simple_failsafe()
                return False
            print("--->Reached: initial pose\nStarting Measurements...")
            for x in range(0.1, 1.1, 0.1):
                next_pose = class_arm.alter_pose_inc(measure_precision_lib.start_poses[arm._limb_name], arm._verbose, posx=x)
                if arm.get_solution(next_pose):
                    arm.move_to_solution()
                else:
                    arm.simple_failsafe()
                    return False
                print("Reached: {}").format((int) (x*10))
                writer.writerow([x*10, next_pose.position.x, arm._current_pose.position.x]) """
                
        print("Finished: Right Arm: Positive X")




    except rospy.ROSInterruptException as e:
        return e
    except KeyboardInterrupt as e:
        return e

if __name__ == '__main__':
    main()