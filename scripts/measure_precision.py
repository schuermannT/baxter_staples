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
        args = parser.parse_args(rospy.myargv()[1:])

        #Init
        rospy.init_node("measure_precision", anonymous = True)
        time.sleep(0.5)
        print("--- Ctrl-D stops the program ---")
        print("Init started...")
        right = class_arm.Arm("right", args.verbose)
        left = class_arm.Arm("left", args.verbose)
        print("Init finished...")

        #Move to neutral Pose
        right.set_neutral()
        left.set_neutral()

        #Measurements
        print("Starting measurements...")
        
            #Right Arm
                #Positive X
        print("--- Right Arm: Positive X")
        with open('right_positive_x.csv', mode='w') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=['Soll', 'Ist'])
            right.get_solution(class_arm.alter_pose_inc(measure_precision_lib.start_poses['right'], right._verbose, posx=-0.1))
            right.move_to_solution()
            right.get_solution(measure_precision_lib.start_poses['right'])
            right.move_to_solution()
            writer.writeheader()
            print("--->Reached: initial pose\nStarting Measurements...")
            for x in range(0.1, 1.1, 0.1):
                next_pose = class_arm.alter_pose_inc(measure_precision_lib.start_poses['right'], right._verbose, posx=x)
                right.get_solution(next_pose)
                right.move_to_solution()
                print("Reached: {}").__format__((int) (x*10))
                writer.writerow('Soll': next_pose.position.x, 'Ist': right._current_pose.position.x)
                
        print("Finished: Right Arm: Positive X")




    except rospy.ROSInterruptException as e:
        return e
    except KeyboardInterrupt as e:
        return e

if __name__ == '__main__':
    main()