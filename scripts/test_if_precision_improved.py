#!/usr/bin/env python
#-*- coding:utf-8 -*-
import argparse
import struct
import sys
import time

import rospy
from numpy import arange

import arm_class
import lut_interface

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

def save_data(arm, pose, filename):
    writefile = open(filename, 'a')
    x_diff = (arm._current_pose.position.x - pose.position.x)*1000
    y_diff = (arm._current_pose.position.y - pose.position.y)*1000
    z_diff = (arm._current_pose.position.z - pose.position.z)*1000
    writefile.write("{},{},{}\n").format(x_diff, y_diff, z_diff)
    writefile.close()
    print("saved data to {}".format(filename))



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
        """ parser.add_argument(
            '-l', '--limb',
            choices=['left', 'right'],
            required=True,
            #default='left',
            help='the limb to run the measurements on'
        ) """
        args = parser.parse_args(rospy.myargv()[1:])

        #Init
        rospy.init_node("measure_precision", anonymous = True)
        time.sleep(0.5)
        print("--- Ctrl-D stops the program ---")
        print("Init started...")
        arm = arm_class.Arm('left', verbose=args.verbose)
        lut = lut_interface.restore_lut_from_file("/home/schuermannBA/ros_ws/src/baxter_staples/precision/my_first_lut/lut")
        number_of_rounds = 10
        filelocation = "/home/schuermannBA/ros_ws/src/baxter_staples/precision/improvement_tests/"
        test_pose1 = arm_class.alter_pose_inc(deepcopy(start_pose['left'].pose), verbose=args.verbose, posx=0.12, posy=0.05)
        test_pose2 = arm_class.alter_pose_inc(deepcopy(start_pose['left'].pose), verbose=args.verbose, posx=0.2, posy=0.02)
        test_pose3 = arm_class.alter_pose_inc(deepcopy(start_pose['left'].pose), verbose=args.verbose, posx=0.15, posy=0.12)
        print("Init finished...")

        #not improved measurements
        for n in range(number_of_rounds):
            if arm.get_solution(test_pose1):
                arm.move_to_solution()
                save_data(arm, test_pose1, filelocation+"not_improved_1.csv")
            else:
                arm.simple_failsafe()
                return False
            if arm.get_solution(test_pose2):
                arm.move_to_solution()
                save_data(arm, test_pose2, filelocation+"not_improved_2.csv")
            else:
                arm.simple_failsafe()
                return False
            if arm.get_solution(test_pose3):
                arm.move_to_solution()
                save_data(arm, test_pose3, filelocation+"not_improved_3.csv")
            else:
                arm.simple_failsafe()
                return False
        #improved with move_precise
        for n in range(number_of_rounds):
            if arm.move_precise(test_pose1):
                save_data(arm, test_pose1, filelocation+"move_precise_1.csv")
            else:
                arm.simple_failsafe()
                return False
            if arm.move_precise(test_pose2):
                save_data(arm, test_pose2, filelocation+"move_precise_2.csv")
            else:
                arm.simple_failsafe()
                return False
            if arm.move_precise(test_pose3):
                save_data(arm, test_pose3, filelocation+"move_precise_3.csv")
            else:
                arm.simple_failsafe()
                return False
        #improved with lut
        for n in range(number_of_rounds):
            if arm.get_solution(lut_interface.improve_pose(test_pose1, lut=lut, limb_name='left')):
                arm.move_to_solution()
                save_data(arm, test_pose1, filelocation+"with_lut_1.csv")
            else:
                arm.simple_failsafe()
                return False
            if arm.get_solution(lut_interface.improve_pose(test_pose2, lut=lut, limb_name='left')):
                arm.move_to_solution()
                save_data(arm, test_pose2, filelocation+"with_lut_2.csv")
            else:
                arm.simple_failsafe()
                return False
            if arm.get_solution(lut_interface.improve_pose(test_pose3, lut=lut, limb_name='left')):
                arm.move_to_solution()
                save_data(arm, test_pose3, filelocation+"with_lut_3.csv")
            else:
                arm.simple_failsafe()
                return False
        #improved with lut an move_precise
        for n in range(number_of_rounds):
            if arm.move_precise(lut_interface.improve_pose(test_pose1, lut=lut, limb_name='left')):
                save_data(arm, test_pose1, filelocation+"all_1.csv")
            else:
                arm.simple_failsafe()
                return False
            if arm.move_precise(lut_interface.improve_pose(test_pose2, lut=lut, limb_name='left')):
                save_data(arm, test_pose2, filelocation+"all_2.csv")
            else:
                arm.simple_failsafe()
                return False
            if arm.move_precise(lut_interface.improve_pose(test_pose3, lut=lut, limb_name='left')):
                save_data(arm, test_pose3, filelocation+"all_3.csv")
            else:
                arm.simple_failsafe()
                return False

        print("finished measurements")
    except rospy.ROSInterruptException as e:
        return e
    except KeyboardInterrupt as e:
        return e


if __name__ == "__main__":
    main()