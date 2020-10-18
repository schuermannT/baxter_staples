#!/usr/bin/env python
#-*- coding:utf-8 -*-

import argparse
import struct
import sys
import time

import rospy
from numpy import arange

sys.path.append("/home/user/schuermann_BA/ros_ws/src/baxter_staples/scripts/base_classes")
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
    """
    Adds the distance between the given pose and the current pose of the given arm to the given file.

        Parameter:
               arm:         Robots limb to get current pose from
               pose:        Pose to compare to current pose
               filename:    Absolute path to the file to be modified
    """
    writefile = open(filename, 'a')
    x_diff = (arm._current_pose.position.x - pose.position.x)*1000
    y_diff = (arm._current_pose.position.y - pose.position.y)*1000
    z_diff = (arm._current_pose.position.z - pose.position.z)*1000
    writefile.write("{},{},{}\n".format(x_diff, y_diff, z_diff))
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
        lut = lut_interface.restore_lut_from_file("/home/user/schuermann_BA/ros_ws/src/baxter_staples/precision/lut.csv")['lut']
        number_of_rounds = 10
        filelocation = "/home/user/schuermann_BA/ros_ws/src/baxter_staples/precision/"
        test_poses = [
            arm_class.alter_pose_inc(deepcopy(start_pose['left'].pose), verbose=args.verbose, posx=0.06, posy=0.06),
            arm_class.alter_pose_inc(deepcopy(start_pose['left'].pose), verbose=args.verbose, posx=0.06, posy=0.15),
            arm_class.alter_pose_inc(deepcopy(start_pose['left'].pose), verbose=args.verbose, posx=0.14, posy=0.11),
            arm_class.alter_pose_inc(deepcopy(start_pose['left'].pose), verbose=args.verbose, posx=0.23, posy=0.06),
            arm_class.alter_pose_inc(deepcopy(start_pose['left'].pose), verbose=args.verbose, posx=0.23, posy=0.15)
        ]
        print("Init finished...")

        #Measurements
        for n in range(number_of_rounds):
            print(n)
            for p in range(len(test_poses)):
                #not improved measurements
                if arm.get_solution(test_poses[p]):
                    arm.move_to_solution()
                    time.sleep(0.05)
                    save_data(arm, test_poses[p], filelocation+"not_improved_{}.csv".format(p+1))
                    time.sleep(0.05)
                else:
                    arm.simple_failsafe()
                    sys.exit()
            for p in range(len(test_poses)):
                #improved with move_precise
                if arm.move_precise(test_poses[p]):
                    time.sleep(0.05)
                    save_data(arm, test_poses[p], filelocation+"move_precise_{}.csv".format(p+1))
                    time.sleep(0.05)
                else:
                    arm.simple_failsafe()
                    sys.exit()
            for p in range(len(test_poses)):
                #improved with lut
                if arm.get_solution(lut_interface.improve_pose(deepcopy(test_poses[p]), lut=lut, limb_name=arm._limb_name)):
                    arm.move_to_solution()
                    time.sleep(0.05)
                    save_data(arm, test_poses[p], filelocation+"with_lut_{}.csv".format(p+1))
                    time.sleep(0.05)
                else:
                    arm.simple_failsafe()
                    sys.exit()
            for p in range(len(test_poses)):
                #improved with lut an move_precise
                if arm.move_precise(lut_interface.improve_pose(deepcopy(test_poses[p]), lut=lut, limb_name=arm._limb_name)):
                    time.sleep(0.05)
                    save_data(arm, test_poses[p], filelocation+"all_{}.csv".format(p+1))
                    time.sleep(0.05)
                else:
                    arm.simple_failsafe()
                    sys.exit()

        print("finished measurements")
    except rospy.ROSInterruptException as e:
        return e
    except KeyboardInterrupt as e:
        return e


if __name__ == "__main__":
    main()