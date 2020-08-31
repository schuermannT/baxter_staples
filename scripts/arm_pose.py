#!/usr/bin/env python
#-*- coding:utf-8 -*-
import argparse
import sys
import time

import rospy
import arm_class


def main():
    try:
        arg_fmt = argparse.RawDescriptionHelpFormatter
        parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)

        parser.add_argument(
            '-l', '--limb',
            choices=['left', 'right'],
            required=True,
            help='The limb you want to know the pose of'
        )
        args = parser.parse_args(rospy.myargv()[1:])

        #Init
        rospy.init_node("arm_pose", anonymous=True)
        time.sleep(0.5)
        print("Init started...")
        arm = arm_class.Arm(args.limb, verbose=False)
        print("Init finished.")
        print("--- Ctrl-C stops the program ---")

        while True:
            x = raw_input("\nPress Enter to read pose or 'x' to exit")
            if x is 'x':
                break
            print("")
            print(arm_class.convert_to_pose(arm._limb.endpoint_pose()))

    except rospy.ROSInterruptException as e:
        return e
    except KeyboardInterrupt as e:
        return e

if __name__ == '__main__':
    main()