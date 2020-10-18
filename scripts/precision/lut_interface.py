#!/usr/bin/env python
#-*- coding:utf-8 -*-

"""
Interface for the handling of the LookUpTable-Datatype (LUT).

This interface is capable of following actions:
- Creating a LUT from scratch
- Creating a LUT from file
- Printing a LUT
- Writing a LUT as a .csv-file
- Apply LUT to a given pose

A LUT is a multilayered dictionary object representing a matrix of measure points across a rectangular workspace.
Each value of the matrix holds the average deviation for this pose of one of the robots limbs.
For further information please see "Metallentfernung an Dokumenten durch den Forschungsroboter Baxter" by "Timo Schürmann".
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


def create_lut(arm, filename, number_of_rounds=10, verbose=False):
    """
    Creates a LUT for a specific workspace shaped according to DIN A4.

    This function moves the arm along a matrix of points across the workspace and measures the deviation.
    The measured deviation will be averaged for each point.
    Please consider, that 10 rounds will need around 50 minutes with a step_width of 0.03.

        Parameter:
            arm:                Robots limb to create a LUT for
            filename:           Absolute path to where the created LUT shall be saved
            number_of_rounds:   Number of rounds to go through for the creation
            verbose:            True -> Every 10 rounds an intermediate LUT will be saved to the filelocation
        Return:
            lut_data:           Created LUT-Object; False if creation failed
    """
    lut_data = dict()
    lut_data[arm._limb_name] = dict()
    lut_data[arm._limb_name]['x_pos'] = dict()
    print("Starting measurements...")
    for r in range(number_of_rounds):
            lut_data[arm._limb_name]['x_pos'] = measure_positive_x(arm=arm, data_dict=lut_data[arm._limb_name]['x_pos'], step_width=0.03, workspace_x=0.297, workspace_y=0.210)
            if lut_data[arm._limb_name]['x_pos'] is False:
                return False
            print("-------------------> Round: {}".format(r+1))
            if verbose and (r == 0 or (r+1)%10 == 0) and r < number_of_rounds-1:
                temp_lut = deepcopy(lut_data)
                print("averaging newest measurements...")
                for y in temp_lut[arm._limb_name]['x_pos'].keys():
                    for x in temp_lut[arm._limb_name]['x_pos'][y].keys():
                        temp_lut[arm._limb_name]['x_pos'][y][x].x /= (r+1)
                        temp_lut[arm._limb_name]['x_pos'][y][x].y /= (r+1)
                        temp_lut[arm._limb_name]['x_pos'][y][x].z /= (r+1)
                        time.sleep(0.01) #without this the calculations for z might generate strange values
                write_lut_as_csv(lut=temp_lut, number_of_rounds=r+1, filename=filename+"_{}_steps".format(r+1)) 
    print("averaging measurements...")
    for y in lut_data[arm._limb_name]['x_pos'].keys():
        for x in lut_data[arm._limb_name]['x_pos'][y].keys():
            lut_data[arm._limb_name]['x_pos'][y][x].x /= (r+1)
            lut_data[arm._limb_name]['x_pos'][y][x].y /= (r+1)
            lut_data[arm._limb_name]['x_pos'][y][x].z /= (r+1)
            time.sleep(0.01) #without this the calculations for z might generate strange values
    write_lut_as_csv(lut=lut_data, number_of_rounds=number_of_rounds, filename=filename+"_final")
    return lut_data

#/home/user/schuermann_BA/ros_ws/src/baxter_staples/precision/my_first_lut/measurements/calibrated/lut

def measure_positive_x(arm, data_dict=dict(), step_width=0.03, workspace_x=0.297, workspace_y=0.210):
    """
    Measure the accuracy of the given robots limb in a workspace of given size.
    The number of measurement points depends on the relationship between workspace size and step width.
    The measured data will be added to that of the given dictionary.

        Parameter:
            arm:            The robots limb to run the measurements on
            data_dict:      The dictionary to be modified; Default: New dictionary will be created
            step_width:     Distance between the measure points; Minimum: 0.02; Default: 0.03
            workspace_x:    Size of the workspace along the x axis
            workspace_y:    Size of the workspace along the y axis
        Return:
            data_dict:      Modified dictionary of measured data; False if any movement failed
    """
    print("--- {} arm: positive x").format(arm._limb_name)    
    diff = Point()
    if step_width < 0.02:
        print("running this script with a step_width < 0.02 will need a lot of time and is therefore not supported in this version...")
        return False
    next_pose = deepcopy(start_pose[arm._limb_name].pose)
    for y_step in arange(start_pose[arm._limb_name].pose.position.y, start_pose[arm._limb_name].pose.position.y+workspace_y, step_width):
        next_pose = arm_class.alter_pose_abs(next_pose, verbose=arm._verbose, posy=y_step)
        y_name = 'y{}'.format(int(y_step*100))
        if not y_name in data_dict.keys():
            data_dict[y_name] = dict()
        for x_step in arange(start_pose[arm._limb_name].pose.position.x, start_pose[arm._limb_name].pose.position.x+workspace_x, step_width):
            x_name = 'x{}'.format(int(x_step*100))
            next_pose = arm_class.alter_pose_abs(next_pose, verbose=arm._verbose, posx=x_step)
            if not arm.move_precise(next_pose):
                arm.simple_failsafe()
                return False
            time.sleep(0.1)
            diff.x = arm._current_pose.position.x - next_pose.position.x
            diff.y = arm._current_pose.position.y - next_pose.position.y
            diff.z = arm._current_pose.position.z - next_pose.position.z
            if not x_name in data_dict[y_name].keys():
                data_dict[y_name][x_name] = deepcopy(diff)
            else:
                data_dict[y_name][x_name].x += diff.x
                data_dict[y_name][x_name].y += diff.y
                data_dict[y_name][x_name].z += diff.z
        print("row finished: {}/{}".format(y_step, start_pose[arm._limb_name].pose.position.y+workspace_y))
    return data_dict

def print_lut_as_csv(lut, number_of_rounds):
    """
    Prints the given LUT and number of rounds in the same format as it would look in a file.

        Parameter:
            lut:                LUT-Object to be printed
            number_of_rounds:   Number of rounds used for the creation of this LUT
    """
    print("\n{}".format(number_of_rounds))
    for arm in lut.keys():
        print(arm)
        for way in lut[arm].keys():
            print(way)
            for y in lut[arm][way].keys():
                for x in lut[arm][way][y].keys():
                    print("{},{},{},{},{}".format(y, x, lut[arm][way][y][x].x, lut[arm][way][y][x].y, lut[arm][way][y][x].z))

def write_lut_as_csv(lut, number_of_rounds, filename):
    """
    Writes the given LUT to a .csv-file. 
    This file can be imported to a spreadsheet program or read back in with 'restore_lut_from_file()'.

        Parameter:
            lut:                The LUT-Object to be saved
            number_of_rounds:   Number of rounds used for the creation of this LUT
            filename:           Absolute path to where the given LUT shall be saved
    """
    writefile = open(filename, 'w')
    writefile.write("{}\n".format(number_of_rounds))
    for arm in lut.keys():
        writefile.write("{}\n".format(arm))
        for way in lut[arm].keys():
            writefile.write("{}\n".format(way))
            for y in lut[arm][way].keys():
                for x in lut[arm][way][y].keys():
                    writefile.writelines("{},{},{},{},{}\n".format(y, x, lut[arm][way][y][x].x, lut[arm][way][y][x].y, lut[arm][way][y][x].z))
    writefile.close()
    print("data written to file")

def restore_lut_from_file(filename):
    """
    Creates a LUT-Object from a suitable file.

        Parameter:
            filename:       Absolute path to the file to be read
        Return:
            restored_data:  Created LUT-Object
    """
    readfile = open(filename, 'r')
    lut = dict()
    lut_string = readfile.readlines()
    arm = lut_string[1].strip()
    way = lut_string[2].strip()
    lut[arm]= dict()
    lut[arm][way] = dict()
    for p in range(3, len(lut_string)):
        string_list = lut_string[p].split(',')
        if not string_list[0] in lut[arm][way].keys():
            lut[arm][way][string_list[0]] = dict()
        lut[arm][way][string_list[0]][string_list[1]] = Point(
            x = float(string_list[2]),
            y = float(string_list[3]),
            z = float(string_list[4])
        )
    restored_data = dict()
    restored_data['number_of_rounds'] = lut_string[0]
    restored_data['lut'] = lut
    return restored_data

def improve_pose(pose, lut, limb_name = 'left'):
    """
    Applies the data of the given LUT-Object to the given pose to improve its accuracy.

        Parameter:
            pose:       Pose to be improved
            lut:        LUT to improve pose with
            limb_name:  Name of the robots limb on which the LUT got created
        Return:
            pose:       Improved pose
    """
    #Create possibilities
    y_name = "y{}".format(int(pose.position.y*100))
    y_name_plus = "y{}".format(int(pose.position.y*100)+1)
    y_name_minus = "y{}".format(int(pose.position.y*100)-1)
    x_name = "x{}".format(int(pose.position.x*100))
    x_name_plus = "x{}".format(int(pose.position.x*100)+1)
    x_name_minus = "x{}".format(int(pose.position.x*100)-1)
    #poses y resembles point in LUT
    if y_name in lut[limb_name]['x_pos'].keys():
        #poses x resembles point in LUT
        if x_name in lut[limb_name]['x_pos'][y_name].keys():
            x_diff = lut[limb_name]['x_pos'][y_name][x_name].x
            y_diff = lut[limb_name]['x_pos'][y_name][x_name].y
            z_diff = lut[limb_name]['x_pos'][y_name][x_name].z
        #poses x slightly smaller than point in LUT
        elif x_name_plus in lut[limb_name]['x_pos'][y_name].keys():
            x_name_minus_2 = "x{}".format(int(pose.position.x*100)-2)
            x_diff = ((lut[limb_name]['x_pos'][y_name][x_name_plus].x*2)+lut[limb_name]['x_pos'][y_name][x_name_minus_2].x)/3
            y_diff = ((lut[limb_name]['x_pos'][y_name][x_name_plus].y*2)+lut[limb_name]['x_pos'][y_name][x_name_minus_2].y)/3
            z_diff = ((lut[limb_name]['x_pos'][y_name][x_name_plus].z*2)+lut[limb_name]['x_pos'][y_name][x_name_minus_2].z)/3
        #poses x slightly bigger than point in LUT
        elif x_name_minus in lut[limb_name]['x_pos'][y_name].keys():
            x_name_plus_2 = "x{}".format(int(pose.position.x*100)+2)
            x_diff = ((lut[limb_name]['x_pos'][y_name][x_name_minus].x*2)+lut[limb_name]['x_pos'][y_name][x_name_plus_2].x)/3
            y_diff = ((lut[limb_name]['x_pos'][y_name][x_name_minus].y*2)+lut[limb_name]['x_pos'][y_name][x_name_plus_2].y)/3
            z_diff = ((lut[limb_name]['x_pos'][y_name][x_name_minus].z*2)+lut[limb_name]['x_pos'][y_name][x_name_plus_2].z)/3
        else:
            print("improve_pose: given pose not in improvable workspace")
            return pose
    #poses y slightly smaller than point in LUT
    elif y_name_plus in lut[limb_name]['x_pos'].keys():
        #poses x resembles point in LUT
        if x_name in lut[limb_name]['x_pos'][y_name_plus].keys():
            x_diff = lut[limb_name]['x_pos'][y_name_plus][x_name].x
            y_diff = lut[limb_name]['x_pos'][y_name_plus][x_name].y
            z_diff = lut[limb_name]['x_pos'][y_name_plus][x_name].z
        #poses x slightly smaller than point in LUT
        elif x_name_plus in lut[limb_name]['x_pos'][y_name_plus].keys():
            x_name_minus_2 = "x{}".format(int(pose.position.x*100)-2)
            x_diff = ((lut[limb_name]['x_pos'][y_name_plus][x_name_plus].x*2)+lut[limb_name]['x_pos'][y_name_plus][x_name_minus_2].x)/3
            y_diff = ((lut[limb_name]['x_pos'][y_name_plus][x_name_plus].y*2)+lut[limb_name]['x_pos'][y_name_plus][x_name_minus_2].y)/3
            z_diff = ((lut[limb_name]['x_pos'][y_name_plus][x_name_plus].z*2)+lut[limb_name]['x_pos'][y_name_plus][x_name_minus_2].z)/3
        #poses x slightly bigger than point in LUT
        elif x_name_minus in lut[limb_name]['x_pos'][y_name_plus].keys():
            x_name_plus_2 = "x{}".format(int(pose.position.x*100)+2)
            x_diff = ((lut[limb_name]['x_pos'][y_name_plus][x_name_minus].x*2)+lut[limb_name]['x_pos'][y_name_plus][x_name_plus_2].x)/3
            y_diff = ((lut[limb_name]['x_pos'][y_name_plus][x_name_minus].y*2)+lut[limb_name]['x_pos'][y_name_plus][x_name_plus_2].y)/3
            z_diff = ((lut[limb_name]['x_pos'][y_name_plus][x_name_minus].z*2)+lut[limb_name]['x_pos'][y_name_plus][x_name_plus_2].z)/3
        else:
            print("improve_pose: given pose not in improvable workspace")
            return pose
    #Given poses y is slightly bigger than point in the LUT
    elif y_name_minus in lut[limb_name]['x_pos'].keys():
        #poses x resembles point in LUT
        if x_name in lut[limb_name]['x_pos'][y_name_minus].keys():
            x_diff = lut[limb_name]['x_pos'][y_name_minus][x_name].x
            y_diff = lut[limb_name]['x_pos'][y_name_minus][x_name].y
            z_diff = lut[limb_name]['x_pos'][y_name_minus][x_name].z
        #poses x slightly smaller than point in LUT
        elif x_name_plus in lut[limb_name]['x_pos'][y_name_minus].keys():
            x_name_minus_2 = "x{}".format(int(pose.position.x*100)-2)
            x_diff = ((lut[limb_name]['x_pos'][y_name_minus][x_name_plus].x*2)+lut[limb_name]['x_pos'][y_name_minus][x_name_minus_2].x)/3
            y_diff = ((lut[limb_name]['x_pos'][y_name_minus][x_name_plus].y*2)+lut[limb_name]['x_pos'][y_name_minus][x_name_minus_2].y)/3
            z_diff = ((lut[limb_name]['x_pos'][y_name_minus][x_name_plus].z*2)+lut[limb_name]['x_pos'][y_name_minus][x_name_minus_2].z)/3
        #poses x slightly bigger than point in LUT
        elif x_name_minus in lut[limb_name]['x_pos'][y_name_minus].keys():
            x_name_plus_2 = "x{}".format(int(pose.position.x*100)+2)
            x_diff = ((lut[limb_name]['x_pos'][y_name_minus][x_name_minus].x*2)+lut[limb_name]['x_pos'][y_name_minus][x_name_plus_2].x)/3
            y_diff = ((lut[limb_name]['x_pos'][y_name_minus][x_name_minus].y*2)+lut[limb_name]['x_pos'][y_name_minus][x_name_plus_2].y)/3
            z_diff = ((lut[limb_name]['x_pos'][y_name_minus][x_name_minus].z*2)+lut[limb_name]['x_pos'][y_name_minus][x_name_plus_2].z)/3
        else:
            print("improve_pose: given pose not in improvable workspace")
            return pose
    else:
        print("improve_pose: given pose not in improvable workspace")
        return pose
    return arm_class.alter_pose_inc(pose, posx=(-x_diff), posy=(-y_diff), posz=(-z_diff)) 

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
            #required=True,
            default='left',
            help='the limb to run the measurements on'
        )
        args = parser.parse_args(rospy.myargv()[1:])

        #Init
        rospy.init_node("lut_interface", anonymous = True)
        time.sleep(0.5)
        print("--- Ctrl-D stops the program ---")
        print("Init started...")
        arm = arm_class.Arm(args.limb, args.verbose)
        print("Init finished...")

        #Move to neutral Pose
        arm.set_neutral()

        create_lut(arm=arm, filename="/home/user/schuermann_BA/ros_ws/src/baxter_staples/precision/my_first_lut/lut.csv", number_of_rounds=50)

        print("\nMeasurements finished...\nExiting program...")
        arm.simple_failsafe()

    except rospy.ROSInterruptException as e:
        return e
    except KeyboardInterrupt as e:
        return e

if __name__ == '__main__':
    main()