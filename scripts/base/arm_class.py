#!/usr/bin/env python
#-*- coding:utf-8 -*-

import struct
import sys
import time

import const_lib
import cam_class

import rospy
import baxter_interface

from copy import deepcopy

from std_msgs.msg import Header

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)

def alter_pose_abs(pose, verbose=False, posx = None, posy = None, posz = None, orx = None, ory = None, orz = None, orw = None):
        """
        Replaces the given poses parameters with given parameter modifiers.

            Parameter:
                pose:       The pose to be modified
                verbose:    True -> print verbose information; False -> dont print verbose information; Default: False
                posx:       The value for the positions x-value to be replaced; Default: None
                posy:       The value for the positions y-value to be replaced; Default: None
                posz:       The value for the positions z-value to be replaced; Default: None
                orx:        The value for the orientations x-value to be replaced; Default: None
                ory:        The value for the orientations y-value to be replaced; Default: None
                orz:        The value for the orientations z-value to be replaced; Default: None
                orw:        The value for the orientations w-value to be replaced; Default: None
            Return:
                workpose:   The modified pose
        """
        if verbose:
            print("--- func: alter_pose_abs ---")
            print("posx={} posy={} posz={} orx={} ory={} orz={} orw={}").format(posx, posy, posz, orx, ory, orz, orw)
        if posx is None: posx = pose.position.x
        if posy is None: posy = pose.position.y
        if posz is None: posz = pose.position.z
        if orx is None: orx = pose.orientation.x
        if ory is None: ory = pose.orientation.y
        if orz is None: orz = pose.orientation.z
        if orw is None: orw = pose.orientation.w
        if verbose: 
            print("--- given pose: ---")
            print(pose)
        workpose = deepcopy(pose)
        workpose.position.x = posx
        workpose.position.y = posy
        workpose.position.z = posz
        workpose.orientation.x = orx
        workpose.orientation.y = ory
        workpose.orientation.z = orz
        workpose.orientation.w = orw
        if verbose:
            print("--- new pose: ---")
            print(workpose)
            print("------------------------")
        return workpose

def alter_pose_inc(pose, verbose=False, posx = 0.0, posy = 0.0, posz = 0.0, orx = 0.0, ory = 0.0, orz = 0.0, orw = 0.0):
    """
    Adds the given parameter modifiers to the given poses parameters.

        Parameter:
            pose:       The pose to be modified
            verbose:    True -> print verbose information; False -> dont print verbose information; Default: False
            posx:       The value for the positions x-value to be modified; Default: None
            posy:       The value for the positions y-value to be modified; Default: None
            posz:       The value for the positions z-value to be modified; Default: None
            orx:        The value for the orientations x-value to be modified; Default: None
            ory:        The value for the orientations y-value to be modified; Default: None
            orz:        The value for the orientations z-value to be modified; Default: None
            orw:        The value for the orientations w-value to be modified; Default: None
        Return:
            workpose:   The modified pose
    """
    if verbose:
        print("--- func: alter_pose_inc ---")
        print("posx={} posy={} posz={} orx={} ory={} orz={} orw={}").format(posx, posy, posz, orx, ory, orz, orw) 
        print("--- given pose: ---")
        print(pose)
    workpose = deepcopy(pose)
    workpose.position.x += posx 
    workpose.position.y += posy
    workpose.position.z += posz
    workpose.orientation.x += orx
    workpose.orientation.y += ory
    workpose.orientation.z += orz
    workpose.orientation.w += orw
    if verbose:
        print("--- new pose: ---")
        print(workpose)
        print("------------------------")
    return workpose

def convert_to_pose(pose_dict):
    """
    Converts the given dictionary to a pose object.

        Parameters:
            pose_dict:  Dictionary containing a pose
        Return:
            pose:       The given dictionary as pose object
    """
    pose = Pose()
    pose.position.x = pose_dict['position'].x 
    pose.position.y = pose_dict['position'].y 
    pose.position.z = pose_dict['position'].z
    pose.orientation.x = pose_dict['orientation'].x 
    pose.orientation.y = pose_dict['orientation'].y 
    pose.orientation.z = pose_dict['orientation'].z 
    pose.orientation.w = pose_dict['orientation'].w
    return pose

def failsafe(left, right, tool=False):
    """
    Moves both arms to neutral positions and disables them.

        Parameters:
            left:   Left arm to be set neutral
            right:  Right arm to be set neutral
            tool:   Flag to determine if the tool shall be stored before setting the left arm neutral; Default: False
    """
    print("Exit routine started\nStoring Tool (if attached) and moving arms to neutral poses")
    right.set_neutral()
    if tool:
        left.store_tool()
    else:
        left.set_neutral()
    left._rs.disable()


class Arm(object):
    """
    Class to manage and control one arm of Baxter
    """
    def __init__(self, limb, verbose=False, cam_wanted=False):
        """
        Constructor for the Arm class.

            Parameters:
                limb:       The limb of Baxter to be managed; Options: 'left', 'right'
                verbose:    True -> print verbose information; False -> dont print verbose information; Default: False
                cam_wanted: Flag to determine if a cam object is created
        """
        self._limb_name = limb
        self._verbose = verbose
        self._limb = baxter_interface.limb.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        self._gripper._type = self._gripper.type()
        self._ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(self._ns, SolvePositionIK)
        self._ikreq = SolvePositionIKRequest()
        self._hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        self._current_pose = convert_to_pose(self._limb.endpoint_pose())
        if self._gripper._type is 'suction':
            self._gripper.set_vacuum_threshold(1.0)
        if self._gripper._type is 'electric':
            self._gripper.set_moving_force(100.0) 
        self._limb.set_joint_position_speed(0.3) 
        self.cam_wanted = cam_wanted   
        if cam_wanted:
            self.cam = cam_class.Cam(limb, self._verbose)
            self.cam.update_z(self._current_pose.position.z)
        
        print("Getting robot state...")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        if not self._init_state:
            print("Enabling robot...")
            self._rs.enable()
        else:
            print("Robot already enabled...")
        if self._gripper._type is 'electric' and not self._gripper.calibrated():
            print("Calibrating electric gripper...")
            self._gripper.calibrate()

    def get_solution(self, pose):
        """
        Warning: Deprecated. Please use move_to_pose() instead of get_solution() and move_to_solution(). These functions are only kept for compatibility.

        Calculates a joint solution for the given pose if reachable.

            Parameters:
                pose:       The pose to be reached by the endpoint effector.
            Return:
                False ->    No valid joint solution could be found for the given pose
                True ->     Valid joint solution found for the given pose  
        """
        if self._verbose: 
            print("--- func: get_solution ---")
            print("--- pose: ---")
            print(pose)
            print("------------------------")
        del self._ikreq.pose_stamp[:]
        self._ikreq.pose_stamp.append(PoseStamped(header=self._hdr, pose=pose))
        try:
            rospy.wait_for_service(self._ns, 5.0)
            resp = self._iksvc(self._ikreq)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service call failed: %s" %(e,))
            return False

        #Check if result is valid, and type of seed ultimately used to get solution
        #convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                self._ikreq.SEED_USER: 'User Provided Seed',
                self._ikreq.SEED_CURRENT: 'Current Joint Angles',
                self._ikreq.SEED_NS_MAP : 'Nullspace Setpoints',
            }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" % (seed_str,))
            #Format solution into Limb API-compatible dictionary
            self._ik_solution = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("\nIK Solution:\n", self._ik_solution)
                print("--------------------------")
                print("Response Message:\n", resp )
            return True
        else:
            print ("INVALID POSE - No Valid Joint Solution Found.")
        return False

    def move_to_solution(self):
        """
        Warning: Deprecated. Please use move_to_pose() instead of get_solution() and move_to_solution(). These functions are only kept for compatibility.

        Executes the movement to the previously calculated pose. Please run get_solution() beforehand for this to function properly.
        """
        if self._verbose:
            print("moving %s arm..." %self._limb_name)
        self._limb.move_to_joint_positions(self._ik_solution)
        self._current_pose = convert_to_pose(self._limb.endpoint_pose())
        time.sleep(1)   #to make sure he really finished his movement and reached the pose

    def move_to_pose(self, pose):
        """
        Calculates and executes a joint solution for the given pose.

            Parameters:
            pose:        The pose to be reached by the endpoint effector.
            Return:
                False ->    No valid joint solution could be found for the given pose
                True ->     The arm reached the given pose
        """
        if self._verbose: 
            print("--- func: get_solution ---")
            print("--- pose: ---")
            print(pose)
            print("------------------------")
        del self._ikreq.pose_stamp[:]
        self._ikreq.pose_stamp.append(PoseStamped(header=self._hdr, pose=pose))
        try:
            rospy.wait_for_service(self._ns, 5.0)
            resp = self._iksvc(self._ikreq)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service call failed: %s" %(e,))
            return False

        #Check if result is valid, and type of seed ultimately used to get solution
        #convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                self._ikreq.SEED_USER: 'User Provided Seed',
                self._ikreq.SEED_CURRENT: 'Current Joint Angles',
                self._ikreq.SEED_NS_MAP : 'Nullspace Setpoints',
            }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" % (seed_str,))
            #Format solution into Limb API-compatible dictionary
            self._ik_solution = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("\nIK Solution:\n", self._ik_solution)
                print("--------------------------")
                print("Response Message:\n", resp )
                print("moving %s arm..." %self._limb_name)
            #execute movement
            self._limb.move_to_joint_positions(self._ik_solution)
            time.sleep(1)   #to make sure he really finished his movement and reached the pose
            self._current_pose = convert_to_pose(self._limb.endpoint_pose())
            if self.cam_wanted:
                self.cam.update_z(self._current_pose.position.z)
            return True
        else:
            print ("INVALID POSE - No Valid Joint Solution Found.")
            print("Given Pose:\n{}".format(pose))
        return False

    def move_precise(self, pose):
        """
        Calculates and executes a joint solution for the given pose in a more precise way.

        Approaches the given pose in three steps to provide a more precise positioning. 
        The first step is offset by 2cm negative in x after which the arm approaches the given pose with one intermediate step.
        For more thorough information on this method please see "Metallentfernung an Dokumenten durch den Forschungsroboter Baxter" by "Timo Schürmann"

            Parameters:
            pose:        The pose to be reached by the endpoint effector.
            Return:
                False ->    No valid joint solution could be found for the given pose
                True ->     The arm reached the given pose
        """
        if self._verbose:
            print("moving {}_arm more precise...").format(self._limb_name)
        if not self.move_to_pose(alter_pose_inc(deepcopy(pose), self._verbose, posx=-0.02)):
            return False
        if self.cam_wanted:
            self.cam.update_z(self._current_pose.position.z)
        if not self.move_to_pose(alter_pose_inc(deepcopy(pose), self._verbose, posx=-0.01)):
            return False
        if not self.move_to_pose(pose):
            return False
        return True

    def move_direct(self, pose, precise=False):
        """
        Calculates and executes a joint solution for the given pose alongside the axes.

        Moves alongside the axes in multiple steps. The movement is executed in following sequence: x-axis, y-axis, z-axis
        Use this function if the workspace is obstructed by possible obstacles and you need a more linear movement.
            Parameters:
            pose:        The pose to be reached by the endpoint effector.
            precise:     Flag to determine if the given pose shall be approached in a more precise manner (for more information see move_precise()); Default: False
            Return:
                False ->    No valid joint solution could be found for the given pose
                True ->     The arm reached the given pose
        """
        if self._verbose:
            print("moving {}_arm on kind of linear way".format(self._limb_name))
        big_move = True
        step_width = 0.03
        while big_move:
            x_diff = pose.position.x - self._current_pose.position.x
            y_diff = pose.position.y - self._current_pose.position.y
            z_diff = pose.position.z - self._current_pose.position.z
            if self._verbose:
                print("way left:\nx: {}\ny: {}\nz: {}".format(x_diff, y_diff, z_diff))
            big_move = False
            x_step = 0.0
            y_step = 0.0
            z_step = 0.0
            if x_diff > step_width:
                x_diff -= step_width
                x_step = step_width
                big_move = True
            elif x_diff < -step_width:
                x_diff += step_width
                x_step = -step_width
                big_move = True
            elif y_diff > step_width:
                y_diff -= step_width
                y_step = step_width
                big_move = True
            elif y_diff < -step_width:
                y_diff += step_width
                y_step = -step_width
                big_move = True
            elif z_diff > step_width:
                z_diff -= step_width
                z_step = step_width
                big_move = True
            elif z_diff < -step_width:
                z_diff += step_width
                z_step = -step_width
                big_move = True
            elif precise:
                self.move_precise(pose)
                return True
            else:
                self.move_to_pose(pose)
                return True
            self.move_to_pose(alter_pose_inc(self._current_pose, verbose=self._verbose, posx=x_step, posy=y_step, posz=z_step))
            if self.cam_wanted:
                self.cam.update_z(self._current_pose.position.z)
        return False
            

    def set_neutral(self, open_gripper=True):
        """
        Moves the arm to a neutral pose.

            Parameters:
            open_gripper: Flag to determine if the gripper shall be opened after reaching the neutral pose
        """
        self._limb.move_to_neutral()
        if open_gripper:
            self._gripper.open()
        self._current_pose = convert_to_pose(self._limb.endpoint_pose())

    def pick(self, pick_pose, remove_staple=False, remove_clip=False, opening=0.0, hover_distance=0.1):
        """
        Performs a classic "pick" movement.

        Approaches the given pose from a heightened pose, closes the gripper and retreats to a heightened pose. 
        With the optional flag parameters slightly different movements can be performed to remove staples or clippers from documents.

            Parameters:
            pick_pose:       Pose at which the gripper shall be closed to perform the picking
            remove_staple:   Flag to determine if a special movement shall be executed for removing staples from documents; Default: False
            remove_clip:     Flag to determine if a special movement shall be executed for removing clips from documents; Default: False
            opening:         Percentage of opening for the gripper. 0.0 -> closed, 100.0 -> open; Default: 0.0
            hover_distance:  Distance between the heightened pose and the pick_pose; Default: 0.1
            Return:
                False ->    No valid joint solution could be found for the given pose
                True ->     The arm reached the given pose
        """
        if self._verbose: 
            print("--- func: pick ---")
            print("--- given pose: ---")
            print(pick_pose)
        hover_pose = alter_pose_inc(deepcopy(pick_pose), verbose=self._verbose, posz = hover_distance)
        #hover point
        if self.get_solution(hover_pose):
            if self._verbose:
                print("moving to hover pose...")
            self.move_to_solution()
            time.sleep(1)   #to prevent from further movement before reaching final position
        else:
            return False
        #pick point
        if self.get_solution(pick_pose):
            if self._verbose:
                print("approaching target...")
            self._limb.set_joint_position_speed(0.15)
            self.move_to_solution()
            print("------------> pick_pose: {}").format(self._limb.endpoint_pose())
        else: 
            return False
        #close gripper
        if self._gripper._type is 'electric':
            self._gripper.command_position(block=True, position=opening)
            time.sleep(1)
            if remove_staple:
                self._gripper.command_position(position=const_lib.gripper_opening)
                time.sleep(1)
            elif self._gripper.missed():
                print("Gripping with {}_arm failed").format(self._limb_name)
                return False
        elif self._gripper._type is 'suction':
            self._gripper.close(timeout = 1.0)
        else:
            print("\nCustom gripper type detected. Please change the grippers to the required hardware.\nRequired gripper hardware:\nLeft: Electric\nRight: Suction")
        #remove clip
        if remove_clip:
            if self.get_solution(alter_pose_inc(pick_pose, verbose=self._verbose, posx=-0.04)):
                if self._verbose:
                    print("removing clip...")
                self.move_to_solution()
            else:
                return False
        #retreat
        if self.get_solution(hover_pose):
            if self._verbose:
                print("retreating to safe position...")
            self.move_to_solution()
            if self._gripper._type is 'suction':
                if self._verbose:
                    print("----> suction: {}").format(self._gripper.vacuum_sensor())
                if not self._gripper.vacuum_sensor() > 8.0:
                        print("Gripping with {}_arm failed\nSuction: {}").format(self._limb_name, self._gripper.vacuum_sensor())
                        return False
            self._limb.set_joint_position_speed(0.3)
            return True
        else:
            return False

    def place(self, place_pose, opening=100.0, hover_distance=0.1):
        """
        Performs a classic "place" movement.

        Approaches the given pose from a heightened pose, opens the gripper and retreats to a heightened pose. 

            Parameters:
            place_pose:      Pose at which the gripper shall be opened to perform the placing
            opening:         Percentage of opening for the gripper. 0.0 -> closed, 100.0 -> open; Default: 100.0
            hover_distance:  Distance between the heightened pose and the pick_pose; Default: 0.1
            Return:
                False ->    No valid joint solution could be found for the given pose
                True ->     The arm reached the given pose    
        """
        if self._verbose: 
            print("--- func: place ---")
            print("--- given pose: ---")
            print(place_pose)
        safe_pose = alter_pose_inc(deepcopy(place_pose), verbose=self._verbose, posz = hover_distance)
        #hover point
        if self.get_solution(safe_pose):
            if self._verbose:
                print("moving to hover pose...")
            self.move_to_solution()
        else:
            return False
        #place point
        if self.get_solution(place_pose):
            if self._verbose:
                print("approaching target...")
            self._limb.set_joint_position_speed(0.15)
            self.move_to_solution()
            time.sleep(1)
            print("------------> place_pose: {}").format(self._limb.endpoint_pose())
            self._gripper.command_position(position=opening)
        else: 
            return False
        #retreat
        if self.get_solution(safe_pose):
            if self._verbose:
                print("retreating to safe position...")
            self.move_to_solution()
            self._limb.set_joint_position_speed(0.3)
            return True
        else:
            return False

    def place_paper(self):
        """
        Performs a special set of movements to place a multipaged document in a repeatable manner.

        For this method to function please provide a proper workspace as described in "Metallentfernung an Dokumenten durch den Forschungsroboter Baxter" by "Timo Schürmann"
        as all executed movements are hardcoded.

            Return:
                False ->    No valid joint solution could be found for the given pose
                True ->     The arm reached the given pose 
        """
        if self._verbose:
            print("--- func: place_paper ---")
        place_paper_pose = convert_to_pose(const_lib.place_paper_pose)
        safe_pose = alter_pose_inc(deepcopy(place_paper_pose), verbose=self._verbose, posz = const_lib.hover_distance['paper'])
        drag_paper_pose = alter_pose_inc(deepcopy(place_paper_pose), verbose=self._verbose, posx=0.30, posz=const_lib.hover_distance['paper'], orx=90.0)
        
        #initial hover point
        if self.get_solution(drag_paper_pose):
            if self._verbose:
                print("moving to hover pose...")
            self.move_to_solution()
        else: 
            return False
        #dragging start point
        alter_pose_abs(drag_paper_pose, verbose=self._verbose, posz=const_lib.table_height[self._gripper._type] + 0.015)
        alter_pose_inc(drag_paper_pose, verbose=self._verbose, posy=0.015, posx=-0.04)
        if self.get_solution(drag_paper_pose):
            if self._verbose:
                print("lowering paper...")
            self.move_to_solution()
        else: 
            return False
        #dragging end point
        alter_pose_inc(place_paper_pose, verbose=self._verbose, posx=0.03, posy=0.01)
        if self.get_solution(place_paper_pose):
            if self._verbose:
                print("dragging paper...")
            self.move_to_solution()
        else: 
            return False
        #place point
        alter_pose_inc(place_paper_pose, verbose=self._verbose, posx=-0.03, posy=-0.01)
        if self.get_solution(place_paper_pose):
            if self._verbose:
                print("approaching target...")
            self._limb.set_joint_position_speed(0.15)
            self.move_to_solution()
            print("------------> place_paper_pose: {}").format(self._limb.endpoint_pose())
            #open gripper
            self._gripper.open()
        else: 
            return False
        #retreat
        if self.get_solution(safe_pose):
            if self._verbose:
                print("retreating to safe position...")
            self.move_to_solution()
            self._limb.set_joint_position_speed(0.3)
            return True
        else:
            return False

    def take_tool(self):
        """
        Picks up the tool from a hardcoded position.

            Return:
                    False ->    No valid joint solution could be found for the given pose
                    True ->     The arm reached the given pose
        """
        if not self._gripper._type is 'electric':
            print("The tool can only be used with an electric gripper\nThe currently used gripper of {}_arm is {}").format(self._limb_name, self._gripper._type)
            return False
        if not self.pick(convert_to_pose(const_lib.tool_pose), opening=const_lib.gripper_opening, hover_distance=const_lib.hover_distance['tool']):
            print("Couldn't take tool")
            return False
        self.set_neutral(open_gripper=False)
        return True

    def store_tool(self):
        """
        Places the tool at a hardcoded position.

            Return:
                    False ->    No valid joint solution could be found for the given pose
                    True ->     The arm reached the given pose
        """
        self._gripper.command_position(const_lib.gripper_opening)
        tool_pose = convert_to_pose(const_lib.tool_pose)
        alter_pose_inc(tool_pose, verbose=self._verbose, posx=-0.002, posy=-0.002, posz=0.03)
        if not self.place(tool_pose, hover_distance=const_lib.hover_distance['tool']):
            print("Couldn't store tool")
            return False
        self.set_neutral()
        return True

    def simple_failsafe(self, open_gripper=True):
        """
        Moves the arm to a neutral pose and disables the robot afterwards.

        For setting both arms neutral please use the global function failsafe().

            Parameters:
            open_gripper: Flag to determine if the gripper shall be opened after reaching a neutral pose    
        """
        print("Exit routine started \nShutting down arm in neutral pose")
        self.set_neutral(open_gripper)
        self._rs.disable()