#!/usr/bin/env python
#-*- coding:utf-8 -*-

import struct
import sys
import time

import const_lib

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
        """changes the current pose with the given absolute values"""
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
        pose.position.x = posx
        pose.position.y = posy
        pose.position.z = posz
        pose.orientation.x = orx
        pose.orientation.y = ory
        pose.orientation.z = orz
        pose.orientation.w = orw
        if verbose:
            print("--- new pose: ---")
            print(pose)
            print("------------------------")
        return pose

def alter_pose_inc(pose, verbose=False, posx = 0.0, posy = 0.0, posz = 0.0, orx = 0.0, ory = 0.0, orz = 0.0, orw = 0.0):
    """changes the current pose with the given modifiers. the modifiers are defaulted to 0"""
    if verbose:
        print("--- func: alter_pose_inc ---")
        print("posx={} posy={} posz={} orx={} ory={} orz={} orw={}").format(posx, posy, posz, orx, ory, orz, orw) 
        print("--- given pose: ---")
        print(pose)
    pose.position.x += posx 
    pose.position.y += posy
    pose.position.z += posz
    pose.orientation.x += orx
    pose.orientation.y += ory
    pose.orientation.z += orz
    pose.orientation.w += orw
    if verbose:
        print("--- new pose: ---")
        print(pose)
        print("------------------------")
    return pose

def convert_to_pose(pose_dict):
    """converts a dictionary to a Pose()"""
    ik_pose = Pose()
    ik_pose.position.x = pose_dict['position'].x 
    ik_pose.position.y = pose_dict['position'].y 
    ik_pose.position.z = pose_dict['position'].z
    ik_pose.orientation.x = pose_dict['orientation'].x 
    ik_pose.orientation.y = pose_dict['orientation'].y 
    ik_pose.orientation.z = pose_dict['orientation'].z 
    ik_pose.orientation.w = pose_dict['orientation'].w
    return ik_pose

def failsafe(left, right, tool=False):
    print("Exit routine started\nStoring Tool (if attached) and moving arms to neutral poses")
    right.set_neutral()
    if tool:
        left.store_tool()
    else:
        left.set_neutral()
    left._rs.disable()


class Arm(object):
    def __init__(self, limb, verbose=False):
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
            return 0

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
            return 1
        else:
            print ("INVALID POSE - No Valid Joint Solution Found.")
        return 0

    def move_to_solution(self):
        print("moving %s arm..." %self._limb_name)
        self._limb.move_to_joint_positions(self._ik_solution)
        self._current_pose = convert_to_pose(self._limb.endpoint_pose())

    def set_neutral(self, open_gripper=True):
        self._limb.move_to_neutral()
        if open_gripper:
            self._gripper.open()
        self._current_pose = convert_to_pose(self._limb.endpoint_pose())

    def pick(self, pick_pose, remove_staple=False, remove_clip=False, opening=0, hover_distance=0.1):
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

    def place(self, place_pose, opening=0, hover_distance=0.1):
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
            #open gripper
            if opening is 0:
                opening=100
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
        if not self._gripper._type is 'electric':
            print("The tool can only be used with an electric gripper\nThe currently used gripper of {}_arm is {}").format(self._limb_name, self._gripper._type)
            return False
        if not self.pick(convert_to_pose(const_lib.tool_pose), opening=const_lib.gripper_opening, hover_distance=const_lib.hover_distance['tool']):
            print("Couldn't take tool")
            return False
        self.set_neutral(open_gripper=False)
        return True

    def store_tool(self):
        self._gripper.command_position(const_lib.gripper_opening)
        tool_pose = convert_to_pose(const_lib.tool_pose)
        alter_pose_inc(tool_pose, verbose=self._verbose, posx=-0.002, posy=-0.002, posz=0.03)
        if not self.place(tool_pose, hover_distance=const_lib.hover_distance['tool']):
            print("Couldn't store tool")
            return False
        self.set_neutral()
        return True

    def simple_failsafe(self):
        print("Exit routine started \nShutting down arm in neutral pose")
        self.set_neutral()
        self._rs.disable()