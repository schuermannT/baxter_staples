#!/usr/bin/env python

import argparse
import struct
import sys

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

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



class IK_Test(object):
    def __init__(self, limb, verbose=False):
        self._limb_name = limb
        self._verbose = verbose
        self._limb = baxter_interface.limb.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        self._ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(self._ns, SolvePositionIK)
        self._ikreq = SolvePositionIKRequest()
        self._hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        
        print("Getting robot state...")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        if not self._init_state:
            print("Enabling robot...")
            self._rs.enable()

    def get_solution(self):
        pose = {
            'left': PoseStamped(
                header=self._hdr,
                pose=Pose(
                    position=Point(
                        x=0.657579481614,
                        y=0.851981417433,
                        z=0.0388352386502,
                    ),
                    orientation=Quaternion(
                        x=-0.366894936773,
                        y=0.885980397775,
                        z=0.108155782462,
                        w=0.262162481772,
                    ),
                ),
            ),
        }
        print pose['left']
        self._ikreq.pose_stamp.append(pose['left'])
        try:
            rospy.wait_for_service(self._ns, 5.0)
            resp = self._iksvc(self._ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" %(e,))
            return 1

        #Check if result is valid, and type of seed ultimately used to get solution
        #convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                self._ikreq.SEED_USER: 'User Provided Seed',
                self._ikreq.SEED_CURRENT: 'Current Joint Angles',
                self._ikreq.SEED_NS_MAP : 'Nullspace Setpoints',
            }.get(resp_seeds[0], 'None')
            print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" % (seed_str,))
            #Format solution into Limb API-compatible dictionary
            self._ik_solution = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print "\nIK Solution:\n", self._ik_solution
                print "--------------------------"
                print "Response Message:\n", resp
        else:
            print "INVALID POSE - No Valid Joint Solution Found."
        return 0

    def move_to_solution(self):
        self._limb.move_to_joint_positions(self._ik_solution)

    def set_neutral(self):
        baxter_interface.limb.Limb("left").move_to_neutral()
        baxter_interface.limb.Limb("right").move_to_neutral()

def main():
    try:
        # Argument Parsing
        arg_fmt = argparse.RawDescriptionHelpFormatter
        parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
        parser.add_argument(
            '-l', '--limb',
            choices=['left', 'right'],
            required=True,
            help="the limb to use"
        )
        parser.add_argument(
            '-v', '--verbose',
            action='store_const',
            const=True,
            default=False,
            help="displays debug information (default = false)"
        )
        args = parser.parse_args(rospy.myargv()[1:])

        # Initialize Node
        rospy.init_node("ik_movetest", anonymous=True)

        print "Ctrl-D stops the program"
        print "Press 'Enter' to initialize the %s arm ..." % args.limb
        raw_input()
        arm = IK_Test(args.limb, args.verbose)

        print "Press 'Enter' to move to neutral position ..."
        raw_input()
        arm.set_neutral()

        print "Press 'Enter' to search for solution ..."
        raw_input()
        arm.get_solution()

        print "Press 'Enter' to execute found solution ..."
        raw_input()
        arm.move_to_solution()

        print "IK Move Test finished."
    except rospy.ROSInterruptException as e:
        return e
    except KeyboardInterrupt as e:
        return e

if __name__ == '__main__':
    main()