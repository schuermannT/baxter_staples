#!/usr/bin/env python

import argparse
import sys

import rospy
import copy
import baxter_interface
from baxter_interface import CHECK_VERSION
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list


"""
{'left_w0': 0.014956312681882784, 'left_w1': 1.2536457988993543, 'left_w2': -0.011504855909140603, 'left_e0': -0.02339320701525256, 'left_e1': 0.7719758315033345, 'left_s0': -0.0030679615757708274, 'left_s1': -0.558752501987262}
{'right_s0': 0.0030679615757708274, 'right_s1': -0.5595194923812047, 'right_w0': -0.013038836697026017, 'right_w1': 1.2567137604751253, 'right_w2': 0.010354370318226542, 'right_e0': 0.022626216621309852, 'right_e1': 0.7727428218972772}
"""


def all_close(goal, actual, tolerance):
    """
    Method for testing if a list of values are within a tolerance of their counterparts in another list.
    @param: goal	A list of floats, a Pose or a PoseStamped
    @param: actual	A list of floats, a Pose or a PoseStamped
    @param: tolerance	A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveItTest(object):
    def __init__(self, limb, verbose=False):
        # Robot Init
        print("Getting robot state...")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot...")
        self._rs.enable()

        # MoveIt-Init
        # Dokumentation hierzu:
        # github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface_tutorial.py
        print "Initializing MoveIt-Commander..."
        moveit_commander.roscpp_initialize(sys.argv)

        # RobotCommander -> provides Information about robot
        print "Initializing Robot-Commander..."
        self.robot = moveit_commander.RobotCommander()

        # Scene = virtual surrounding of the robot
        print "Initializing Scene"
        self.scene = moveit_commander.PlanningSceneInterface()

        # Instantiate MoveGroupCommander -> Interface to a planning group
        self.group_name = baxter_interface.limb.Limb(limb).name + "_arm"
        if verbose: print "Movegroup: %s" % self.group_name
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # Display trajectories in Rviz
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

        # Getting basic informations
        self.planning_frame = self.move_group.get_planning_frame()  # get name of reference frame
        if verbose: print "==== Planning frame: %s" % self.planning_frame
        self.eef_link = self.move_group.get_end_effector_link()  # get name of ende-effector link for this group
        if verbose: print "==== End effector link: %s" % self.eef_link
        self.group_names = self.robot.get_group_names()  # get list of all groups on this robot
        if verbose: print "==== Available Planning Groups:", self.robot.get_group_names()
        if verbose:  # print entire robot state for debugging
            print"====Printing robot state"
            print self.robot.get_current_state()
            print ""

        # Misc variables
        self.box_name = ''
        self.verbose = verbose

    def go_to_joint_state(self):
        """Planning and executing movement to a joint state goal"""
        joint_goal = self.move_group.get_current_joint_values()
        if self.verbose: print joint_goal
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0
        joint_goal[6] = 0
        if self.verbose: print joint_goal

        """
        {'left_w0': 0.014956312681882784, 'left_w1': 1.2536457988993543, 'left_w2': -0.011504855909140603, 'left_e0': -0.02339320701525256, 'left_e1': 0.7719758315033345, 'left_s0': -0.0030679615757708274, 'left_s1': -0.558752501987262}
        {'right_s0': 0.0030679615757708274, 'right_s1': -0.5595194923812047, 'right_w0': -0.013038836697026017, 'right_w1': 1.2567137604751253, 'right_w2': 0.010354370318226542, 'right_e0': 0.022626216621309852, 'right_e1': 0.7727428218972772}
        """
        self.move_group.set_planning_time(50)
        self.move_group.go(joint_goal,
                           wait=True)  # can be called with joint values, poses, or without parameter if goal is already set
        self.move_group.stop()

        current_joints = self.move_group.get_current_joint_values()
        if self.verbose: print current_joints
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self):
        """Planning and executing movement to a pose goal"""

        # define desired pose for the end-effector
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        self.move_group.set_pose_target(pose_goal)

        plan = self.move_group.go(wait=True)  # compute plan to goal an execute it
        self.move_group.stop()  # to terminate residual movement
        self.move_group.clear_pose_targets()  # clear targets after planning with poses
        # (there's no equivalent for joint-values)

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        """Plan (not execute) a cartesian path by specifying a list of waypoints for end-effector to go through"""
        waypoints = []
        wpose = self.move_group.get_current_pose().pose
        if self.verbose: print "current pose: %s" % wpose
        wpose.position.z -= scale * 0.1  # move up
        wpose.position.y += scale * 0.1  # move sideways
        if self.verbose: print "planned pose: %s" % wpose
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # move for-/backwards
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # move sideways
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step -> resolution (in this case = 1cm)
            0.0  # jump_treshold -> here disabled
        )

        return plan, fraction

    def display_trajectory(self, plan):
        """Publish the trajectory, for Rviz to visualize"""
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        """Executes the given plan. This needs to be already computed"""
        self.move_group.execute(plan, wait=True)
        # current joint state needs to be in within some tolerance of first waypoint


def set_neutral():
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
        nodename = "moveittest"
        if args.verbose: print "Node: %s" % nodename
        rospy.init_node(nodename, anonymous=True)

        print "Ctrl-D stops the program"
        print "Press 'Enter' to initialize the %s arm ..." % args.limb
        raw_input()
        arm = MoveItTest(args.limb, args.verbose)

        print "Press 'Enter' to move to neutral position ..."
        raw_input()
        set_neutral()

        print "Press 'Enter' to move to specified joint values ..."
        raw_input()
        arm.go_to_joint_state()

        print "Press 'Enter' to plan and display a Cartesian path ..."
        raw_input()
        cartesian_plan, fraction = arm.plan_cartesian_path()

        print "Press 'Enter' to display a saved trajectory ..."
        raw_input()
        arm.display_trajectory(cartesian_plan)

        print "Press 'Enter' to execute saved path ..."
        raw_input()
        arm.execute_plan(cartesian_plan)

        print "MoveItTest finished."
    except rospy.ROSInterruptException as e:
        return e
    except KeyboardInterrupt as e:
        return e


if __name__ == '__main__':
    main()
