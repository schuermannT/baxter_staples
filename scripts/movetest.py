#!/usr/bin/env python

import rospy

from std_msgs.msg import UInt16

import baxter_interface
from baxter_interface import CHECK_VERSION




class Movetest(object):
    def __init__(self):
        """
        Moves both arms in some way.
        """
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._left_arm = baxter_interface.limb.Limb("left")
        self._right_arm = baxter_interface.limb.Limb("right")
        self._left_gripper = baxter_interface.Gripper("left")
        self._left_joint_names = self._left_arm.joint_names()
        self._right_joint_names = self._right_arm.joint_names()

        # set gripper parameters
        self._left_gripper.set_velocity(20)

        # control parameters
        self._rate = 500.0  # Hz

        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # set joint state publishing to 500Hz
        self._pub_rate.publish(self._rate)

    def _reset_control_modes(self):
        rate = rospy.Rate(self._rate)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._left_arm.exit_control_mode()
            self._right_arm.exit_control_mode()
            self._pub_rate.publish(100)  # 100Hz default joint state rate
            rate.sleep()
        return True

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose.
        """
        print("Moving to neutral pose...")
        self._left_arm.move_to_neutral()
        self._right_arm.move_to_neutral()

    def clean_shutdown(self):
        print("\nExiting example...")
        # return to normal
        self._reset_control_modes()
        self.set_neutral()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

    def open_gripper(self):
        self._left_gripper.open()
        print("Opening Gripper")
        return True

    def close_gripper(self):
        self._left_gripper.close(1.0)
        print("Closing Gripper")
        return True

    def move(self):
        self.set_neutral()
        """
        Performs Movement.
        """
        pos1_left = {'left_w0': -0.7432136917304829, 'left_w1': 1.4522963109305154, 'left_w2': -0.3781262642137545,
                     'left_e0': -0.0674951546669582, 'left_e1': 0.9652574107768966, 'left_s0': -0.6208787238966212,
                     'left_s1': -0.8433059381400062}
        pos1_right = {'right_s0': 0.9759952762920945, 'right_s1': -0.08551942892461181, 'right_w0': -0.15378157398551273,
                      'right_w1': 1.2110778320355342, 'right_w2': 0.20210196880390327, 'right_e0': 0.07171360183364309,
                      'right_e1': 0.4348835533655148}
        print("Moving to Position 1...")
        self._left_arm.move_to_joint_positions(pos1_left)
        self._right_arm.move_to_joint_positions(pos1_right)
        print("Movetest finished...")

    def print_angles(self):
        left_angles = self._left_arm.joint_angles()
        right_angles = self._right_arm.joint_angles()
        print left_angles
        print right_angles


def main():
    print("Initializing node... ")
    rospy.init_node("movetest")

    #print baxter_interface.limb.Limb("left").joint_angles()
    print baxter_interface.limb.Limb("right").joint_angles()

    #movetest = Movetest()
    #movetest.set_neutral()

    #raw_input("==== Press Enter for next step")
    #movetest.print_angles()
    # movetest.move()

    print("Done.")


if __name__ == '__main__':
    main()
