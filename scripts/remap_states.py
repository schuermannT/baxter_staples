#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

goal_pos = 0;
pub = rospy.Publisher('joint_states', JointState, queue_size = 10)

def remap_callback(msg):
    pub.publish(msg)

def main():
    print "Initializing Node..."
    rospy.init_node('joint_control', anonymous = True)
    rospy.Subscriber('/robot/joint_states', JointState, remap_callback)
    print "---Ctrl-C to stop---"
    print "Running..."
    rospy.spin()

main()