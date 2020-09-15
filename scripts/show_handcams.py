#!/usr/bin/env python

import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

import baxter_interface

class Camtest(object):
    def __init__(self, limb):
        sub_ns = "/cameras/{}_hand_camera/image".format(limb)
        self._limb = limb
        self.sub = rospy.Subscriber(sub_ns, Image, self.show_callback)
        self.bridge = CvBridge()
        print("Getting robot state...")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        if not self._init_state:
            print("Enabling robot...")
            self._rs.enable()
        else:
            print("Robot already enabled...")

    def show_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg)
            cv2.imshow(self._limb, img)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print("Bridge-Error: {}".format(e))

def main():
    print("Initializing node... ")
    rospy.init_node("camtest")

    left_cam = Camtest('left')
    #right_cam = Camtest('right')
    
    rospy.spin()
    print("Finished")

if __name__ == '__main__':
    main()
