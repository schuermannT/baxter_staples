#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)

start_poses = {
                'left': PoseStamped(
                    pose=Pose(
                        position=Point(
                            x=0.405,
                            y=0.108,
                            z=0.000,
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
                            x=0.400,
                            y=-0.250,
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
