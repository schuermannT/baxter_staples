#!/usr/bin/env python

import rospy
hdr = Header(stamp=rospy.Time.now(), frame_id='base')
start_poses = {
                'left': PoseStamped(
                    header=hdr,
                    pose=Pose(
                        position=Point(
                            x=0.657579481614,
                            y=0.851981417433,
                            z=0.0388352386502,
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
                    header=hdr,
                    pose=Pose(
                        position=Point(
                            x=0.657579481614,
                            y=0.851981417433,
                            z=0.0388352386502,
                        ),
                        orientation=Quaternion(
                            x=-0.000,
                            y=0.999,
                            z=0.000,
                            w=0.000,
                        ),
                    ),
                )
            }
