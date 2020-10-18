#!/usr/bin/env python

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
)

table_height = {'suction':-0.175, 'electric':-0.197}    #length from baseframe of baxter to table. It differs depending on the gripper type

gripper_base_offset = 1.0                               #attachement point for gripper is at 1 meter height when posx is 0

hover_distance = {'paper':0.38, 'tool':0.1}             #distance between gripper/tool and table

gripper_opening = 68.0                                  #opening percentage for gripper to gently grab the tool 

#Poses
tool_pose = {'position': Point(x=0.507, y=0.003, z=-0.194), 
             'orientation': Quaternion(x=0.000, y=0.999, z=0.000, w=0.000)}
staple_pose = {'position': Point(x=0.405, y=0.108, z=-0.204), 
               'orientation': Quaternion(x=0.000, y=0.999, z=0.000, w=0.000)}

pick_paper_pose = {'position': Point(x=0.660, y=-0.150, z=-0.173), 
                   'orientation': Quaternion(x=0.000, y=0.999, z=0.000, w=0.000)}
place_paper_pose = {'position': Point(x=0.420, y=0.137, z=-0.18), 
                    'orientation': Quaternion(x=0.999, y=0.012, z=0.000, w=0.000)}