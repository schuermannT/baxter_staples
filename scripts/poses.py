#!/usr/bin/env python

neutral = {
    'left' : {'left_w0': 0.67, 
                'left_w1': 1.03, 
                'left_w2': -0.50,
                'left_e0': -1.19, 
                'left_e1': 1.94, 
                'left_s0': -0.08,
                'left_s1': -1.0},
    'right' : {'right_s0': 0.08, 
                'right_s1': -1.0, 
                'right_w0': -0.67,
                'right_w1': 1.03, 
                'right_w2': 0.50, 
                'right_e0': 1.19,
                'right_e1': 1.94}
}

table_height = -0.175 #length from baseframe of baxter to table

gripper_base_offset = 1.0 #attachement point for gripper is at 1 meter height when posx is 0

hover_distance = 0.07