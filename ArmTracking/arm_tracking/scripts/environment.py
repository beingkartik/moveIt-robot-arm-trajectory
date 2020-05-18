#!/usr/bin/env python
"""
Created on Sun May 17 13:43:03 2020

@author: kartik
"""


class Environment():
    def __init__(self, robot_name = 'kinova'):
        self.robot_name = robot_name
        self.def_workpiece()
    
    def def_workpiece(self):
        self.workpiece_size = [0.3, 0.10, 0.10]
        self.workpiece_pose = [0.0, -0.66, 0.1, 1.0]
        self.workpiece_shape = "cube"
        
