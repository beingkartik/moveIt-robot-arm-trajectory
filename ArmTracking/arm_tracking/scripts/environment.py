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
        
    def get_workpiece_edge(self):
        pass
        #rospy.Subscriber('arm_tracking/pose_tracking',TrackedPose,self.get_tracked_pose)
#float64[] robot_base_rvec
#float64[] robot_base_tvec
#float64[] robot_ef_rvec
#float64[] robot_ef_tvec
#float64[] workpiece_rvec
#float64[] workpiece_tvec
##workpiece marker corner x coordinates
#float64[] workpiece_corners_x 
##workpiece marker corner y coordinates
#float64[] workpiece_corners_y 
