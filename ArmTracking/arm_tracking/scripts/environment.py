#!/usr/bin/env python
"""
Created on Sun May 17 13:43:03 2020

@author: kartik
"""


class Environment():
    def __init__(self, robot ):
        self.robot = robot
        
    def add_all_objects(self):
        self.add_workpiece()
        self.add_backwall()
        self.add_table()
    
    def add_workpiece(self):
        self.workpiece_size = [0.3, 0.10, 0.10]
        #x,y,z
        self.workpiece_pose = [0.66, 0, 0.1, 1.0]
        self.workpiece_name = "workpiece"
        self.robot.get_and_add_box(self.workpiece_size,self.workpiece_pose,self.workpiece_name)
        
    def add_backwall(self):
        self.backwall_size = [0.01,2,2]
        self.backwall_pose = [-0.25,0,0.5,0]
        self.backwall_name = "backwall"
        self.robot.get_and_add_box(self.backwall_size,self.backwall_pose,self.backwall_name)
    
    def add_table(self):
        self.table_size = [2,2,0.01]
        self.table_pose = [0,0,0.01,0]
        self.table_name = "table"
        self.robot.get_and_add_box(self.table_size,self.table_pose,self.table_name)

    
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
