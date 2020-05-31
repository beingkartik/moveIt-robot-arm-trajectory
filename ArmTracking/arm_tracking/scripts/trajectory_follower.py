#!/usr/bin/env python
"""
Created on Sun May 17 14:46:41 2020

@author: kartik
"""
from __future__ import print_function

from arm_tracking_planner_executer import Robot
from environment import Environment
from pose_tracker import PoseTracker
from arm_pid import ArmPID
import numpy as np
import rospy
import matplotlib.pyplot as plt
import time


if __name__ == '__main__':
    robot = Robot('kinova')
    robot.set_planner_type('RRT')
    env = Environment()    
    pose_track = PoseTracker(robot,env)
    
    time.sleep(2)
    while(1):
        if pose_track.calibrate_transforms():
            break    
    robot.get_and_add_box(env.workpiece_size,env.workpiece_pose,env.workpiece_shape)
    
    #robot,kp,kd,ki,axes, error_thres, pose_tracker
    #axes = [1,0,0] means tracking done only along first axis
    #axes = [1,1,1] means tracking done for all 3 axis
    #axes is in robot base frame 
    pid_controller = ArmPID(robot,0.1,0,0,[1,0,0],0.01,pose_track)
#    target_trajectory = pose_track.get_workpiece_edge()
    target_trajectory = env.get_workpiece_edge()
    
    for target in target_trajectory:
        
        print(target)
        waypoints = []        
        waypoints.append((robot.group.get_current_pose().pose))
        
        ##mention target orientation in the last 3 entries here
        target_pose = [0,0,0,0,180,0]
        target_pose[:3] = target
        waypoints.append(target_pose)
            
        pid_controller.point_pid_cartesian(target)
#        plan = robot.get_plan_move_along_line(waypoints)
#        robot.execute_trajectory(plan)
#        rospy.loginfo(pid_controller.get_error(target))

    