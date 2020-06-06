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
    robot = Robot('kinova','real')
    robot.set_planner_type('RRT')
    env = Environment()    
    env.add_all_objects()

    pose_track = PoseTracker(robot,env, image_topic='/camera/color/image_raw')
    
    time.sleep(2)
    while(1):
        if pose_track.calibrate_transforms(load = True):
            break    
    
    
    #robot,kp,kd,ki,axes, error_thres, pose_tracker
    #axes = [1,0,0] means tracking done only along first axis
    #axes = [1,1,1] means tracking done for all 3 axis
    #axes is in robot base frame 
    pid_controller = ArmPID(robot,1,0,0,[1,1,1],0.01,pose_track)
    target_trajectory = pose_track.get_workpiece_edge()
#    target_trajectory = env.get_workpiece_edge()
    
    #move to first point directly
    target_pose = [0,0,0,0,180,0]
    target_pose[:3] = target_trajectory[0]
        
    plan = robot.get_plan_move_to_goal(target_pose)        
    robot.execute_trajectory(plan)
    
    for target in target_trajectory[1:]:
#        target[-1] = 0.2
        print(target)
        waypoints = []        
#        waypoints.append((robot.group.get_current_pose().pose))
        waypoints.append((robot.get_end_effector_pose()))
        
        ##mention target orientation in the last 3 entries here
        target_pose = [0,0,0,0,180,0]
        target_pose[:3] = target
        
        waypoints.append(target_pose)
            
#        pid_controller.point_pid_cartesian(target)
        plan = robot.get_plan_move_along_line(waypoints)
        robot.display_trajectory(plan)
#        robot.execute_trajectory(plan,wait=True)
#        rospy.loginfo(pid_controller.get_error(target))

    