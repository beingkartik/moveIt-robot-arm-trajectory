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

if __name__ == '__main__':
    robot = Robot('kinova')
    robot.set_planner_type('RRT')
    env = Environment()    
    pose_track = PoseTracker(robot,env)

    robot.get_and_add_box(env.workpiece_size,env.workpiece_pose,env.workpiece_shape)
    
    pid_controller = ArmPID(robot,0.1,0,0,[1,0,0],0.01,pose_track)
    target_trajectory = pose_track.get_workpiece_edge()
    for target in target_trajectory:
        
        print(target)
        waypoints = []        
        waypoints.append((robot.group.get_current_pose().pose))
        target_pose = [0,0,0,0,180,0]
        target_pose[:3] = target
        waypoints.append(target_pose)
            
        pid_controller.point_pid_cartesian(target)
#        plan = robot.get_plan_move_along_line(waypoints)
#        robot.execute_trajectory(plan)
        rospy.loginfo(pid_controller.get_error(target))
    

#    
#    poses = [[-0.2, -0.63, 0.3, 0, 180, 0],[0.1, -0.43, 0.4, 0, 180, 0],[-0.15, -0.63, 0.2, 0, 180, 0]]
#    plan = robot.get_plan_move_to_goal(poses[0])
#    robot.execute_trajectory(plan,wait=True) 
#    plan = robot.get_plan_move_to_goal(poses[2])
#    robot.execute_trajectory(plan,wait=True) 
    
