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
from arm_tracking.msg import PidUpdate

def pid_update_log(pid_update_msg,trajectory_error_log):
    trajectory_error_log.append(pid_update_msg.error)

#given previous run of trajectory, use that to update the trajectory points for next time
def update_trajectory(target_trajectory,trajectory_error_log,first_point_rframe):
    first_point_rframe += trajectory_error_log[0]    
    for i in range(len(target_trajectory)):
        target_trajectory[i] += trajectory_error_log[i+1]
    for  i in range(len(trajectory_error_log)):
        trajectory_error_log.pop()
    
if __name__ == '__main__':
    robot = Robot('kinova','real')
    robot.set_planner_type('RRT')
    env = Environment(robot)    
    pose_track = PoseTracker(robot,env, image_topic='/usb_cam/image_raw')
    time.sleep(2)
    
    trajectory_error_log = []
    pid_update_log_sub = rospy.Subscriber('/arm_tracking/pid_update_log',PidUpdate,pid_update_log,trajectory_error_log)    
    while(1):
        if pose_track.calibrate_transforms(load = True):
            rospy.loginfo("Done Calibrating")
            break    
#    
    
    #robot,kp,kd,ki,axes, error_thres, pose_tracker
    #axes = [1,0,0] means tracking done only along first axis
    #axes = [1,1,1] means tracking done for all 3 axis
    #axes is in robot base frame 
    num_of_points_traj  = 10
    pid_controller = ArmPID(robot,1,0,0,[0,1,1],0.01,pose_track)
    if robot.virtual_or_real == 'virtual':
        #define the workpiece
        env.add_workpiece()
        target_trajectory = pose_track.get_workpiece_edge(num_of_points_traj)
        end_effector_orientation = [0,180,0]
        first_point_rframe  = np.array([0.203678965569,-0.253386229277,0.155322819948])

        
    elif robot.virtual_or_real == 'real':
#        first_point_rframe  = np.array([0.203678965569,-0.253386229277,0.155322819948])
#
#        start_position_tvec = [0.12132009418469955, -0.11700861386993623, 0.5636463963589043]
#        final_position_tvec = [-0.1465526466706532, -0.11202300671933697, 0.5007474329585156]

        first_point_rframe  = np.array([0.0,-0.316760659218,0.224803119898])

        start_position_tvec = [0.23970833027465058, -0.17373478045301092, 1.00908767612386]
        final_position_tvec = [0.024123448287631223, -0.1736568186761976, 1.0053317296619972]
        
#        first_point_rframe  = np.array([0.0,-0.316760659218,0.254803119898])
#
#        start_position_tvec = [0.23970833027465058, -0.24673478045301092, 1.00908767612386]
#        final_position_tvec = [0.024123448287631223, -0.24476568186761976, 1.0053317296619972]
#

        #for usb camera
        first_point_rframe  = np.array([0.0636801421642,-0.294865041971,0.220806658268])

        start_position_tvec = [0.4753038164648825, 0.2168884053488654, 0.689539731129068]
        final_position_tvec = [0.36019145350636145, 0.19266412708556974, 0.7355971098470163]
        final_position_tvec = [0.3653038164648825, 0.2168884053488654, 0.689539731129068]
        
        start_position = pose_track.transform.get_pos_rframe(start_position_tvec)
        final_position = pose_track.transform.get_pos_rframe(final_position_tvec)
        
        target_trajectory = np.linspace(start_position,final_position,num_of_points_traj)

#        end_effector_orientation = [-0.87322640419, -0.485389411449, 0.00657376274467, 0.0427713394165]
        end_effector_orientation = [0.810226976871,-0.584178268909,-0.0134347546846,0.0456880331039]
        
#    pid_controller.trajectory_point_wise_pid(target_trajectory,end_effector_orientation,first_point_rframe)
    pid_controller.trajectory_continuous_pid(target_trajectory,end_effector_orientation,first_point_rframe)
    time.sleep(0.2)
    
    
    ###ISSUE IN UPDATING TARGET TRAJECTORY! THE ORIGINAL TARGET TRAJECTORY IS BEING LOST. NEED TO CALCULATE ERROR WITH THE ORG TARGET TRAJECTORY, BUT MOVE WITH THE UPDATED TARGET TRAJECTORY
    
#    update_trajectory(target_trajectory,trajectory_error_log,first_point_rframe)
#    print("running again")
#    time.sleep(2)
#    
##    pid_controller.pid.Kp = 0
#    pid_controller.trajectory_continuous_pid(target_trajectory,end_effector_orientation,first_point_rframe)
#    time.sleep(0.2)
#    update_trajectory(target_trajectory,trajectory_error_log,first_point_rframe)
#    time.sleep(2)

#    print("running again")
#    
#    pid_controller.trajectory_continuous_pid(target_trajectory,end_effector_orientation,first_point_rframe)
#    time.sleep(0.2)
#    update_trajectory(target_trajectory,trajectory_error_log,first_point_rframe)
#    time.sleep(2)
#
#    print("running again")
#
#    pid_controller.trajectory_continuous_pid(target_trajectory,end_effector_orientation,first_point_rframe)