#!/usr/bin/env python
"""
Created on Sat May 16 21:41:48 2020

@author: kartik
"""


from arm_tracking_planner_executer import Robot
import numpy as np
from pose_tracker import PoseTracker
from environment import Environment
from pid_control import PID
import rospy,std_msgs
from arm_tracking.msg import PidUpdate,PidParam
#inmport pose_estimation

class ArmPID(object):
    def __init__(self,robot, kp,kd,ki,axes, error_thres, pose_tracker):
    #initialise pid controller 
    #axes = list [0,1,0] #list of size 3 (x,y,z) representing axes along which control is required

        self.robot = robot
        self.pid = PID(kp,kd,ki)
        self.pid_param = PidParam(kp,ki,kd)
        self.pose_tracker = pose_tracker
        self.axes = axes
        self.error_thresh = error_thres
        self.jac_step = 0.002
        self.pid_update_pub = rospy.Publisher('/arm_tracking/pid_update_log',PidUpdate,queue_size=10)
        
    #execute pid for 
    
    def trajectory_pid(self,target_trajectory):
        for target in target_trajectory:
            waypoints=[]
            waypoints.append((self.robot.group.get_current_pose().pose))
            waypoints.append(target)
            #first move from current pos to the target pos along a straight line
            pre_plan = self.robot.get_plan_move_along_line(waypoints)
            self.robot.execute_trajectory(pre_plan)
            #execute pid control to exactly reach target
#            self.point_pid_jacobian(target)
            self.point_pid_cartesian(target)
            
#    do pid control for point based on jacobian
    def point_pid_jacobian(self,target):
        
        error_trans = self.get_error(target)

        while(np.linalg.norm(error_trans) > self.error_thresh):
#            delta_trans = self.pid(error_trans)
            delta_trans = error_trans
            joint_angle = self.robot.group.get_current_joint_values()
    
            jacob = self.robot.group.get_jacobian_matrix(joint_angle)[:3]
            delta_angle = np.linalg.lstsq( jacob, delta_trans, rcond = None )[0]
            org_joint_angle = joint_angle[:]
            for i,ang in enumerate(joint_angle):
                joint_angle[i] = ang + self.jac_step*delta_angle[i]
            print(org_joint_angle,joint_angle)            
            plan = self.robot.get_plan_move_to_joint(joint_angle)
            
            self.robot.execute_trajectory(plan,True)
            error_trans = self.get_error(target)

#    do pid control for point based on cartesian control
    def point_pid_cartesian(self,target):
        
        error_trans = self.get_error(target)

        while(np.linalg.norm(error_trans) > self.error_thresh):
            
            
            
            update_msg = PidUpdate()
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            update_msg.header = h
            update_msg.pid_param = self.pid_param
            
            pid_update = self.pid.Update(error_trans)
            delta_trans = pid_update[0]
            update_msg.error = pid_update[1]
            update_msg.integ_error = pid_update[2]
            update_msg.delta_error = pid_update[3]
            update_msg.response = pid_update[0]
#            update_msg.respone = delta_trans
#            delta_trans = error_trans
            self.pid_update_pub.publish(update_msg)
            
            current_pose = self.robot.group.get_current_pose().pose            
            new_pose = current_pose
            new_pose.position.x += delta_trans[0]
            new_pose.position.y += delta_trans[1]
            new_pose.position.z += delta_trans[2]
            
            plan = self.robot.get_plan_move_along_line([current_pose,new_pose])
            
            self.robot.execute_trajectory(plan,True)
            error_trans = self.get_error(target)


    #returns 3d vector showing error along each axes    
    def get_error(self,target):
        #get pose as array 
        robot = self.pose_tracker.get_robot_ef_position()
#        robot = self.pose_tracker.get_robot_pose()
        
        error = target - robot[:,0]
        for index,axis in enumerate(self.axes):
            if axis == 0:
                error[index] = 0
        
        return error
       
    def execute(self):
        self.robot.set_planner_type('RRT')
        poses = [[0.1, -0.63, 0.3, 0, 180, 0],[0.1, -0.43, 0.4, 0, 180, 0],[-0.1, -0.63, 0.2, 0, 180, 0]]
        plan = self.robot.get_plan_move_to_goal(poses[0])
        self.robot.execute_trajectory(plan,wait=True) 

    
    
if __name__ == '__main__':
    robot = Robot('kinova')
    env = Environment()    
    pose_track = PoseTracker(robot,env)
    arm_pid = ArmPID(robot,0,0,0,{0,0,1},0.02,pose_track)
    arm_pid.execute()