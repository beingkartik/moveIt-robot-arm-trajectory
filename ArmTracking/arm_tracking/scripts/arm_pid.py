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
import time
from geometry_msgs.msg import Pose

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
    
#    def trajectory_pid(self,target_trajectory):
#        for target in target_trajectory:
#            waypoints=[]
##            waypoints.append((self.robot.group.get_current_pose().pose))
#            waypoints.append((self.robot.get_end_effector_pose()))
#            waypoints.append(target)
#            #first move from current pos to the target pos along a straight line
#            pre_plan = self.robot.get_plan_move_along_line(waypoints)
#            self.robot.execute_trajectory(pre_plan)
#            #execute pid control to exactly reach target
##            self.point_pid_jacobian(target)
#            self.point_pid_cartesian(target)

#move to point 1, perform pid at point 1
#do repeat:
    #find (dx,dy,dz) = point (j+1) - point j
    #move current_pos + (dx,dy,dz)
    #do pid for point (j+1)

    def trajectory_pid_delta_control(self,target_trajectory,end_effector_orientation,first_point_rframe):
            
        #first move from current pos to the first point along a straight line
        target = target_trajectory[0]
        rospy.loginfo(target)

        target_pose= list(first_point_rframe) + end_effector_orientation
        plan = self.robot.get_plan_move_to_goal(target_pose)
        self.robot.execute_trajectory(plan)
        time.sleep(1)
        #do pid control to reach the first point
        self.point_pid_cartesian(target)
        
        prev_target = target
        
        for target in target_trajectory[1:]:
            
            rospy.loginfo(target)
            delta_trans = target - prev_target
            current_pose = self.robot.get_end_effector_pose()
            target_pose = self.robot.get_end_effector_pose()
            target_pose.position.x += delta_trans[0]
            target_pose.position.y += delta_trans[1]
            target_pose.position.z += delta_trans[2]
            
            for i in range(5):
                current_pose = self.robot.get_end_effector_pose()
                pre_plan = self.robot.get_plan_move_along_line([current_pose,target_pose])
                self.robot.execute_trajectory(pre_plan)
                time.sleep(1)
            #execute pid control to exactly reach target
#            self.point_pid_jacobian(target)
            self.point_pid_cartesian(target)
            prev_target = target
            
#    do pid control for point based on jacobian
#    def point_pid_jacobian(self,target):
#        
#        error_trans = self.get_error(target)
#
#        while(np.linalg.norm(error_trans) > self.error_thresh):
##            delta_trans = self.pid(error_trans)
#            delta_trans = error_trans
#            joint_angle = self.robot.group.get_current_joint_values()
#    
#            jacob = self.robot.group.get_jacobian_matrix(joint_angle)[:3]
#            delta_angle = np.linalg.lstsq( jacob, delta_trans, rcond = None )[0]
#            org_joint_angle = joint_angle[:]
#            for i,ang in enumerate(joint_angle):
#                joint_angle[i] = ang + self.jac_step*delta_angle[i]
#            print(org_joint_angle,joint_angle)            
#            plan = self.robot.get_plan_move_to_joint(joint_angle)
#            
#            self.robot.execute_trajectory(plan,True)
#            error_trans = self.get_error(target)

#    do pid control for point based on cartesian control

    
    def point_pid_cartesian(self,target):
        
        error_trans = self.get_error(target)
        next_pose = ''

        while(np.linalg.norm(error_trans) > self.error_thresh):
            
           
            pid_update = self.pid.Update(error_trans)
            
            delta_trans = pid_update[0]
            current_pose, next_pose = Pose(),Pose()
#            current_pose = self.robot.group.get_current_pose().pose 
            current_pose.position = self.robot.get_end_effector_pose().position
            current_pose.orientation = self.robot.get_end_effector_pose().orientation
            next_pose.orientation = self.robot.get_end_effector_pose().orientation
            next_pose.position.x += self.robot.get_end_effector_pose().position.x + delta_trans[0]
            next_pose.position.y += self.robot.get_end_effector_pose().position.y + delta_trans[1]
            next_pose.position.z += self.robot.get_end_effector_pose().position.z + delta_trans[2]
#            print(current_pose)
#            print(next_pose)
            plan = self.robot.get_plan_move_along_line([current_pose,next_pose])
            
            self.robot.display_trajectory(plan)
            time.sleep(0.5)
            self.robot.execute_trajectory(plan,True)
            time.sleep(1)
            self.publish_pid_update_msg(pid_update)
            error_trans = self.get_error(target)
            print("doing pid")
#            rospy.loginfo(error_trans)
        
        rospy.loginfo("Completed PiD for this point. Final Error ",)
        error_trans = self.get_error(target)
        rospy.loginfo(error_trans)
#                      str(next_pose.position.x)next_pose.position.y,next_pose.position.z))
        

    
    #returns 3d vector showing error along each axes    
    def get_error(self,target,robot = None):
        #get pose as array 
        if self.robot.virtual_or_real == "real":
            robot = self.pose_tracker.get_robot_ef_position()
        elif self.robot.virtual_or_real == "virtual":
            robot = self.pose_tracker.get_robot_pose()
#        robot = self.pose_tracker.get_robot_pose()
        error = target - robot
        for index,axis in enumerate(self.axes):
            if axis == 0:
                error[index] = 0
        return error
    
    #generates and publishes the pid_update message
    def publish_pid_update_msg(self,pid_update_output):
        update_msg = PidUpdate()
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        update_msg.header = h
        update_msg.pid_param = self.pid_param
        update_msg.error = pid_update_output[1]
        update_msg.integ_error = pid_update_output[2]
        update_msg.delta_error = pid_update_output[3]
        update_msg.response = pid_update_output[0]
        self.pid_update_pub.publish(update_msg)
            
    
if __name__ == '__main__':
    robot = Robot('kinova','virtual')
    env = Environment()    
    pose_track = PoseTracker(robot,env)
    arm_pid = ArmPID(robot,0,0,0,{0,0,1},0.02,pose_track)
    arm_pid.execute()