#!/usr/bin/env python

"""
Created on Tue May 12 12:15:41 2020

@author: kartik
"""

import rospy
import moveit_commander
import sys, os, time
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
import tf, math
import pdb
from sensor_msgs.msg import JointState
#import numpy as np
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene

class Robot(object):
    def __init__(self,robot_name):
        #Initialise Moveit interface
        cleaned_args = [arg for arg in sys.argv if not os.path.basename(__file__) in os.path.basename(__file__)]
        moveit_commander.roscpp_initialize(cleaned_args)
        if robot_name == "kinova":
            rospy.init_node("kinova_move_group", anonymous = True)
            self.pub = rospy.Publisher("/if_node_finish", String, queue_size = 10)
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
            self.group = moveit_commander.MoveGroupCommander("arm")
            self.group.set_planning_time(5)
            self.gripper = moveit_commander.MoveGroupCommander("gripper")
            rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance',0.0)
            rospy.wait_for_service("/apply_planning_scene",10.0)
            rospy.wait_for_service("/get_planning_scene",10.0)
            self.aps = rospy.ServiceProxy("/apply_planning_scene",ApplyPlanningScene)
            self.gps = rospy.ServiceProxy("/get_planning_scene",GetPlanningScene)
            rospy.sleep(2)
            self.disp_traj_publisher = rospy.Publisher("/move_group/display_planned_path",moveit_msgs.msg.DisplayTrajectory, queue_size=20)
            self.disp_traj = moveit_msgs.msg.DisplayTrajectory()
            self.disp_traj.trajectory_start = self.robot.get_current_state()
#            self.arm_joint_sub = rospy.Subscriber("/joint_state",JointState, self.get_arm_joints)
#            self.curr_arm_joint_state = []
            self.arm_traj = []
            self.rate = rospy.Rate(10)
            self.group.allow_replanning(1)
            self.disp_complete_or_segment = 'segment'
        
    def get_arm_joints(self,msg):
        self.curr_arm_joint_state = msg
        
        
    def set_planner_type(self,planner_type):
        if planner_type == "RRT":
            self.group.set_planner_id("RRTConnectkConfigDefault")
        elif planner_type == "RRT*":
            self.group.set_planner_id("RRTstarkConfigDefault")
        elif planner_type == "PRM*":
            self.group.set_planner_if("PRMstarkConfigDefault")
    
    def get_pose_from_list(self, pose_as_type_list):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = pose_as_type_list[0]
        pose_goal.position.y = pose_as_type_list[1]
        pose_goal.position.z = pose_as_type_list[2]
        
        if len(pose_as_type_list) == 6:
            quat = tf.transformations.quaternion_from_euler(math.radians(pose_as_type_list[3]), math.radians(pose_as_type_list[4]), math.radians(pose_as_type_list[5]))
            pose_goal.orientation.x = quat[0]
            pose_goal.orientation.y = quat[1]
            pose_goal.orientation.z = quat[2]
            pose_goal.orientation.w = quat[3]
            
        elif len(pose_goal)==7:
            pose_goal.orientation.x = pose_as_type_list[3]
            pose_goal.orientation.y = pose_as_type_list[4]
            pose_goal.orientation.z = pose_as_type_list[5]
            pose_goal.orientation.w = pose_as_type_list[6]
        
        return pose_goal
        
    def get_plan_move_to_goal(self,ee_pose):
        pose_goal = self.get_pose_from_list(ee_pose)            
        self.group.set_pose_target(pose_goal)
        self.group.set_planning_time(20)
        plan = self.group.plan()
        rospy.sleep(2)
        return plan
        
    def get_plan_move_to_joint(self,joint_state):
        if type(joint_state) is JointState:
            joint_goal = joint_state
        elif type(joint_state) is list:
            joint_goal = self.robot.get_current_state().joint_state       
            joint_goal.position = joint_state
        self.group.set_joint_value_target(joint_goal)
        plan = self.group.plan()
        return plan

    def get_plan_move_along_line(self, waypoints):
        waypoints_type_pose = []
        for point in waypoints :
            if type(point) is geometry_msgs.msg.Pose:
                waypoints_type_pose.append(point)
            else:
                waypoints_type_pose.append(self.get_pose_from_list(point))
        plan, fraction = self.group.compute_cartesian_path(waypoints_type_pose,0.02,0.0)
        return plan
            
    def execute_trajectory(self, plan, wait = True):
        self.group.execute(plan, wait=wait)
        if wait : self.group.stop()
        self.group.clear_pose_targets()
        self.display_trajectory(plan)

    def display_trajectory(self, plan):
        
        if self.disp_complete_or_segment == "complete":
            self.disp_traj.trajectory.append(plan)
        elif self.disp_complete_or_segment == "segment":
            self.disp_traj.trajectory = []
            self.disp_traj.trajectory.append(plan)

            self.disp_traj.trajectory_start = self.robot.get_current_state()
            
        self.disp_traj_publisher.publish(self.disp_traj)
        
    def get_and_add_box(self,sizes,poses,shape):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.position.x = poses[0]
        box_pose.pose.position.y = poses[1]
        box_pose.pose.position.z = poses[2]
        box_pose.pose.orientation.w = poses[3]
        box_name = shape
        self.scene.add_box(box_name,box_pose, size = (sizes[0],sizes[1],sizes[2]))
        

            

        
if __name__ == '__main__':
    robot = Robot('kinova')
    robot.set_planner_type('RRT')
#    rospy.loginfo('Moving right')
#    robot.move_to_goal([0.1, -0.63, 0.2, 0, 180, 0])
#    rospy.loginfo('Moving left')
#    robot.move_to_goal([-0.1, -0.63, 0.2, 0, 180, 0])
#    rospy.loginfo('Moving up')
#    robot.move_to_goal([-0.1, -0.63, 0.3, 0, 180, 0])
    
    poses = [[0.1, -0.63, 0.3, 0, 180, 0],[0.1, -0.43, 0.4, 0, 180, 0],[-0.1, -0.63, 0.2, 0, 180, 0]]
    plan = robot.get_plan_move_to_goal(poses[2])
    robot.execute_trajectory(plan,wait=True) 
    t =  time.time()
#    print(t)    
    plan = robot.get_plan_move_to_goal(poses[0])
    robot.execute_trajectory(plan,wait=False) 
    rospy.sleep(0.5)
    robot.group.stop()
    plan = robot.get_plan_move_to_goal(poses[2])
    robot.execute_trajectory(plan,wait=True) 
    t =  time.time()
    
    
    

#    rospy.loginfo('Moving right up')
#    plan = robot.get_plan_move_to_goal([0.1, -0.63, 0.3, 0, 180, 0])
#    robot.execute_trajectory(plan)
#    rospy.loginfo('Moving up')
#    plan = robot.get_plan_move_to_goal([0.1, -0.43, 0.4, 0, 180, 0])
#    robot.execute_trajectory(plan)
#    rospy.loginfo('Moving Cartesian Path')
#    waypoints = []
#    waypoints.append((robot.group.get_current_pose().pose))
#    waypoints.append([-0.1, -0.63, 0.2, 0, 180, 0])
#    plan = robot.get_plan_move_along_line(waypoints)
#    robot.execute_trajectory(plan)
    
    rospy.spin()