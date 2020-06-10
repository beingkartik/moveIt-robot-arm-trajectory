#!/usr/bin/env python
"""
Created on Thu Jun  4 04:04:27 2020

@author: kartik
"""



from __future__ import print_function

from arm_tracking_planner_executer import Robot
from environment import Environment
#from pose_tracker import PoseTracker
#from arm_pid import ArmPID
import numpy as np
import rospy
import matplotlib.pyplot as plt
import time
from arm_tracking.msg import TrackedPose

#from tranforms import Transforms


class CalibrateTransforms():
    
    #if load == True, precalibrated data points are loaded
    def __init__(self,robot,load = True):
        self.load = load
        if not load:
            self.robot = robot
            self.pose_tracking_sub = rospy.Subscriber('arm_tracking/pose_tracking',TrackedPose,self.get_tracked_pose)
            
    def perform_calibration(self):        
        
        
        if self.load :
            camera_data_y = np.load('/home/kartik/arm_tracking/visuo_servoing/visuo_servo_ws/src/kinova-ros/ArmTracking/arm_tracking/data/camera_data_y.npy')
            camera_data_z = np.load('/home/kartik/arm_tracking/visuo_servoing/visuo_servo_ws/src/kinova-ros/ArmTracking/arm_tracking/data/camera_data_z.npy')
            
            y_trajectory = np.load('/home/kartik/arm_tracking/visuo_servoing/visuo_servo_ws/src/kinova-ros/ArmTracking/arm_tracking/data/y_trajectory.npy')
            z_trajectory = np.load('/home/kartik/arm_tracking/visuo_servoing/visuo_servo_ws/src/kinova-ros/ArmTracking/arm_tracking/data/z_trajectory.npy')
           
           
        else:
            
            num_of_points_traj = 10
            end_effector_orientation = [-0.87322640419, -0.485389411449, 0.00657376274467, 0.0427713394165]
    
            #trajectory along z
            start_position_z = [0.211836367846,-0.264596700668, 0.249753106833]
            final_position_z_traj = [0.211836367846,-0.264596700668, 0.154527920485]
            z_trajectory = np.linspace(start_position_z,final_position_z_traj,num_of_points_traj)

            camera_data_z = self.move_and_find_camera_line(z_trajectory,end_effector_orientation)    
            np.save('/home/kartik/arm_tracking/visuo_servoing/visuo_servo_ws/src/kinova-ros/ArmTracking/arm_tracking/data/camera_data_z.npy',camera_data_z)
            np.save('/home/kartik/arm_tracking/visuo_servoing/visuo_servo_ws/src/kinova-ros/ArmTracking/arm_tracking/data/z_trajectory.npy',z_trajectory)
    #                
            #trajectory along x
            start_position_y = [0.211836367846,-0.264596700668, 0.154527920485]
            final_position_y_traj = [0.211836367846, -0.613495767117, 0.154527920485]
            y_trajectory = np.linspace(start_position_y,final_position_y_traj,num_of_points_traj)
            
            camera_data_y = self.move_and_find_camera_line(y_trajectory,end_effector_orientation)    
            np.save('/home/kartik/arm_tracking/visuo_servoing/visuo_servo_ws/src/kinova-ros/ArmTracking/arm_tracking/data/camera_data_y.npy',camera_data_y)
            np.save('/home/kartik/arm_tracking/visuo_servoing/visuo_servo_ws/src/kinova-ros/ArmTracking/arm_tracking/data/y_trajectory.npy',y_trajectory)

        
        camera_axis_along_y = self.get_best_fit_3d_line(camera_data_y,np.sign(y_trajectory[-1][1] - y_trajectory[0][1]))
        camera_axis_along_z = self.get_best_fit_3d_line(camera_data_z,np.sign(z_trajectory[-1][2] - z_trajectory[0][2]))
            
        camera_axis_along_x = np.cross(camera_axis_along_y,camera_axis_along_z.T)
        R_c_rb = np.array((camera_axis_along_x.T,camera_axis_along_y.T,camera_axis_along_z.T)).T
        return R_c_rb
#         R_cr = np.array((camera_axis_along_x),camera_axis_along_y,camera_axis_along_z).T
        
        
#        return R_cr
    def get_tracked_pose(self,tracked_pose):
        self.tracked_pose = tracked_pose
    
    def move_and_find_camera_line(self,target_trajectory,end_effector_orientation) :
        camera_traj_tvec = []

        target_pose= list(target_trajectory[0]) + end_effector_orientation
        print(target_pose) 
        plan = self.robot.get_plan_move_to_goal(target_pose)        
        self.robot.execute_trajectory(plan,wait=True)
        time.sleep(1)
        plan = self.robot.get_plan_move_to_goal(target_pose)        
        self.robot.execute_trajectory(plan,wait=True)
        time.sleep(1)
        tvec = self.tracked_pose.robot_ef_tvec
        camera_traj_tvec.append(tvec)
        rospy.loginfo(tvec)
        
        for target in target_trajectory[1:]:

            waypoints = [self.robot.get_end_effector_pose()]
            target_pose= list(target) + end_effector_orientation
        
            waypoints.append(target_pose)
            print(target_pose)
            plan = self.robot.get_plan_move_along_line(waypoints)
            self.robot.execute_trajectory(plan,wait=True)
            time.sleep(1)
            plan = self.robot.get_plan_move_to_goal(target_pose)        
            self.robot.execute_trajectory(plan,wait=True)
            time.sleep(1)
            tvec = self.tracked_pose.robot_ef_tvec
            camera_traj_tvec.append(tvec)
            rospy.loginfo(tvec)
            
#        return camera_traj_tvec
        
        return camera_traj_tvec
        
        
    def get_best_fit_3d_line(self,data,sign):
#        x = np.mgrid[-2:5:120j]
#        y = np.mgrid[1:9:120j]
#        z = np.mgrid[-5:3:120j]
#        
#        data = np.concatenate((x[:, np.newaxis], 
#                               y[:, np.newaxis], 
#                               z[:, np.newaxis]), 
#                              axis=1)
#        
        # Perturb with some Gaussian noise
#        data += np.random.normal(size=data.shape) * 0.4
        
        # Calculate the mean of the points, i.e. the 'center' of the cloud
        num_points = data.shape[0]
        data = np.array(data)
        datamean = data.mean(axis=0)
        
        # Do an SVD on the mean-centered data.
        uu, dd, vv = np.linalg.svd(data - datamean)
        
        # Now vv[0] contains the first principal component, i.e. the direction
        # vector of the 'best fit' line in the least squares sense.
        
        # Now generate some points along this best fit line, for plotting.
        
        # I use -7, 7 since the spread of the data is roughly 14
        # and we want it to have mean 0 (like the points we did
        # the svd on). Also, it's a straight line, so we only need 2 points.
        linepts = vv[0] * np.mgrid[0.1:-0.1:2j][:, np.newaxis]
        
        # shift by the mean to get the line in the right place
        linepts += datamean
        direction = (data[int(num_points/2):].mean(axis=0) - data[:int(num_points/2)].mean(axis=0))
        sign *= np.sign(np.dot(vv[0],direction))
        return vv[0]*sign
        # Verify that everything looks right.
        
#        import matplotlib.pyplot as plt
#        import mpl_toolkits.mplot3d as m3d
#        
#        ax = m3d.Axes3D(plt.figure())
#        ax.scatter3D(*data.T)
#        ax.plot3D(*linepts.T)
#        plt.show()
#        
    
if __name__ == '__main__':
    robot = Robot('kinova','virtual')
    robot.set_planner_type('RRT')
    env = Environment(robot)    
    env.add_all_objects()
    cal_method = CalibrateTransforms(robot)
    cal_method.perform_calibration()
#    pose_track = PoseTracker(robot,env, image_topic='/camera/color/image_raw')
#    
#    time.sleep(2)
#    
#    pose_track.calibrate_transforms()    
