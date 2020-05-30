#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May 29 10:48:57 2020

@author: kartik
Original by Anjali Asar 
"""
from __future__ import print_function

#!/usr/bin/env python
import numpy as np 
from scipy.spatial import distance

class Transforms():
    '''###
    Input params:
    R_cb --> robot base frame in camera frame axis (We get this from aruco marker on base frame)
    R_rb --> root base frame in robot frame axis (This should just be an identity matrix, as this is our global origin)
    ###'''
    def __init__(self, R_cb, R_rb=np.identity(3)): 
        self.R_cb = R_cb
        self.R_r_rbase = R_rb


    '''###
    Calibrates the Rotation matrix tranformation between robot and camera frame.
    R_rc ---> camera frame in robot frame axis
    ###'''
    def calibrate_rc(self):
        self.R_c_rbase = self.R_cb #cv2.Rodrigues(self.R_cb)[0]
        #print (self.R_c_rbase, self.R_r_rbase.shape)
        self.R_rc = np.dot(self.R_r_rbase, np.transpose(self.R_c_rbase))


    '''###
    Input params:
    c_pos --> Position of aruco marker in camera frame 
    Returns pos of input point in robot frame
    (Need this mostly to find the workpiece position in robot frame)
    ###'''
    def get_pos_rframe(self, c_pos):
        #get_rc_transform() #Do we need this here? or call it separately? 
        self.calibrate_rc()
        pos = np.dot(self.R_rc, np.transpose(c_pos))
        return pos 

    '''###
    Input Params:
    pos_rw --> Workpiece position in robot frame
    pos_re --> end effector in robot frame 
    Returns distance between the two
    (Need this to calculate the current distance being 
    maintained between robot end effector and workpiece)
    ###'''
    def get_dist_from_pos(self, pos_rw, pos_re):
        dist = pos_re - pos_rw
        return dist

    '''###
    Given a set of waypoints, uses above methods to calculate 
    desired distance to maintain between workpiece and robot end-
    effector, for each waypoint 
    ###'''
    def compute_desired_diff(self, pos_cw, waypoints):
        desired_diff = []
        pos_rw = self.get_pos_rframe(pos_cw)
        for i in range(0, len(waypoints)):
            dist = self.get_dist_from_pos(pos_rw, waypoints)
            desired_diff.append(dist)
        return desired_diff

    '''###
    Given current position of end-effector, uses above methods to calculate 
    current distance between workpiece and robot end-
    effector. 
    ###'''
    def compute_curr_diff(self, pos_cw, pos_re):
        pos_rw = self.get_pos_rframe(pos_cw)
        curr_dist = self.get_dist_from_pos(pos_rw, pos_re)
        return curr_dist
        


###Testing
if __name__ == '__main__':
    
    '''###
    Gives us a set of waypoints to traverse
    along xz plane
    Not sure if we need this, wrote it for my ease
    ###'''

    def get_init_trajectory(init_pos, fin_pos, N, non_axis): #N = no. of waypoints, non_axis = axis that is not forming the plane of the line
            traj = np.zeros([N, 3])
            dist = distance.euclidean(init_pos,fin_pos)
            d_pos = dist/N
            if non_axis == 'x':
                axes = [0, 1, 2]
            elif non_axis == 'y':
                axes = [1, 0, 2]
            elif non_axis == 'z':
                axes = [2, 0, 1]
            init_pos_2d = np.array([init_pos[axes[1]], init_pos[axes[2]]])
            fin_pos_2d = np.array([fin_pos[axes[1]], fin_pos[axes[2]]])
            try:
                slope = float(fin_pos_2d[1] - init_pos_2d[1])/(fin_pos_2d[0] - init_pos_2d[0])
            except:
                slope = np.inf
            print ("SLOPE:", fin_pos_2d, init_pos_2d, slope)
            theta = np.arctan(slope)
            x_cos = np.cos(theta)
            y_sin = np.sin(theta)
            for i in range(0, N):
                x = init_pos_2d[0] + d_pos*(i+1)*x_cos
                y = init_pos_2d[1] + d_pos*(i+1)*y_sin
    
                traj[i][axes[0]] = init_pos[axes[0]]
                traj[i][axes[1]] = x
                traj[i][axes[2]] = y
            return traj 
    
    
    
    traj = get_init_trajectory([0, -0.3, 0.3], [0, -0.3, 0.13], 10, 'y')
    print (traj)
    cb = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    rb = np.array([[1,0,0],[0,1,0],[0,0,1]])
    r1 = Transforms(cb, rb)
    desired_diff = r1.compute_desired_diff(([1,5,0.8]), traj)
    print ("calibrated rc:", r1.R_rc)
    print ("Desired diff", desired_diff)
