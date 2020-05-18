#!/usr/bin/env python
"""
Created on Sun May 17 14:03:33 2020

@author: kartik
"""
import numpy as np
#import pyquaternion as pyq
import cv2
from cv_bridge import CvBridge,CvBridgeError
import rospy
from sensor_msgs.msg import Image
from arm_tracking_planner_executer import Robot
from environment import Environment
from cv2 import aruco

class PoseTracker():
    def __init__(self, robot,env):
        self.robot = robot
        self.env = env
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.get_image)
        self.image = None
        self.camera_instrinsic = np.array(((617.0026849655,-0.153855356,315.5900337131),#fx, s,cx
                   (0,614.4461785395,243.0005874753), ##0,fy,cy
                   (0,0,1) ))
        self.dist = np.array((0.1611730644,-0.3392379107,0.0010744837,	0.000905697)) #k1,k2,p1,p2 ie radial dist and tangential dist
        self.marker_side = 0.039

    def get_image(self,Image):
        try:
            self.image = self.bridge.imgmsg_to_cv2(Image,desired_encoding = 'bgr8')
            
        except CvBridgeError as e:
            print e

    def estimate_pose(self,frame, marker_side, camera_instrinsic,dist):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        # detector parameters can be set here (List of detection parameters[3])
        parameters =  aruco.DetectorParameters_create()
        #parameters.adaptiveThreshConstant = 10
        
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        output = []
        for id_ in ids:
            if id_ is not None:
            # estimate pose of each marker and return the values
            # rvet and tvec-different from camera coefficients
                rvec, tvec = aruco.estimatePoseSingleMarkers(corners[id_[0]], marker_side, camera_instrinsic, dist)
                output.append([rvec,tvec,corners])
                
        return output
    
    def get_robot_and_workpiece_pose(self):
        try:
            poses = self.estimate_pose(self.image,self.marker_side,self.camera_instrinsic,self.dist)
            if len(poses) != 2:
                rospy.logerr('Only '+ str(len(poses))+' markers visible in the image')
        except:
            poses = []
            rospy.logerr('Not able to track')
            
    
    
    def get_robot_pose(self):
        robot_pose = self.robot.group.get_current_pose().pose.position
        robot_pose = np.array((robot_pose.x,robot_pose.y,robot_pose.z))
        return robot_pose
        
    
    def get_workpiece_edge(self):
        shape = self.env.workpiece_size
        target_end_points = np.linspace((0,shape[1],shape[2],1),(shape[0],shape[1],shape[2],1),10)
        
        transformation_matrix = np.identity(4)
        transformation_matrix [:3,3] = self.env.workpiece_pose[:3] #dx,dy,dz
        
        workpiece_edge_points = (np.dot(transformation_matrix,target_end_points.T)).T[:,:3]
        return workpiece_edge_points
    
if __name__ == '__main__':
    robot = Robot('kinova')
    env = Environment()    
    posetracker = PoseTracker(robot,env)
    
    while(1):
        posetracker.get_robot_and_workpiece_pose()
#    while(1):
#        if posetracker.image is not None:
##            image = posetracker.image.copy()
#            cv2.namedWindow( "Display window") 
#            cv2.imshow( "Display window", posetracker.image)
#            cv2.waitKey(3)
    rospy.spin()    
    
        
        