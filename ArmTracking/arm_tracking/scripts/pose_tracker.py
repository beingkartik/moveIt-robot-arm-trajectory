#!/usr/bin/env python
"""
Created on Sat May 30 22:31:22 2020

@author: kartik
"""
from __future__ import print_function
import numpy as np
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import matplotlib.pyplot as plt
#import pyquaternion as pyq
import sys
from cv_bridge import CvBridge,CvBridgeError
import rospy
from sensor_msgs.msg import Image
from arm_tracking_planner_executer import Robot
from environment import Environment
from calibration_transforms import Transforms
if  '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path : sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') 
import cv2
from cv2 import aruco
from arm_tracking.msg import PoseTracking

class PoseTracker():
    def __init__(self, robot,env):
        self.robot = robot
        self.env = env
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.get_image)
        self.image = None
        
        #put camera matrices here
        self.camera_instrinsic = np.array(((617.0026849655,-0.153855356,315.5900337131),#fx, s,cx
                   (0,614.4461785395,243.0005874753), ##0,fy,cy
                   (0,0,1) ))
        self.dist = np.array((0.1611730644,-0.3392379107,0.0010744837,0.000905697)) #k1,k2,p1,p2 ie radial dist and tangential dist
        
        self.marker_side = 0.039
        
        #aruco marker tracks all the markers in the frame and returns poses as list, specify which pose belongs to what 
        self.robot_base_id,self.robot_ef_id, self.workpiece_id = 0,1,2
        self.pub = rospy.Publisher('arm_tracking/tracked_image',Image, queue_size=10)
        self.pose_tracking_pub = rospy.Publisher('arm_tracking/pose_tracking',PoseTracking,queue_size=10)
#        self.numpy_sub = rospy.Subscriber('numpy_float',PoseTracking,self.numpy_sub_fun)
        
    def numpy_sub_fun(self,data):
        print(data)
            
    def get_image(self,Image):
        try:
            self.image = self.bridge.imgmsg_to_cv2(Image,desired_encoding = 'bgr8')
        except CvBridgeError as e:
            print (e)

#    takes cv image and publishes as Image
    def publish_image(self,cvImage):
        try:
            Image = self.bridge.cv2_to_imgmsg(cvImage,encoding = 'bgr8')
            self.pub.publish(Image)
            
        except CvBridgeError as e:
            print (e)  
            
#   given the 3 aruco markers, this gets which marker belongs to which object
    def assign_markers(self,centers):
        x_indices = [pt[0] for pt in centers]
        y_indices = [pt[1] for pt in centers]
        self.robot_base_id, self.workpiece_id = np.argmax(x_indices),np.argmax(y_indices)
        self.robot_ef_id = int(3 - self.workpiece_id - self.robot_base_id)        
        
    def estimate_pose(self,frame, marker_side, camera_instrinsic,dist):
        try:
            pose_tracking_msg = PoseTracking()
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
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[id_[0]], marker_side, camera_instrinsic, dist)
                    output.append([rvec,tvec,corners[id_[0]]])
            
            frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
            centers = []
    
            for i in range(len(ids)):
                c = corners[i][0]
                center = (int(c[:, 0].mean()), int(c[:, 1].mean()))
                centers.append(center)
                
            self.assign_markers(centers)
            
            texts = ["base","ef","work"]
            for i,j in enumerate([self.robot_base_id,self.robot_ef_id, self.workpiece_id]):
                
                text = texts[i]
                cv2.putText(frame_markers,text,centers[j],cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),4,bottomLeftOrigin=False)
            
            pose_tracking_msg.robot_base_rvec = output[self.robot_base_id][0].squeeze()
            pose_tracking_msg.robot_base_tvec = output[self.robot_base_id][1].squeeze()
            pose_tracking_msg.robot_ef_rvec   = output[self.robot_ef_id][0].squeeze()
            pose_tracking_msg.robot_ef_tvec   = output[self.robot_ef_id][1].squeeze()
            pose_tracking_msg.workpiece_rvec   = output[self.workpiece_id][0].squeeze()
            pose_tracking_msg.workpiece_tvec   = output[self.workpiece_id][1].squeeze()
            pose_tracking_msg.workpiece_corners_x = (output[self.workpiece_id][2].squeeze())[:,0].tolist()
            pose_tracking_msg.workpiece_corners_y = (output[self.workpiece_id][2].squeeze())[:,1].tolist()
            self.pose_tracking_pub.publish(pose_tracking_msg)
            self.publish_image(frame_markers)
            text = "Successfully Tracking " + str(len(output))+" markers"
            rospy.loginfo(text)
#            print(output)
    #           
    #        
    #        for i in range(len(ids)):
    #            c = corners[i][0]
    #            center = (int(c[:, 0].mean()), int(c[:, 1].mean()))
    #            cv2.putText(frame_markers,str(i),center,cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),10,bottomLeftOrigin=False)
    #            plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "o", label = "id={0}".format(ids[i]))
    #            centers.append(center)
                
            
#            self.assign_markers(centers)
    
            return output
        except:
            rospy.logerr('Not able to track')
    
    def get_robot_and_workpiece_pose(self):
        try:
            poses = self.estimate_pose(self.image,self.marker_side,self.camera_instrinsic,self.dist)
            if len(poses) != 3:
                rospy.logerr('Only '+ str(len(poses))+' markers visible in the image')
            else:
                rospy.loginfo("SUCCEESSSS")
        except:
            poses = []
            rospy.logerr('Not able to track')
    
    
    def calibrate_transforms(self):
        poses = self.estimate_pose(self.image,self.marker_side,self.camera_instrinsic,self.dist)
        rvec_rb = poses[self.robot_base_id][0]
        R_rb,_ = cv2.Rodrigues(rvec_rb)
        self.transform = Transforms(R_rb)
        rospy.loginfo(self.transform.R_cb)
            
    
    #get robot ef position (x,y,z) in robot frame using camera tracking
    def get_robot_ef_position(self):
        print(type(self.image))
        self.calibrate_transforms()
        poses = self.estimate_pose(self.image,self.marker_side,self.camera_instrinsic,self.dist)
        p_cr = poses[self.robot_ef_id][1] #get tvec from aruco marker
        return self.transform.get_pos_rframe(p_cr[0])

#        try:
#            print(type(self.image))
#            self.calibrate_transforms()
#            poses = self.estimate_pose(self.image,self.marker_side,self.camera_instrinsic,self.dist)
#            p_cr = poses[self.robot_ef_id][1] #get tvec from aruco marker
#            return self.transform.get_pos_rframe(p_cr[0])
#        except:
#            rospy.logerr('Not able to track')
            
        
    #gets robot ef pose(6D) in robot frame using moveit
    def get_robot_pose(self):

        robot_pose = self.robot.group.get_current_pose().pose.position
        robot_pose = np.array((robot_pose.x,robot_pose.y,robot_pose.z))
        return robot_pose
        
    def get_workpiece_marker_position(self):
        poses = self.estimate_pose(self.image,self.marker_side,self.camera_instrinsic,self.dist)
        p_cr = poses[self.workpiece_id][1] #get tvec from aruco marker
        return self.transform.get_pos_rframe(p_cr)
        
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
        if posetracker.image is not None: 
            posetracker.calibrate_transforms
            posetracker.get_robot_and_workpiece_pose()
#    while(1):
#        if posetracker.image is not None:
##            image = posetracker.image.copy()
#            cv2.namedWindow( "Display window") 
#            cv2.imshow( "Display window", posetracker.image)
#            cv2.waitKey(3)
    rospy.spin()    
    
        
        