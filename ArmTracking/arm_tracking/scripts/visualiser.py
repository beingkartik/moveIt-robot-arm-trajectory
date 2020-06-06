#!/usr/bin/env python
"""
Created on Fri May 22 13:27:22 2020

@author: kartik
"""
from __future__ import print_function
from collections import deque
import matplotlib.pyplot as plt
from arm_tracking_planner_executer import Robot
import numpy as np
import rospy
from sensor_msgs.msg import Image
from arm_tracking.msg import PidUpdate,PidParam
from cv_bridge import CvBridge,CvBridgeError

class Visualiser():
    def __init__(self):
        
        self.image_sub = rospy.Subscriber('/arm_tracking/tracked_image',Image,self.get_image)
        self.pid_update_log_sub = rospy.Subscriber('/arm_tracking/pid_update_log',PidUpdate,self.update_pid_log)
        self.update_log = deque()
        self.update_log_len_reqd = 100
        self.error_log = deque()        
        self.bridge = CvBridge()

        
    def get_image(self,Image):
        try:
            self.image = self.bridge.imgmsg_to_cv2(Image,desired_encoding = 'bgr8')
        except CvBridgeError as e:
            print (e)
            
    def update_pid_log(self,PidUpdate):
        self.update_log.append(PidUpdate)
        self.error_log.append(PidUpdate.error)
#        print(len(self.error_log))
        if len(self.update_log) > self.update_log_len_reqd :
            self.update_log.popleft()
            self.error_log.popleft()
    
    def show_graph(self,fig):
        
        error_log = np.array(self.error_log)
        plt.clf()
        
        fig.add_subplot(2,2,1)
        plt.plot(np.arange(0,len(error_log)),error_log[:,0])
        plt.title("error along x axis")
        fig.add_subplot(2,2,2)
        plt.plot(np.arange(0,len(error_log)),error_log[:,1])
        plt.title("error along y axis")
        fig.add_subplot(2,2,3)
        plt.plot(np.arange(0,len(error_log)),error_log[:,2])
        plt.title("error along z axis")
        
        if self.image is not None:
            fig.add_subplot(2,2,4)
            plt.imshow(self.image)
        
        plt.draw()
        plt.pause(1)
        

#        fig.show()

if __name__ == '__main__':
    rospy.init_node('visualiser',anonymous=True)

    vis = Visualiser()
    fig = plt.figure()
    
    while(1):
        
        if len(vis.error_log)>2:        vis.show_graph(fig)
    
    rospy.spin()