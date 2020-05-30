#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 14 13:12:50 2020

@author: kartik
"""
import numpy as np
import sys
if  '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path : sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') 
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
#%matplotlib nbagg
import os
import time


path = '/home/kartik/Downloads/Johns_study_data/3v3/1/minus'
image_folder = 'A'

frame = cv2.imread(os.path.join(path,image_folder,'left0000.jpg'))
counter = 0; 
f = []
for (dirpath, dirnames, filenames) in os.walk(os.path.join(path,image_folder)):
    f.extend(filenames)
    f.sort()
    break
    
prev_frame = cv2.imread(os.path.join(path,image_folder,f[0]))
size = prev_frame.shape
blank_image = np.zeros((size[0],size[1]*2,size[2]), np.uint8)

for index,f_ in enumerate(f):
    if index%2 == 0:
        prev_frame = cv2.imread(os.path.join(path,image_folder,f_))
    else:
        new_frame = cv2.imread(os.path.join(path,image_folder,f_))
        blank_image[:,:size[1],:]=prev_frame
        blank_image[:,size[1]:size[1]*2,:]=new_frame
        plt.imsave(os.path.join('/home/kartik/catkin_ws/src/kinova-ros/ArmTracking/arm_tracking/scripts','stitched_image',f_),
                   blank_image,format='jpg')
    
    
plt.imshow(blank_image)
plt.show()
        
        
        