#Libraries the file requires
#Python does not import twice unless specifically told.
import pyrealsense2 as rs
import cv2
import numpy as np
import time
import sys
import math
import random
#Function file for quality control
from CV_Functions import (
                        Product_line_Resolution,
                        Hardware_reset)



""" FUNCTIONS FOR RENSEC
---------------------------------------------------------------------------
"""
def find_line_model(points):
    m = (points[1,1] - points[0,1]) / (points[1,0] - points[0,0] + sys.float_info.epsilon)
    c = points[1,1] - m * points[1,0]                                    
    return m, c


def find_intercept_point(m, c, x0, y0):
    x = (x0 + m*y0 - m*c)/(1 + m**2)
    y = (m*x0 + (m**2)*y0 - (m**2)*c)/(1 + m**2) + c
    return x, y


""" MAIN
---------------------------------------------------------------------------
"""


FPS = 30

if __name__ == "__main__":
    
    #Hardware rest if desired, uncomment the line below    
    #Hardware_reset() 
    
    pipeline = rs.pipeline()
    config = rs.config()
    
    #Gets product information.
    Product_line_Resolution(pipeline,config)
    
    #Configure the camera settings
    #Connect camera and take a look at intel realsense-viewer for configuration
    #https://github.com/IntelRealSense/librealsense
    
    #Depth settings
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, FPS)
    #RGB settings
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, FPS)
    
    #Start streaming
    configuration = rs.config()
    pipeline.start(config)
    
    
    # Ransac parameters
    ransac_iterations = 450  # number of iterations
    ransac_threshold = 2    # threshold 
    ransac_ratio = 0.6      # ratio of inliers required to assert that a model fits well to data
    Samples = 250
    
    rng = np.random.default_rng()
    Points = np.zeros((2,2))
    
    XYLIST = []
    TEST_samples = []
    model_m = 0.
    model_c = 0.
    
    while True:
              
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        
        #Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        #Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        
        start = time.time()
        #-------------------------------------
        #Edge pixels
        edges = cv2.Canny(color_image,275,300)
        
        
        
        XYLIST.clear()
        
        for i in range(edges.shape[0]):
            for j in range(edges.shape[1]):
                if(edges[i,j] == 255):
                    XYLIST.append([i,j])
        
        TEST_samples = random.sample(XYLIST,Samples)
        #-------------------------------------
        #RANSAC 
        ratio = 0.
        for it in range(ransac_iterations):
            rand = rng.integers(0, len(XYLIST), size = 2)
            
            P1 = XYLIST[rand[0]]
            P2 = XYLIST[rand[1]]
            
            Points[0,0] = P1[0]
            Points[0,1] = P1[1]
            Points[1,0] = P2[0]
            Points[1,1] = P2[1]
            m, c = find_line_model(Points)
            
            num = 0
            
            
            for ind in range(len(TEST_samples)):

                x0 = TEST_samples[ind][0]
                y0 = TEST_samples[ind][1]

                x1, y1 = find_intercept_point(m, c, x0, y0)
    
                dist = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
                
                if dist < ransac_threshold:
                    num += 1
        
            if num/float(Samples) > ratio:
                ratio = num/float(Samples)
                model_m = m
                model_c = c
                Stored_P1 = P1
                Stored_P2 = P2
                
            if num > Samples*ransac_ratio:
                print("DONE")
                break
    
        
        #-------------------------------------
        #Show RGB image
        cv2.namedWindow('RGB stream', cv2.WINDOW_AUTOSIZE)
        cv2.line(color_image, Stored_P1, Stored_P2, color = (255,0,0), thickness = 3)
        cv2.imshow('RGB stream', color_image)
        #-------------------------------------
        #Show edge image
        
        cv2.namedWindow('edges stream', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('edges stream', edges)
        end = time.time()
        print(end-start)
        
        k = cv2.waitKey(1)
                
        #Escape stream
        if k%256 == 27:
            # ESC pressed
            print("Escape hit, closing...")
            break
                
        #Special image capturing
        elif k%256 == 32:
            # SPACE pressed
            Space_capture(Space_img_counter, color_image, depth_image)
            Space_img_counter += 1
  
    # Stop streaming
    pipeline.stop()