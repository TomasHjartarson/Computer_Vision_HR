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


#Intersection calculation
def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       return -1,-1

    d = (det(*line1), det(*line2))
    x = (int) (det(d, xdiff) / div)
    y = (int) (det(d, ydiff) / div)
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
    
    
    
    intersection_points = []
    Line_points = []
    line_amount = 4
    

    while True:
              
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        
        
        #Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        start = time.time()
        #-------------------------------------
        #Edge pixels
        edges = cv2.Canny(color_image,100,200,apertureSize = 3)
        lines = cv2.HoughLines(edges,1,np.pi/180,50)
        
        Line_points.clear()
        intersection_points.clear()
        for i in range(line_amount):
            for rho,theta in lines[i]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 1000*(-b))
                    y1 = int(y0 + 1000*(a))
                    x2 = int(x0 - 1000*(-b))
                    y2 = int(y0 - 1000*(a))
                    
                    Stored_P1 = [x1,y1]
                    Stored_P2 = [x2,y2]
                    cv2.line(color_image,(x1,y1),(x2,y2),(0,0,255),2)
                    
                            
                    if(i == 0):
                        Line_points.insert(0, [Stored_P1,Stored_P2])
                    elif(i == 1):
                        Line_points.insert(1, [Stored_P1,Stored_P2])
                    elif(i == 2):
                        Line_points.insert(2, [Stored_P1,Stored_P2])
                    else:
                        Line_points.insert(3, [Stored_P1,Stored_P2])      

        intersection_points.append(line_intersection(Line_points[0],Line_points[1]))
        intersection_points.append(line_intersection(Line_points[0],Line_points[2]))
        intersection_points.append(line_intersection(Line_points[0],Line_points[3]))
        intersection_points.append(line_intersection(Line_points[1],Line_points[2]))
        intersection_points.append(line_intersection(Line_points[1],Line_points[3]))
        intersection_points.append(line_intersection(Line_points[2],Line_points[3]))
            
        print(intersection_points)
        #-------------------------------------
        #Show RGB image
        cv2.namedWindow('RGB stream', cv2.WINDOW_AUTOSIZE)
        for j in range(6):
            if(intersection_points[j] != [-1,-1]):
                color_image = cv2.circle(color_image, intersection_points[j], 10, [255,0,255], thickness = -1)
        cv2.imshow('RGB stream', color_image)
        
        
        
        #-------------------------------------
        #Show edge image
        
        cv2.namedWindow('edges stream', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('edges stream', edges)
        end = time.time()
        #print(end-start)
        
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