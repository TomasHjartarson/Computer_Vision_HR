#Libraries the file requires
#Python does not import twice unless specifically told.
import pyrealsense2 as rs
import cv2
import numpy as np
import time
#Function file for quality control
from CV_Functions import (Call_repeatedly, 
                        Image_capture,
                        Space_capture,
                        Product_line_Resolution,
                        Hardware_reset)

""" MAIN
---------------------------------------------------------------------------
"""
#VARIABLES
#text on images
font = cv2.FONT_HERSHEY_SIMPLEX
FPS = 30
radius = 10
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
    #Infrared 1
    config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, FPS)
    #Infrared 2
    config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, FPS)  
    
    #Start streaming
    configuration = rs.config()
    pipeline.start(config)
    
    
    #Counting images
    img_counter = 1
    Space_img_counter = 1
    
    #Start recurring
    #future_calls = Call_repeatedly(10, Image_capture,img_counter,
    #                                color_image, depth_colormap) 
    
    #stop future calls
    #future_calls()
        
    while True:
              
        #Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        Infrared1_frame = frames.get_infrared_frame(1) #LEFT
        Infrared2_frame = frames.get_infrared_frame(2) #RIGHT
        if not depth_frame or not color_frame:
            continue
        
        #Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        INFR1       = np.asanyarray(Infrared1_frame.get_data())
        INFR2       = np.asanyarray(Infrared2_frame.get_data())
        
        start = time.time()
        edges = cv2.Canny(color_image,100,200)
 
        #-------------------------------------
        #Show RGB image
        cv2.namedWindow('RGB stream', cv2.WINDOW_AUTOSIZE)
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