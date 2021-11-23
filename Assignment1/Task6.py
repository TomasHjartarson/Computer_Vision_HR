
import pyrealsense2 as rs
import cv2
import numpy as np
import time
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
radius = 41
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
    maxLoc = [0,0]    
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
        ############### BRIGHTNESS
        """
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (radius, radius), 0)
        max_val = 0
        
        for i in range(gray.shape[0]):
            for j in range(gray.shape[1]):
                if(gray[i,j] > max_val):
                    max_val = gray[i,j]
                    maxLoc = [i,j]
                    
        cv2.circle(color_image, maxLoc, radius, (255, 0, 0), 2)
        """
        ############### REDDEST
        HSV = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        result = color_image.copy()
        lower = np.array([170,50,50])   
        upper = np.array([180,255,255])
        mask = cv2.inRange(HSV, lower, upper)
        result = cv2.bitwise_and(HSV, HSV, mask=mask)
        
        max_val = 0
        for i in range(result.shape[0]):
            for j in range(result.shape[1]):
                if(result[i,j,1] > max_val):
                    max_val = result[i,j,1]
                    maxLoc = [i,j]
                    
        cv2.circle(color_image, maxLoc, radius, (255, 0, 0), 2)
        
        #-------------------------------------
        #Show RGB image
        cv2.namedWindow('RGB stream', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RGB stream', color_image)
        end = time.time()
        print("Brightest spot processing [s]: ")
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