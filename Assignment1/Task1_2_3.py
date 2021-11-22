
import pyrealsense2 as rs
import cv2
import numpy as np

#Function file
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
        
        #Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        #Alpha controls the colour sensitivity, please do not use JET as colormap        
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.3),
                                           cv2.COLORMAP_VIRIDIS)
        
        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape
        
        
        #Show Depth image
        cv2.namedWindow('Depth stream', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Depth stream', depth_colormap)
        
        #Show RGB image
        cv2.namedWindow('RGB stream', cv2.WINDOW_AUTOSIZE)
        cv2.putText(color_image,('FPS : ' + str(FPS)), (10,450), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow('RGB stream', color_image)
        
        #Show Infrared 1 image (LEFT)
        cv2.namedWindow('Infrared 1 (LEFT)', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Infrared 1 (LEFT)', INFR1)
        
        #Show Infrared 2 image (RIGHT)
        cv2.namedWindow('Infrared 2 (RIGHT)', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Infrared 2 (RIGHT)', INFR2)
        
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
