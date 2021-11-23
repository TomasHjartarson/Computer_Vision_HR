
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


cap = cv2.VideoCapture("http://10.0.0.5:8080/video")

radius = 41
while(True):
    ret, frame = cap.read()
    
    #Bright
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #gray = cv2.GaussianBlur(gray, (radius, radius), 0)
    #(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
    #cv2.circle(frame, maxLoc, radius, (255, 0, 0), 2)
      
      
    
    #Reddest  
    HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    result = frame.copy()
    lower = np.array([170,50,50])   
    upper = np.array([180,255,255])
    mask = cv2.inRange(HSV, lower, upper)
    result = cv2.bitwise_and(HSV, HSV, mask=mask)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result[:,:,1])
    cv2.circle(frame, maxLoc, radius, (255, 0, 0), 2) 
    
    
    
    #Show image  
    cv2.imshow('frame',frame)
    
    k = cv2.waitKey(1)
            
    #Escape stream
    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break
        