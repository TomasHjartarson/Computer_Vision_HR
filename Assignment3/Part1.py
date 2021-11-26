import pyrealsense2 as rs
import numpy as np
import cv2
import time

from CV_Functions import (
                        Product_line_Resolution,
                        Hardware_reset)


classFile = 'coco.names'
classNames = []

with open(classFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')
    
FPS = 30
width = 640
height = 480
confThreshold = 0.5
nmsThreshold = 0.3

def findObjects(outputs,img):
    hT,wT,cT = img.shape
    bbox = []
    classIds = []
    confs = []
    
    for output in outputs: 
        for det in output:
            scores = det[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > confThreshold:
                w , h = int(det[2]*wT) , int(det[3]*hT)
                x , y = int((det[0]*wT)-w/2) , int((det[1]*hT) - h/2)
                bbox.append([x,y,w,h])
                classIds.append(classId)
                confs.append(float(confidence))
    indices = cv2.dnn.NMSBoxes(bbox,confs,confThreshold,nmsThreshold)
    
    for i in indices:      
        box = bbox[i]
        x,y,w,h = box[0],box[1],box[2],box[3]
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,255),2)
        cv2.putText(img,f'{classNames[classIds[i]].upper()} {int(confs[i]*100)}%',
                    (x,y-10),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
        
        
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
    
    modelConfiguration = 'yolov3-416.cfg'
    modelWeights = 'yolov3-416.weights'
    
    net = cv2.dnn.readNetFromDarknet(modelConfiguration,modelWeights)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

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
        blob = cv2.dnn.blobFromImage(color_image,1/255,(width,height),[0,0,0],1,crop = False)
        net.setInput(blob)
        
        layerNames = net.getLayerNames()
        outputNames = [layerNames[i-1] for i in net.getUnconnectedOutLayers()]
        outputs = net.forward(outputNames)

        
        findObjects(outputs, color_image)
        end = time.time()
        #-------------------------------------
        #Show RGB image
        frames_per_second = (end-start)
        cv2.namedWindow('RGB stream', cv2.WINDOW_AUTOSIZE)
        cv2.putText(color_image,('FPS : ' + str(frames_per_second)), (10,450), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imshow('RGB stream', color_image)
        
        k = cv2.waitKey(1)
        
        #Escape stream
        if k%256 == 27:
            # ESC pressed
            print("Escape hit, closing...")
            break

    # Stop streaming
    pipeline.stop()