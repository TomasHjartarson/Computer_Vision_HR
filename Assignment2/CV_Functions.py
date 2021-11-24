#Libraries the functions require.
import pyrealsense2 as rs
import cv2
from threading import Event, Thread

"""
------------------------------------------------
""" 

#Hardware reset of cameras if required
def Hardware_reset():
    ctx = rs.context()
    devices = ctx.query_devices()
    for dev in devices:
        print(dev)
        dev.hardware_reset()

"""
------------------------------------------------
""" #Function for product line and setting resolution of respective camera.

def Product_line_Resolution(pipeline,config):
    # Configure depth and color streams
    
    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))
    
    #Check if the RGB camera is there.
    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("Depth camera with Color sensor is a requirement")
        exit(0)

"""
------------------------------------------------
""" #Function to call repeatedly for image capturing of the stream

def Call_repeatedly(interval, func, *args):
    stopped = Event()
    def loop():
        while not stopped.wait(interval): # the first call is in `interval` secs
            func(*args)
    Thread(target=loop).start()    
    return stopped.set         
    

"""
------------------------------------------------
""" #Function for reccurent saving the frames from the stream.

def Image_capture(img_counter, color_image, depth_colormap):
  
  #color image capture
  color_img = "Color_frame_{}.png".format(img_counter)
  cv2.imwrite(color_img, color_image)
  print("{} written!".format(color_img))
  
  #Depth image capture
  depth_img = "Depth_frame_{}.png".format(img_counter)
  cv2.imwrite(depth_img, depth_colormap)
  print("{} written!".format(depth_img))
  img_counter += 1

"""
------------------------------------------------
""" #Function for special saving of frames

def Space_capture(Space_img_counter, color_image, depth_colormap):
  
  #color image capture
  color_img = "Space_Color_frame_{}.png".format(Space_img_counter)
  cv2.imwrite(color_img, color_image)
  print("{} written!".format(color_img))
 
  #Depth image capture
  depth_img = "Space_Depth_frame_{}.png".format(Space_img_counter)
  cv2.imwrite(depth_img, depth_colormap)
  print("{} written!".format(depth_img))
  
  
    
  
    
"""
------------------------------------------------
""" 
if __name__ == "__main__":
    print("0")