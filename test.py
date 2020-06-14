import pyrealsense2 as rs
import numpy as np
import cv2 
import time
import math
from camera import Camera

start=time.time()

enable_rgb = True
enable_depth = True
width = 640
height = 480

camera = Camera(width=width, height=height, enable_rgb=enable_rgb, enable_depth=enable_depth)
end=time.time()
print("running time",str(end-start))
cv2.namedWindow('Update', cv2.WINDOW_AUTOSIZE)
# color_image, depth_image=camera._run()
color_image = camera.markerprocess()
# result=camera.get_marker_position()
# dis= camera.get_distance()
# print(result)
# print(dis)

# print("running time",str(end-start))
while True:
    if color_image is not None:
        cv2.imshow('Update',color_image)
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q') or key == 27:
        cv2.destroyAllWindows()
        break


