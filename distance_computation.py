## License: Apache 2.0. See LICENSE file in root directory.
## Parts of this code are
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

##################################################
##      configurable realsense viewer           ##
##################################################

import pyrealsense2 as rs
import numpy as np
import cv2
import time

#
# NOTE: it appears that imu, rgb and depth cannot all be running simultaneously.
#       Any two of those 3 are fine, but not all three: causes timeout on wait_for_frames()
#
device_id = "829212070352"  # "923322071108" # serial number of device to use or None to use default
enable_imu = False
enable_rgb = True
enable_depth = True
# TODO: enable_pose
# TODO: enable_ir_stereo

def nothing(x):
    pass


# Configure streams

pipeline = None
if enable_depth or enable_rgb:
    pipeline = rs.pipeline()
    config = rs.config()

    # if we are provided with a specific device, then enable it
    if None != device_id:
        config.enable_device(device_id)

    if enable_depth:
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)  # depth
    if enable_rgb:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)  # rgb

    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    if enable_depth:
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: ", depth_scale)
        if enable_rgb:
            # Create an align object
            # rs.align allows us to perform alignment of depth frames to others frames
            # The "align_to" is the stream type to which we plan to align depth frames.
            align_to = rs.stream.color
            align = rs.align(align_to)
    
    # eat some frames to allow autoexposure to settle
    for i in range(0, 5):
        pipeline.wait_for_frames()

try:
    frame_count = 0
    start_time = time.time()
    frame_time = start_time
    
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    # Starting with 100's to prevent error while masking
    h,s,v = 100,100,100

    # Creating track bar
    cv2.createTrackbar('h', 'RealSense',0,179,nothing) #149
    cv2.createTrackbar('s', 'RealSense',0,255,nothing) #65
    cv2.createTrackbar('v', 'RealSense',0,255,nothing) #59


    while True:
        last_time = frame_time
        frame_time = time.time() - start_time
        frame_count += 1

        #
        # get the frames
        #
        if enable_rgb or enable_depth:
            frames = pipeline.wait_for_frames(5000 if (frame_count > 1) else 10000) # wait 10 seconds for first frame

        if enable_rgb or enable_depth:
            # Align the depth frame to color frame
            aligned_frames = align.process(frames) if enable_depth and enable_rgb else None
            depth_frame = aligned_frames.get_depth_frame() if aligned_frames is not None else frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame() if aligned_frames is not None else frames.get_color_frame()

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data()) if enable_depth else None
            color_image = np.asanyarray(color_frame.get_data()) if enable_rgb else None


            h = cv2.getTrackbarPos('h','RealSense')
            s = cv2.getTrackbarPos('s','RealSense')
            v = cv2.getTrackbarPos('v','RealSense')

            lower_color = np.array([h,s,v])
            upper_color = np.array([180,255,255])
            hsv_img = cv2.cvtColor(color_image,cv2.COLOR_BGR2HSV)
            
            frame_threshed = cv2.inRange(hsv_img, lower_color, upper_color)
            
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            colorizer = rs.colorizer()
            colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

            # Stack both images horizontally
            images = None
            if enable_rgb:
                images = frame_threshed if enable_depth else color_image
            elif enable_depth:
                images = depth_colormap

            # Show images
            
            if images is not None:
                cv2.imshow('RealSense', images)

        # Press esc or 'q' to close the image window
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:
    # Stop streaming
    if pipeline is not None:
        pipeline.stop()


