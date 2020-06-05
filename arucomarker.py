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


#####global varibales
# global centre



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

   
    #Load the dictionary that was used to generate the markers.
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)

    # Initialize the detector parameters using default values
    parameters =  cv2.aruco.DetectorParameters_create()


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

            colorizer = rs.colorizer()
            colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

            # Stack both images horizontally
            images = None
            if enable_rgb:
                images = np.hstack((color_image, colorized_depth)) if enable_depth else color_image
            elif enable_depth:
                images = colorized_depth

            # Detect the markers in the image
            markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(color_image, dictionary, parameters=parameters)
            
            aruco_list = {}
            # centre= {}
            result_center={}
            # orient_centre= {}
            if markerIds is not None:
                # Print corners and ids to the console
                # result=zip(markerIds, markerCorners)
                for k in range(len(markerCorners)):
                    temp_1 = markerCorners[k]
                    temp_1 = temp_1[0]
                    temp_2 = markerIds[k]
                    temp_2 = temp_2[0]
                    aruco_list[temp_2] = temp_1
                key_list = aruco_list.keys()
                font = cv2.FONT_HERSHEY_SIMPLEX
                for key in key_list:
                    # dict_entry = aruco_list[key]    
                    # centre[key] = dict_entry[0] + dict_entry[1] + dict_entry[2] + dict_entry[3]
                    # centre[key][:] = [int(x / 4) for x in centre]
                    # orient_centre[key] = centre[key] + [0.0,5.0]
                    # centre[key] = tuple(centre[key])  
                    # orient_centre[key] = tuple((dict_entry[0]+dict_entry[1])/2)
                    # cv2.circle(images,centre[key],1,(0,0,255),8)
                    dict_entry = aruco_list[key]    
                    centre = dict_entry[0] + dict_entry[1] + dict_entry[2] + dict_entry[3]
                    centre[:] = [int(x / 4) for x in centre]
                    # orient_centre = centre + [0.0,5.0]
                    centre = tuple(centre)
                    result_center[key]= centre
                    # orient_centre = tuple((dict_entry[0]+dict_entry[1])/2)
                    cv2.circle(images,centre,1,(0,0,255),8)
                
                #compute distance
                if len(result_center)<2:
                    print("No enough marker detected")
                else:
                    dis2cam0 = rs.depth_frame.get_distance(depth_frame,result_center[0][0],result_center[0][1])
                    dis2cam5 = rs.depth_frame.get_distance(depth_frame,result_center[5][0],result_center[5][1])
                    obj0_pos = [result_center[0][0],result_center[0][1],dis2cam0]
                    target5_pos = [result_center[5][0],result_center[5][1],dis2cam5]
                    dis_obj2target = np.sqrt(pow(obj0_pos[0]-target5_pos[0],2)+pow(obj0_pos[1]-target5_pos[1],2)+pow(obj0_pos[2]-target5_pos[2],2))
                    print(dis_obj2target)
                    

                # Outline all of the markers detected in our image
                images = cv2.aruco.drawDetectedMarkers(images, markerCorners, borderColor=(0, 0, 255))
                # print(result_center[0])

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




