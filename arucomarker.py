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
import math



def pixel2point(frame,u):
    u_x = u[0]
    u_y = u[1]
    # Get depth from pixels
    dis2cam_u = frame.get_distance(u_x,u_y)
    # Convert pixels to 3D coordinates in camera frame(deprojection)
    depth_intrin = frame.profile.as_video_stream_profile().intrinsics
    u_pos = rs.rs2_deproject_pixel_to_point(depth_intrin, [u_x, u_y], dis2cam_u)

    return u_pos

# Distance computation through pixels 
def distance_pixel(frame,u,v):
    
    # Copy pixels into the arrays (to match rsutil signatures)
    u_x = u[0]
    u_y = u[1]
    v_x = v[0]
    v_y = v[1]
    # Get depth from pixels
    dis2cam_u = frame.get_distance(u_x,u_y)
    dis2cam_v = frame.get_distance(v_x,v_y)
    # Convert pixels to 3D coordinates in camera frame(deprojection)
    depth_intrin = frame.profile.as_video_stream_profile().intrinsics
    u_pos = rs.rs2_deproject_pixel_to_point(depth_intrin, [u_x, u_y], dis2cam_u)
    v_pos = rs.rs2_deproject_pixel_to_point(depth_intrin, [v_x, v_y], dis2cam_v)
    
    # Calculate distance between two points
    dis_obj2target = np.sqrt(pow(u_pos[0]-v_pos[0],2)+pow(u_pos[1]-v_pos[1],2)+pow(u_pos[2]-v_pos[2],2))
    
    return dis_obj2target

# Distance computation through 3d points
def distance_3dpoints(u,v):
    
    dis_obj2target = np.sqrt(pow(u[0]-v[0],2)+pow(u[1]-v[1],2)+pow(u[2]-v[2],2))

    return dis_obj2target


if __name__ == "__main__":

    # Default settings such as serial number
    device_id = "829212070352"  # "923322071108" # serial number of device to use or None to use default
    enable_rgb = True
    enable_depth = True
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

        # Aruco marker part
        # Load the dictionary that was used to generate the markers.
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


                # Aruco marker part
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
                    # print(key_list)
                    for key in key_list:
                        dict_entry = aruco_list[key]    
                        centre = dict_entry[0] + dict_entry[1] + dict_entry[2] + dict_entry[3]
                        centre[:] = [int(x / 4) for x in centre]
                        # orient_centre = centre + [0.0,5.0]
                        centre = tuple(centre)
                        result_center[key]= centre
                        # orient_centre = tuple((dict_entry[0]+dict_entry[1])/2)
                        cv2.circle(images,centre,1,(0,0,255),8)
                    
                    # Compute distance when matching the conditions
                    # print(result_center)
                    print(len(result_center))
                    if len(result_center)<4:
                        print("No enough marker detected")
                    
                    else:
                        # Moving object localization marker
                        x_id0 = result_center[0][0]
                        y_id0 = result_center[0][1]
                        p_0 = [x_id0,y_id0]
                        
                        # Target object localization marker 
                        # Single ID-5
                        x_id5 = result_center[5][0]
                        y_id5 = result_center[5][1]
                        p_5 = [x_id5,y_id5]
                        
                        # Dual ID-4 and ID-3
                        x_id4 = result_center[4][0]
                        y_id4 = result_center[4][1]
                        p_4 = [x_id4,y_id4]

                        x_id3 = result_center[3][0]
                        y_id3 = result_center[3][1]
                        p_3 = [x_id3,y_id3]

                        # Deproject pixel to 3D point
                        point_0 = pixel2point(depth_frame,p_0)           
                        point_5 = pixel2point(depth_frame,p_5)
                        point_4 = pixel2point(depth_frame,p_4)
                        point_3 = pixel2point(depth_frame,p_3)
                        # point_0_new = np.array(point_0)
                        # point_5_new = np.array(point_5)
                        # point_4_new = np.array(point_4)
                        # point_3_new = np.array(point_3)
                        # Calculate target point
                        point_target=[point_4[0]+point_3[0]-point_5[0],point_4[1]+point_3[1]-point_5[1],point_4[2]+point_3[2]-point_5[2]]
                        # Compute distance
                        # dis=distance_3dpoints(point_target,point_0)

                        # Display target and draw a line between them
                        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
                        target_pixel = rs.rs2_project_point_to_pixel(color_intrin, point_target)
                        target_pixel[0] = int(target_pixel[0])
                        target_pixel[1] = int(target_pixel[1])
                        cv2.circle(images,tuple(target_pixel),1,(0,0,255),8)
                        cv2.line(images,tuple(p_0),tuple(target_pixel),(0,255,0),2)
                     
                        # Euclidean distance
                        dis_obj2target = distance_3dpoints(point_0,point_target) 
                        dis_obj2target_goal = dis_obj2target*np.sin(np.arccos(0.02/dis_obj2target))
                        
                        print(dis_obj2target_goal)
                        

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






