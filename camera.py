import pyrealsense2 as rs
import numpy as np
import cv2 
import time
import math


WIDTH = 424
HEIGHT = 240

class Camera(object):
    
    def __init__(self,width = WIDTH, height = HEIGHT, enable_rgb=True, enable_depth=True, device_id = None):
        
        self.device_id = "829212070352"  # Lab: "828112071102"  Home:"829212070352"
        self.enable_rgb = enable_rgb
        self.enable_depth = enable_depth

        self.width = width
        self.height = height
        self.resize = (width != WIDTH) or (height != HEIGHT)

        self.pipeline = None
        if self.enable_depth or self.enable_rgb:
            self.pipeline = rs.pipeline()
            config = rs.config()

            # if we are provided with a specific device, then enable it
            if None != self.device_id:
                config.enable_device(self.device_id)

            if enable_depth:
                config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)  # depth
            
            if enable_rgb:
                config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)  # rgb

            # Start streaming
            profile = self.pipeline.start(config)

            # Getting the depth sensor's depth scale (see rs-align example for explanation)
            if self.enable_depth:
                depth_sensor = profile.get_device().first_depth_sensor()
                depth_scale = depth_sensor.get_depth_scale()
                print("Depth Scale is: ", depth_scale)
                if self.enable_rgb:
                    # Create an align object
                    # rs.align allows us to perform alignment of depth frames to others frames
                    # The "align_to" is the stream type to which we plan to align depth frames.
                    align_to = rs.stream.color
                    self.align = rs.align(align_to)

            time.sleep(1)   # let camera warm up

        # initialize frame state
        # original color image
        self.color_image = None
        self.color_frame = None
        self.depth_image = None
        self.depth_frame = None
        self.frame_count = 0
        self.start_time = time.time()
        self.frame_time = self.start_time
        self.running = True
        
        #For displaying (including marker detection) 
        self.images = None
        #For detected markers
        self.useful_color_image = None

        # Aruco marker part
        # Load the dictionary that was used to generate the markers.
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)

        # Initialize the detector parameters using default values
        self.parameters =  cv2.aruco.DetectorParameters_create()
    
    # Capture current frame  
    def _capture(self):      
        
        # get the frames
        try:
            if self.enable_rgb or self.enable_depth:
                frames = self.pipeline.wait_for_frames(200 if (self.frame_count > 1) else 10000) # wait 10 seconds for first frame
        except Exception as e:
            logging.error(e)
            return
        #
        # convert camera frames to images
        #
        if self.enable_rgb or self.enable_depth:
            # Align the depth frame to color frame
            aligned_frames = self.align.process(frames) if self.enable_depth and self.enable_rgb else None
            depth_frame = aligned_frames.get_depth_frame() if aligned_frames is not None else frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame() if aligned_frames is not None else frames.get_color_frame()

            # Convert images to numpy arrays
            self.depth_frame = depth_frame
            self.color_frame = color_frame
            self.depth_image = np.asanyarray(depth_frame.get_data()) if self.enable_depth else None
            self.color_image = np.asanyarray(color_frame.get_data()) if self.enable_rgb else None
        
        # return original images including color image and depth image(CV form) along wiht color,depth frame(for pyrealsense)
        return self.color_image, self.depth_image, self.depth_frame, self.color_frame
    
    #  Display frame as video streaming
    def _display(self):
        
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('Usefulimage', cv2.WINDOW_AUTOSIZE)
        while self.running:
            
            # 
            # Display
            # 
            # # 1. display normal video without marker detection
            # color_image,depth_image,depth_frame,color_frame = self._capture()
            
            # # 2. display video with marker detection
            display_image = self.markerprocess()
           
            # Show images
            # Make sure to get the useful image 
            if display_image is not None:
                cv2.imshow('RealSense',display_image)
            
            # Press esc or 'q' to close the image window
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

    # For displaying to test the marker detection visually
    def markerprocess(self):
        
        # Capture the frame first
        color_image,depth_image,depth_frame,color_frame=self._capture()

        # Aruco marker part
        # Detect the markers in the image
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(color_image, self.dictionary, parameters=self.parameters)

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
                cv2.circle(color_image,centre,1,(0,0,255),8)
            
            # Compute distance when matching the conditions
            # print(result_center)
            # print(len(result_center))
            if len(result_center)<4:
                print("No enough marker detected")
            
            if len(result_center)>=4:
                # To avoid keyerror
                # start = time.time()
                try:
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
                    point_0 = self.pixel2point(depth_frame,p_0)           
                    point_5 = self.pixel2point(depth_frame,p_5)
                    point_4 = self.pixel2point(depth_frame,p_4)
                    point_3 = self.pixel2point(depth_frame,p_3)
                    # Calculate target point
                    point_target=[point_4[0]+point_3[0]-point_5[0],point_4[1]+point_3[1]-point_5[1],point_4[2]+point_3[2]-point_5[2]]
                    # Compute distance
                    # dis=distance_3dpoints(point_target,point_0)

                    # Display target and draw a line between them
                    color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
                    target_pixel = rs.rs2_project_point_to_pixel(color_intrin, point_target)
                    target_pixel[0] = int(target_pixel[0])
                    target_pixel[1] = int(target_pixel[1])
                    cv2.circle(color_image,tuple(target_pixel),1,(0,0,255),8)
                    cv2.line(color_image,tuple(p_0),tuple(target_pixel),(0,255,0),2)
                
                    # Euclidean distance
                    dis_obj2target = self.distance_3dpoints(point_0,point_target) 
                    dis_obj2target_goal = dis_obj2target*np.sin(np.arccos(0.02/dis_obj2target))

                    
                    # print(dis_obj2target_goal)
                    # images = cv2.aruco.drawDetectedMarkers(images, markerCorners, borderColor=(0, 0, 255))
                    # end = time.time()
                    # print(str(end-start))
                
                except KeyError:
                    print("Keyerror!!!")

            # Outline all of the markers detected in our image
            # images = cv2.aruco.drawDetectedMarkers(images, markerCorners, borderColor=(0, 0, 255))
                # self.images = color_image
        self.images = cv2.aruco.drawDetectedMarkers(color_image, markerCorners, borderColor=(0, 0, 255))
        
        return self.images
    
    def get_marker_position(self):
        
        # Capture the frame first
        color_image,depth_image,depth_frame,color_frame=self._capture()

        # Aruco marker part
        # Detect the markers in the image
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(color_image, self.dictionary, parameters=self.parameters)

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
                centre = tuple(centre)
                result_center[key]= centre

        return result_center

    def get_distance(self):
        
        start_time=time.time()
        temp = self.get_marker_position()
        while (len(temp)<4):
            temp = self.get_marker_position() 
            end=time.time()
            if(end-start_time>0.02):
                print("No enough markers detected for distance computation")
                return 0
                # break
        
        try:
            # Moving object localization marker
            x_id0 = temp[0][0]
            y_id0 = temp[0][1]
            p_0 = [x_id0,y_id0]
            
            # Target object localization marker 
            # Single ID-5
            x_id5 = temp[5][0]
            y_id5 = temp[5][1]
            p_5 = [x_id5,y_id5]
            
            # Dual ID-4 and ID-3
            x_id4 = temp[4][0]
            y_id4 = temp[4][1]
            p_4 = [x_id4,y_id4]

            x_id3 = temp[3][0]
            y_id3 = temp[3][1]
            p_3 = [x_id3,y_id3]

            # Deproject pixel to 3D point
            point_0 = self.pixel2point(self.depth_frame,p_0)           
            point_5 = self.pixel2point(self.depth_frame,p_5)
            point_4 = self.pixel2point(self.depth_frame,p_4)
            point_3 = self.pixel2point(self.depth_frame,p_3)
            # Calculate target point
            point_target=[point_4[0]+point_3[0]-point_5[0],point_4[1]+point_3[1]-point_5[1],point_4[2]+point_3[2]-point_5[2]]
            
            # Compute distance
            # dis=distance_3dpoints(point_target,point_0)

            # Display target and draw a line between them
            # color_intrin = self.color_frame.profile.as_video_stream_profile().intrinsics
            # target_pixel = rs.rs2_project_point_to_pixel(color_intrin, point_target)
            # target_pixel[0] = int(target_pixel[0])
            # target_pixel[1] = int(target_pixel[1])
            # cv2.circle(color_image,tuple(target_pixel),1,(0,0,255),8)
            # cv2.line(color_image,tuple(p_0),tuple(target_pixel),(0,255,0),2)
        
            # Euclidean distance
            dis_obj2target = self.distance_3dpoints(point_0,point_target) 
            dis_obj2target_goal = dis_obj2target*np.sin(np.arccos(0.02/dis_obj2target))

            
            # print(dis_obj2target_goal)
            # useful_color_image=color_image
            # images = cv2.aruco.drawDetectedMarkers(images, markerCorners, borderColor=(0, 0, 255))
            # end = time.time()
            # print(str(end-start))
        
        except KeyError:
            print("Keyerror!!!")

        return dis_obj2target_goal
        



        

                          

    def pixel2point(self,frame,u):
        
        u_x = u[0]
        u_y = u[1]
        # Get depth from pixels
        dis2cam_u = frame.get_distance(u_x,u_y)
        # Convert pixels to 3D coordinates in camera frame(deprojection)
        depth_intrin = frame.profile.as_video_stream_profile().intrinsics
        u_pos = rs.rs2_deproject_pixel_to_point(depth_intrin, [u_x, u_y], dis2cam_u)

        return u_pos

    # Distance computation through pixels 
    def distance_pixel(self,frame,u,v):
        
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
    def distance_3dpoints(self,u,v):
        
        dis_obj2target = np.sqrt(pow(u[0]-v[0],2)+pow(u[1]-v[1],2)+pow(u[2]-v[2],2))

        return dis_obj2target

    def shutdown(self):
        self.running = False
        time.sleep(0.1)
        if self.pipeline is not None:
            self.pipeline.stop()


if __name__ == "__main__":

    try:
        enable_rgb = True
        enable_depth = True
        width = 640
        height = 480
        camera = Camera(width=width, height=height, enable_rgb=enable_rgb, enable_depth=enable_depth)
        camera._display()
  
    finally:
        camera.shutdown()
    
