#!/usr/bin/env python

import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

center1 = [0, 0]
center2 = [0, 0]
depth1 = 0
depth2 = 0
tag1missing = 0
tag2missing = 0

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        # Convert RGB image into black/white
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        # Choose which tags are being used so they can be recognized
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        parameters = aruco.DetectorParameters_create()

        # Detect tags with ids and corresponding corners
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        tag1found=False
        tag2found=False

        for i in range(0, len(corners)):
            if ids[i] == 1:
                tag1found=True
            elif ids[i] == 2:
                tag2found=True

        if not tag1found:
            tag1missing+=1
        if not tag2found:
            tag2missing+=1

        # If tag found, check its location to find the distance to it
        for i in range(0, len(corners)):
            if ids[i] == 1:
                tag1missing=0
                center1 = np.mean(corners[i][0], axis=0)
                depth1 = depth_frame.get_distance(center1[0], center1[1])
            elif ids[i] == 2:
                tag2missing=0
                center2 = np.mean(corners[i][0], axis=0)
                depth2 = depth_frame.get_distance(center2[0], center2[1])

        print("Tag1: x=", center1[0], " y=", center1[1], " depth=", depth1, " missing=", tag1missing)
        print("Tag2: x=", center2[0], " y=", center2[1], " depth=", depth2, " missing=", tag2missing)


finally:
    # Stop streaming
    pipeline.stop()
