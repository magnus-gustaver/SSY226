#!/usr/bin/env python

import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
import math

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Set global variables
center = [0, 0, 0]
iterations=0
averageCorner = np.zeros((4, 3))
den=[0,0,0,0]
markerWidth = 0.11 # real width in meters
imageWidth = 640 # Width of input image in pixels
imageHeight = 480 # Height of input image in pixels

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle between to vectors in radians """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def transform_corner_data(corners, i):
    """ Returns coordinates of the corners of marker i """
    corner1temp = [corners[i][0][0][0], corners[i][0][0][1], depth_frame.get_distance(corners[i][0][0][0], corners[i][0][0][1])]
    corner2temp = [corners[i][0][1][0], corners[i][0][1][1], depth_frame.get_distance(corners[i][0][1][0], corners[i][0][1][1])]
    corner3temp = [corners[i][0][2][0], corners[i][0][2][1], depth_frame.get_distance(corners[i][0][2][0], corners[i][0][2][1])]
    corner4temp = [corners[i][0][3][0], corners[i][0][3][1], depth_frame.get_distance(corners[i][0][3][0], corners[i][0][3][1])]
    return np.array([[corner1temp],[corner2temp],[corner3temp],[corner4temp]])

def change_scale(v, scale):
    """ Input vector with parameters [pixel, pixel, meter] and return vector with meter parameters """
    return [v[0] * scale, v[1] * scale, v[2]]

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

        tagFound = False

        for i in range(0, len(corners)):
            if ids[i] == 1:
                tagFound = True

        if tagFound:
            # If tag found, check its location to find the distance to it
            for i in range(0, len(corners)):
                if ids[i] == 1:
                    iterations = iterations + 1

                    corner = transform_corner_data(corners, i)

                    for j in range(0, 4):
                        if corner[j,2] > 0.1:
                            averageCorner[j,:] = np.add(averageCorner[j,:], corner[j,:])
                            den[j] = den[j]+1
                 
                    if iterations == 30:
                        for j in range(0, 4):
                            if den[j] != 0:
                                averageCorner[j,:] = averageCorner[j,:] / den[j] #averageCorner[j,:] = [averageCorner[j,0] / den[j], averageCorner[j,1] / den[j], averageCorner[j,2] / den[j]]

                        u = np.subtract(averageCorner[0,:], averageCorner[3,:])
                        v = np.subtract(averageCorner[2,:], averageCorner[3,:])

                        # Scale in meter/pixel
                        scale = markerWidth*2/(np.linalg.norm([u[0],u[1]])+np.linalg.norm([v[0],v[1]]))
                        u = change_scale(u, scale)
                        v = change_scale(v, scale)

                        # Normal vector of marker
                        n = np.cross(u, v)
                        n = unit_vector(n)

                        # Get approximate center of marker and create vector there
                        xy = np.mean(corners[i][0], axis=0)
                        center = [xy[0], xy[1], depth_frame.get_distance(center[0], center[1])]
                        cameraToCenter = np.array([center[0] - imageWidth/2, center[1] - imageHeight/2, center[2]])
                        vec = change_scale(cameraToCenter, scale)
                        
                        RotateAroundXAxis = math.degrees(angle_between([vec[1],vec[2]],[n[1],n[2]]))

                        RotateAroundYAxis = math.degrees(angle_between([vec[0],vec[2]],[n[0],n[2]]))

                        RotateAroundZAxis = -np.mean(np.array(
                        [math.atan2((corner[1,1] - corner[0,1]), (corner[1,0] - corner[0,0])), math.atan2((corner[1,0] - corner[2,0]), (corner[2,1] - corner[1,1])),
                         math.atan2((corner[2,1] - corner[3,1]), (corner[2,0] - corner[3,0])), math.atan2((corner[0,0] - corner[3,0]), (corner[3,1] - corner[0,1]))]))

                        averageCorner = np.zeros((4, 3))
                        iterations = 0
                        den=[0,0,0,0]

        else:
            print("error")

finally:
    # Stop streaming
pipeline.stop()
