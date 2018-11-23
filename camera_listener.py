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

center = [0, 0, 0]
tagMissing = 0
iterations=0
corner1=[0,0,0]
corner2=[0,0,0]
corner3=[0,0,0]
corner4=[0,0,0]
den=[0,0,0,0]

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

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

        if not tagFound:
            tagMissing += 1

        if tagFound:
            # If tag found, check its location to find the distance to it
            for i in range(0, len(corners)):
                if ids[i] == 1:
                    iterations = iterations + 1
                    corner1temp = [corners[i][0][0][0], corners[i][0][0][1], 0]
                    corner2temp = [corners[i][0][1][0], corners[i][0][1][1], 0]
                    corner3temp = [corners[i][0][2][0], corners[i][0][2][1], 0]
                    corner4temp = [corners[i][0][3][0], corners[i][0][3][1], 0]
                    corner1temp[2] = depth_frame.get_distance(corner1temp[0], corner1temp[1])
                    corner2temp[2] = depth_frame.get_distance(corner2temp[0], corner2temp[1])
                    corner3temp[2] = depth_frame.get_distance(corner3temp[0], corner3temp[1])
                    corner4temp[2] = depth_frame.get_distance(corner4temp[0], corner4temp[1])

                    if corner1temp[2] > 0.1:
                        corner1=np.add(corner1, corner1temp)
                        den[0]=den[0]+1
                    if corner2temp[2] > 0.1:
                        corner2 = np.add(corner2, corner2temp)
                        den[1]=den[1]+1
                    if corner3temp[2] > 0.1:
                        corner3 = np.add(corner3, corner3temp)
                        den[2] = den[2] + 1
                    if corner4temp[2] > 0.1:
                        corner4 = np.add(corner4, corner4temp)
                        den[3] = den[3] + 1

                    RotateAroundZAxis = -np.mean(np.array(
                        [math.atan2((corner2temp[1] - corner1temp[1]), (corner2temp[0] - corner1temp[0])), math.atan2((corner2temp[0] - corner3temp[0]), (corner3temp[1] - corner2temp[1])),
                         math.atan2((corner3temp[1] - corner4temp[1]), (corner3temp[0] - corner4temp[0])), math.atan2((corner1temp[0] - corner4temp[0]), (corner4temp[1] - corner1temp[1]))]))
                    #RotateAroundZAxis = math.atan2((corner2temp[1] - corner1temp[1]), (corner2temp[0] - corner1temp[0]))



                    if iterations == 30:
                        tagMissing = 0
                        if den[0] != 0:
                            corner1 = [corner1[0] / den[0], corner1[1] / den[0], corner1[2] / den[0]]
                        if den[1] != 0:
                            corner2 = [corner2[0]/den[1],corner2[1]/den[1],corner2[2]/den[1]]
                        if den[2] != 0:
                            corner3 = [corner3[0]/den[2],corner3[1]/den[2],corner3[2]/den[2]]
                        if den[3] != 0:
                            corner4 = [corner4[0] / den[3], corner4[1] / den[3], corner4[2] / den[3]]

                        u = [corner1[0]-corner4[0], corner1[1]-corner4[1], corner1[2]-corner4[2]]
                        v = [corner3[0]-corner4[0], corner3[1]-corner4[1], corner3[2]-corner4[2]]
                        temp1=[u[0],u[1]]
                        temp2=[v[0],v[1]]
                        scale = 0.11*2/(np.linalg.norm(temp1)+np.linalg.norm(temp2))
                        v = [v[0]*scale, v[1]*scale, v[2]]
                        u = [u[0] * scale, u[1] * scale, u[2]]

                        n = np.cross(u, v)
                        nnorm=np.linalg.norm(n)
                        n=[n[0]/nnorm,n[1]/nnorm,n[2]/nnorm]
                        #print(n)
                        #print("---------------")
                        xy = np.mean(corners[i][0], axis=0)
                        center = [xy[0], xy[1], depth_frame.get_distance(center[0], center[1])]
                        v = [(center[0] - 640/2) * scale, (center[1] - 480/2) * scale, center[2]]
                        #print(v)
                        #print("--------")

                        print(math.degrees(angle_between([v[1],v[2]],[n[1],n[2]])))


                        corner1 = [0, 0, 0]
                        corner2 = [0, 0, 0]
                        corner3 = [0, 0, 0]
                        corner4 = [0, 0, 0]
                        iterations = 0
                        den=[0,0,0,0]


        else:
            print("error")




finally:
    # Stop streaming
    pipeline.stop()
