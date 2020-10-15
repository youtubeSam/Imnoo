#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Camera Matrix
K = [[283.6180612179999, 0.0               , 324.01932299260216],
     [0.0              , 283.59733652086277, 230.77136448609954],
     [0.0              , 0.0               , 1.0]]

# Image size
width = 640
height = 480
depthRange = 5

# Coordinate Transformation WORLD -- IMU
x_world_imu = [3.61868340928, 2.26234526228, -0.0798130349149]
q_world_imu = [0.65263402981, -0.720831628019, -0.137397053672, 0.188660109256]

# Coordinate Transformation CAM -- IMU
x_imu_cam = [0.010595359912, -0.00387474494912, -0.0329115388796]
q_imu_cam = [0.00284046632982, 0.706135051133, -0.00548179804573, 0.708050260365]
