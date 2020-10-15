#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Generate measurements from YOLO-Detections and depth image
"""

import argparse

import cv2
from cv_bridge import CvBridge
bridge = CvBridge()

import numpy as np

import os
import subprocess
import time

import rosbag
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8

from Plotter import *
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from Stamp import *
from FileWriter import *
from Plotter import *
from Quaternions import *
from Patch import *
from CameraParameters import *
import ImagePainter


################
## User settings
################

# General
verbose = True
generateVideo = True
fNameVideo = 'simVideo'
figureSize = [8.8,4]

# Input image size
imgWidth = 640
imgHeight = 480

## Ros topics
ODOMETRY_TOPIC =        "/maplab_rovio/T_G_I"       # Odometry information of IMU frame
DEPTH_IMAGE_TOPIC =     "/depth_registered/image"   # Depth information
BOUNDING_BOXES_TOPIC =  "/ncs_yolo/bounding_boxes"  # Bounding boxes
DETECTION_IMAGE_TOPIC = "/ncs_yolo/detection_image" # Bounding boxes
IMAGE_TOPIC =           "/rgb_undistorted_denoised/image" # Image for test

## Output files
f_odometry = FileWriter("odometry.dat")
f_measurements = FileWriter("measurements.dat")

## Setup patch object
Patch.imgWidth = imgWidth-1;
Patch.imgHeight = imgHeight-1;

## Inverse camera matrix
Kinv = np.linalg.inv(K);
R_imu_cam = quadToRot(q_imu_cam)

## Figures
#ax3d = fig3d.gca(projection='3d')
fig = plt.figure(figsize=figureSize)
#fig.tight_layout(pad=3.0)
ax1 = fig.add_subplot(121)
ax = fig.add_subplot(122)

# Poses 3D
#origin = np.zeros((1,3))
#rotOrigin = np.identity(3)
#plotQuadCoordinateFrame(ax,x_world_imu,q_world_imu)
#plotCoordinateFrame(ax3d,origin,rotOrigin)

traj3d = []
measurements3d = []

if generateVideo:
  try:
    os.mkdir("videos")
  except:
    pass  

def flipBoundingBoxes(bb):
  for b in bb.bounding_boxes:
    max = w-b.xmin
    min = w-b.xmax
    b.xmin = min
    b.xmax = max
    max = h-b.ymin
    min = h-b.ymax
    b.ymin = min
    b.ymax = max
  return bb

def extractMeasurements(t,depthMsg,boundingBoxMsg,odometryMsg,imageMsg):
  global f_odometry
  newMeasurements = []
  cylinderR = 1
  boundingBoxMinPixelSpace = 15;
  currPos = [odometryMsg.pose.position.x,
             odometryMsg.pose.position.y,
             odometryMsg.pose.position.z]
  traj3d.append(currPos)

  depthImage = bridge.imgmsg_to_cv2(depthMsg, desired_encoding="32FC1")
  # Rotate RGB image if necessary
  depthImage = cv2.rotate(depthImage,cv2.ROTATE_180);
  rgbImage = bridge.imgmsg_to_cv2(imageMsg, desired_encoding="bgr8")

  patches = []
  for b in boundingBoxMsg.bounding_boxes:
    # Check if desired object class
    if(b.Class != 'car' and b.Class != 'truck'):
      continue
    xmin = max(b.xmin,0)
    ymin = max(b.ymin,0)
    xmax = b.xmax
    ymax = b.ymax

    # Check if detection not at boundary
    if(xmin < boundingBoxMinPixelSpace or
       ymin < boundingBoxMinPixelSpace or
       xmax > imgWidth-1-boundingBoxMinPixelSpace or
       ymax > imgHeight-1-boundingBoxMinPixelSpace):
        continue
    
    p = Patch(depthImage[ymin:ymax+1,xmin:xmax+1],xmin,xmax,ymin,ymax)

    # Only use detections with depth information inside circle
    if ~np.isnan(p.medianDepthCircle):
      patches.append(p)

  # Sort patches according to minimum depth values inside bounding boxes
  patches.sort(key=operator.attrgetter('minDepth'))

  qi = [odometryMsg.pose.orientation.w,
        odometryMsg.pose.orientation.x,
        odometryMsg.pose.orientation.y,
        odometryMsg.pose.orientation.z]
  R_world_imu = quadToRot(qi)
  
  for p in patches:
    rgbImage = ImagePainter.markPatch(rgbImage, p)
    pxCoords = p.getFlippedCenterCoordinates()
    coord3d = Kinv.dot(pxCoords.T)
    coord3d = coord3d/np.linalg.norm(coord3d)*(p.medianDepthCircle+cylinderR) # Measurement vector in cameara frame
    coord3d = R_imu_cam.dot(coord3d) - np.array(x_imu_cam)
    coord3d = R_world_imu.dot(coord3d) - np.array(x_world_imu) + np.array(currPos)
    coord3d = coord3d.tolist()
    #coord3d = (np.array(currPos)+(R_world_imu.dot(R_imu_cam.dot(coord3d)))-np.array(x_world_imu)).tolist()
    measurements3d.append(coord3d)
    newMeasurements.append(coord3d)
    f_measurements.writeMeasurement(t,coord3d)
  f_odometry.writeOdometry(t,odometryMsg)
  return rgbImage, patches, newMeasurements




def main():
  parser = argparse.ArgumentParser(description="Generate measurements from YOLO detections and depth image.")
  parser.add_argument("in_bag_file", help="Input ROS bag.")
  args = parser.parse_args()

  boundingBoxStamp = Stamp()
  depthStamp = Stamp()

  hasImageInfo = False    # Check if bounding box and depth image arrived
  depthMsg = None
  boundingBoxMsg = None
  t0 = None
  i = 0

  nMeasurements = 0

  for topic, msg, t in rosbag.Bag(args.in_bag_file).read_messages():
    if topic == ODOMETRY_TOPIC and hasImageInfo:
      if t0 == None:
        t0 = depthStamp
      [rgbImage, patches, newMeas] = extractMeasurements(depthStamp.minus(t0),depthMsg,boundingBoxMsg,msg,imageMsg)
      ax.clear()
      showImage(ax1, rgbImage)
      ax1.set_title("Frame %i"%(i))
      ax1.set_position([0.03,0.11,0.5,0.77])
      pos = ((np.array(traj3d)-np.array(x_world_imu)).transpose()).transpose();
      #plotTrajectory3d(ax,pos)
      hTray = plotTrajectory2d(ax, pos[:,0:2])
      hTray[0].set_label('ROVIO Trajectory')
      hMeasPoint = plot2dMeasurementPoints(ax, np.array(measurements3d))
      hMeasPoint.set_label('Measurement point')
      hMeas = plot2dMeasurements(ax,pos[-1,:],newMeas)
      hMeas[0].set_label('Measurement vector')
      qi = [msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z]
      nMeasurements += len(newMeas)
      ## 3d Plots
      #plotFieldOfView(ax3d, pos[-1,:], qMult( qInv(q_world_imu),qi))
      #plotQuadCoordinateFrame(ax3d,pos[-1,:],qMult(qi,q_imu_cam))

      #plotQuadCoordinateFrame(ax,pos[-1,:],qMult(qi,q_imu_cam))
      orient = np.array(quadToRot(qMult(qi,q_imu_cam)))
      plotCoordinateFrame2d(ax, pos[-1,:], orient)

      ax.axis([-3, 12, -1, 14])
      ax.grid()
      #ax.legend(bbox_to_anchor=(-0.1, 1))
      ax.legend(loc=1, prop={'size': 9})
      ax.set_position([0.58, 0.12, 0.38, 0.77])
      plt.xlabel('x (m)')
      plt.ylabel('y (m)')
      tAct = t.to_time()-t0.getFloat()
      #plt.title()
      #plt.show()
      hasImageInfo  = False

      if generateVideo:
        plt.savefig("videos/file%03d.png" % i)
      i += 1
      if verbose:
        print("Processed " + str(i) + " frames, " + str(nMeasurements) + " measurements.") 
    if topic == DEPTH_IMAGE_TOPIC:
      depthStamp = Stamp()
      depthStamp.set(msg.header.stamp)
      depthMsg = msg
      if depthStamp.isEqual(boundingBoxStamp):
        hasImageInfo = True

    if topic == BOUNDING_BOXES_TOPIC:
      boundingBoxStamp = Stamp()
      boundingBoxStamp.set(msg.header.stamp)
      boundingBoxMsg = msg
      if boundingBoxStamp.isEqual(depthStamp):
        hasImageInfo = True

    if topic == IMAGE_TOPIC:
      imageStamp = Stamp()
      imageStamp.set(msg.header.stamp)
      imageMsg = msg;
  return


if __name__ == '__main__':
  try:
    main()
    os.chdir("videos")
    try:
      os.remove(fNameVideo+'.mp4')
    except:
      pass
    subprocess.call([
        'ffmpeg', '-framerate', '6', '-i', 'file%03d.png', '-r', '30', '-pix_fmt', 'yuv420p',
        fNameVideo+'.mp4'])

  except rospy.ROSInterruptException:
    pass
