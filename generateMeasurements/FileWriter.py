#!/usr/bin/env python
# -*- coding: utf-8 -*-

class FileWriter:

  def __init__(self,filename):
    self.filename = filename
    self.odometryIndex = 0
    self.measurementIndex = 0
    self.lastMeasurementTime = -1;
    f = open(self.filename,"w")
    f.close()

  def writeOdometry(self,time,odometryMsg):
    f = open(self.filename,"a+")
    line = '{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n'.format(self.odometryIndex,
                                                      time.getFloat(),
                                                      odometryMsg.pose.position.x,
                                                      odometryMsg.pose.position.y,
                                                      odometryMsg.pose.position.z,
                                                      odometryMsg.pose.orientation.w,
                                                      odometryMsg.pose.orientation.x,
                                                      odometryMsg.pose.orientation.y,
                                                      odometryMsg.pose.orientation.z)
    f.writelines(line)
    self.odometryIndex += 1
    f.close()

  def writeMeasurement(self,time,measurement):
    f = open(self.filename,"a+")
    t = time.getFloat()
    if (t != self.lastMeasurementTime):
      self.lastMeasurementTime = t
      self.measurementIndex += 1

    line = '{}\t{}\t{}\t{}\t{}\n'.format(self.measurementIndex,
                                         t,
                                         measurement[0],
                                         measurement[1],
                                         measurement[2])
    f.writelines(line)
    f.close()
