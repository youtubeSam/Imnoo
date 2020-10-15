#!/usr/bin/env python
# -*- coding: utf-8 -*-

class Stamp:
  def __init__(self):
    self.secs = 0
    self.nsecs = 0

  def set(self,s):
    self.secs = s.secs
    self.nsecs = s.nsecs

  def getDiffTo(self,s):
    res = Stamp()
    res.secs = self.secs-s.secs
    res.nsecs = self.nsecs-s.nsecs
    return res

  def minus(self,s):
    res = Stamp()
    ndiff = self.nsecs-s.nsecs
    if ndiff < 0:
      res.nsecs = 1e9+ndiff
      res.secs = self.secs-s.secs-1
    else:
      res.nsecs = ndiff
      res.secs = self.secs-s.secs
    return res

  def getFloat(self):
    time = self.secs
    time = time + self.nsecs*1e-9
    return time

  def cprint(self):
    buf = "Time = %.10f\n" % (self.getFloat())
    print(buf)

  def isEqual(self,stamp2):
    if self.secs == stamp2.secs and self.nsecs == stamp2.nsecs:
      return True
    return False
