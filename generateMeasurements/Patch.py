
import operator
import numpy as np
from matplotlib import pyplot as plt

class Patch:
  consumedIndices = []
  rangeThreshhold = []
  imgWidth = -1
  imgHeight = -1

  def __init__(self ,img ,xmin ,xmax ,ymin ,ymax):
    '''
    :param img: Depth image patch
    :param xmin: min x-coordinate in entire image
    :param xmax: max x-coordinate in entire image
    :param ymin: min y-coordinate in entire image
    :param ymax: max y-coordinate in entire image
    '''

    self.img = img

    self.xmin = xmin
    self.ymin = ymin
    self.xmax = xmax
    self.ymax = ymax

    self.w = (xmax - xmin) + 1    # Width of patch
    self.h = (ymax - ymin) + 1    # Height of patch

    self.minDepth = np.nan        # Min depth inside patch
    self.maxDepth = np.nan        # Max depth inside patch

    self.medianDepthCircle = np.nan   # Median depth inside circle located at center
    self.medianDepthBox = np.nan      # Median depth inside entire box

    self.hasCircleMeasurement = self.computeMedianDepthCircle()
    self.hasBoxMeasurement = self.computeMedianDepthBox()

    self.indCircle = []
    self.indBox = []

    self.Npixels = self.w * self.h

    self.xc = xmin + np.round(self.w/2).astype(int)
    self.yc = ymin + np.round(self.h/2).astype(int)

    self.noncovered = np.ones((self.h ,self.w))
    self.overlappingPatches = []
    self.nonOverlappingPatches = []
    self.coveragePercentage = dict()

  def getFlippedCenterCoordinates(self):
    '''
    Flip absolute pixel coordinates of center of detection. Used in case of raw images are rotated.
    :return: flipped center pixel coordinates
    '''
    return np.array([self.imgWidth-self.xc, self.imgHeight-self.yc, 1])

  def getCenterCoordinates(self):
    '''
    Get absolute pixel coordinates of center of detection.
    :return: center pixel coordinates
    '''
    return np.array([self.xc, self.yc, 1])

  def computeMedianDepthBox(self):
    '''
    Compute median depth inside the entire bounding box
    :return: true: has depth information, false: no depth information
    '''
    values = self.img[np.where(~np.isnan(self.img))]
    self.medianDepthBox = np.median(values)
    if(np.isnan(self.medianDepthBox)):
      return False
    self.minDepth = np.min(values)
    self.maxDepth = max(values)
    return True

  def computeMedianDepthCircle(self):
    '''
    Compute median depth inside a circle located at center of bounding box
    :return: true: has depth information, false: no depth information
    '''
    cId = self.getPixelCircle(min(self.w, self.h)/8)
    values = self.img[cId]
    self.medianDepthCircle = np.median(values[np.where(~np.isnan(values))])
    return np.isnan(self.medianDepthCircle)

  def isOverlapping(self,p2):
    '''
    :param p2: Patch to compare with
    :return: (true / false) if patch is overlapping
    '''
    p1 = self

    if p2 in self.overlappingPatches:
      return True
    if p2 in self.nonOverlappingPatches:
      return False

    p2.coveragePercentage[p1] = 0
    p1.coveragePercentage[p2] = 0

    xmina = min(p1.xmin ,p2.xmin)
    xmaxa = max(p1.xmax ,p2.xmax)

    ymina = min(p1.ymin ,p2.ymin)
    ymaxa = max(p1.ymax ,p2.ymax)

    a = ymaxa - ymina + 1
    b = xmaxa - xmina + 1

    k1 = np.zeros((a ,b))
    k2 = np.zeros((a ,b))

    k1[(p1.ymin - ymina):(p1.ymax - ymina + 1) , (p1.xmin - xmina):(p1.xmax - xmina + 1)] = 1
    k2[(p2.ymin - ymina):(p2.ymax - ymina + 1) , (p2.xmin - xmina):(p2.xmax - xmina + 1)] = 1

    r = k1 + k2 - 1
    r = r.clip(min = 0)

    # Number of overlapping pixels
    count = np.sum(r)

    self.coveragePercentage[p2] = count/self.Npixels
    p2.coveragePercentage[self] = count/self.Npixels

    if count > 0:
      p1.overlappingPatches.append(p2)
      p2.overlappingPatches.append(p1)
      return True

    return False


  def getPixelCircle(self, r):
    '''
    :param r: Pixel radius, used min(w,h)/8, integer valued
    :return: tuple ([y0,y1,...],[x0,x1,..]) of pixels inside the circular center area
    '''
    r = np.round(r).astype(int)
    xc = np.round(self.w/2).astype(int)
    yc = np.round(self.h/2).astype(int)
    if r == 0:
      return (tuple([np.array([yc]),np.array([xc])]))
    c=[]
    x = []
    y = []
    xtest = []
    ytest = []
    if min(xc,yc) < r:
      return None
  
    r2 = r**2
    for i in np.arange(r+1):
      i2 = i**2  
      for j in np.arange(r+1):
        if i == 0 and j == 0:
          x.append(j+xc)
          y.append(i+yc)
          continue
        if i == 0:
          x.append(j+xc)
          y.append(i+yc)
          x.append(-j+xc)
          y.append(i+yc)
          continue
        if j == 0:
          x.append(j+xc)
          y.append(i+yc)
          x.append(j+xc)
          y.append(-i+yc)
          continue
        if j**2+i**2 <= r2:
          x.append(j+xc)
          y.append(i+yc)
          x.append(j+xc)
          y.append(-i+yc)
          x.append(-j+xc)
          y.append(i+yc)
          x.append(-j+xc)
          y.append(-i+yc)
    return tuple([np.array(y),np.array(x)])
