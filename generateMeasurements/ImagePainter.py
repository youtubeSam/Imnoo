import cv2

def markPatch(img,patch):
  '''
  :param img: Entire image
  :param patch: Patch object
  :return: Image with marked patch
  '''
  col = (255,255,0)                     # Drawing color
  img = drawCrosshair(img, patch, col)  # Draw crosshair hat center
  img = writeDistance(img, patch, col)  # Write median distance
  imt = drawBoundingBox(img, patch, col)
  return img

def writeDistance(img, patch, color):
  '''
  :param img: Entire image
  :param patch: Patch object
  :param color: Text color 
  :return:
  '''
  font = cv2.FONT_HERSHEY_SIMPLEX
  fontScale = 1
  thickness = 2
  org = (patch.xc+4,patch.yc+28)
  distString = "%.1f m" % patch.medianDepthCircle
  img = cv2.putText(img, distString, org, font,
                      fontScale, color, thickness, cv2.LINE_AA)
  return img

def drawCrosshair(img, patch, col):
  linewidth = 1
  size = 20
  xc = patch.xc
  yc = patch.yc
  img[yc - linewidth:yc + linewidth, xc - size:xc + size, :] = col
  img[yc - size:yc + size, xc - linewidth:xc + linewidth, :] = col
  return img

def drawBoundingBox(img, patch, color):
  pt1 = (patch.xmin, patch.ymin)
  pt2 = (patch.xmin+patch.w, patch.ymin+patch.h)
  img = cv2.rectangle(img, pt1, pt2, color, thickness=2, lineType=8, shift=0)
  return img

def pixelPaint(img, coords, color):
  for (y, x) in coords:
    img[y, x, :] = color
  print(img[y, x, :])
  return img
