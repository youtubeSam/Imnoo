import cv2
import numpy as np

COLOR_GREEN = (255, 0, 0)
COLOR_GRAY = (125, 125, 125)

class QRdrawer:
    camShape = None;
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 1
    textColor = (255, 0, 0)
    thickness = 2
        
    hspace = 5
    vspace = 5

    def __init__(self, camShape):
        self.camShape = camShape;

    def drawPolygon(self, img, de):
        poly = de.polygon
        color = COLOR_GREEN
        linewidth = 3
        
        cv2.line(img, poly[0], poly[1], color, linewidth)
        cv2.line(img, poly[1], poly[2], color, linewidth)
        cv2.line(img, poly[2], poly[3], color, linewidth)
        cv2.line(img, poly[3], poly[0], color, linewidth)

    def drawStatusBar(self, img, QRfilesDict):
        for qfname in QRfilesDict:
            mat = self.getStatusBar(QRfilesDict[qfname] )
            img = np.hstack((img, mat))  
        return img
   
    def getStatusBar(self, qf):
        label_width1, label_height1 = self.getTextDimensions(qf.fname)            
        progressStringMax = "("+str(qf.nCodes)+"/"+str(qf.nCodes)+")"
        progressString = "("+str(qf.nDetections)+"/"+str(qf.nCodes)+")"
        label_width2, label_height2 = self.getTextDimensions(progressStringMax)            
        label_width = max(label_width1, label_width2) 
        checkBarShape = (self.camShape[0], label_width + 5, self.camShape[2])
        orgFilename = (2, label_height1 + 5)
        orgProgressText = (2, label_height1 + label_height2 + 15)
        mat = np.zeros(checkBarShape, np.uint8)+255
        cv2.putText(mat,qf.fname, orgFilename, self.font, self.fontScale, self.textColor, self.thickness, cv2.LINE_AA )
        cv2.putText(mat,progressString , orgProgressText, self.font, self.fontScale, self.textColor, self.thickness, cv2.LINE_AA )
        
        return mat 

 
    def getTextDimensions(self, label):
        (label_width, label_height), baseline = cv2.getTextSize(label, self.font, self.fontScale, self.thickness)
        return label_width, label_height
