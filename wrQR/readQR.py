import argparse
import re
import math
import numpy as np
from pyzbar.pyzbar import decode
import cv2
from pdf2image import convert_from_path

from QRdetection import *
from QRfile import *
from QRdrawer import *

CHECK_BAR_SIZE = 100

parser = argparse.ArgumentParser(
        description="Recover textfile from PDF containing QR codes"            
)
parser.add_argument('-p','--pdfFile', help="PDF file containing QR codes")
parser.add_argument('-c','--cameraId', default=0, help="Camera id to be used")
args = parser.parse_args()

# TODO: Allow pdf-read from scanned file
'''
if(args.pdfFile != None):
    print("Using pdf file as input")
    print(args.pdfFile)
    pages = convert_from_path(args.pdfFile)
    for p in pages:
        #decoded = decode(p)
        rgbImg = cv2.cvtColor(np.array(p.convert("RGB")), cv2.COLOR_BGR2GRAY)
        (t,bwImg) = cv2.threshold(rgbImg, 127, 255, cv2.THRESH_BINARY)
        decoded = decode(bwImg)
        cv2.namedWindow('finalImg', cv2.WINDOW_NORMAL)
        for d in decoded:
            print(d.polygon)
            bwImg = drawPolygon(bwImg, d.polygon, (255,0,0))
        cv2.imshow("finalImg", bwImg)
'''            

cap = cv2.VideoCapture(int(args.cameraId));
camShape = None
if not (cap.isOpened()):
    raise RuntimeError("No camera found with id "+args.cameraId+
            ". Try to use a different camera id using -c flag.")
else:
    ret, frame = cap.read()
    camShape = frame.shape

drawer = QRdrawer(camShape)

fileDict = dict()
qFdict = dict()
wasWritten = dict()
detectedOnce = False;

unformattedFiles = [];

while(True):
    # Read camera data    
    ret, frame = cap.read()
    # Detect and decode QR codes
    data = decode(frame)
    for d in data:
        detectedOnce = True;
        de = QRdetection(d)
        if not de.isValid:
            if not (de.text in unformattedFiles):
                unformattedFiles.append(de.text)
                de.fname = "unformatted"+str(len(unformattedFiles))+".txt"
                QRfile(de).writeFile()
            drawer.drawPolygon(frame,de)
            continue
        fname = de.fname
        if not fname in qFdict:    
            # Generate a new QRfile
            qF = QRfile(de)
            qFdict[fname] = qF 
            wasWritten[fname] = False
        else:
            # Append data to existing QRfile
            qF = qFdict[fname]
            qF.addDetection(de)
        if qF.isComplete() and not wasWritten[fname]:
            qF.writeFile()
            wasWritten[fname] = True

        drawer.drawPolygon(frame, de)
    frame = cv2.flip(frame,1)  
    frame = drawer.drawStatusBar(frame, qFdict)
    cv2.imshow("preview", frame)

    if cv2.waitKey(1) & 0xFF == ord("q") or (detectedOnce and not (False in wasWritten.values())):
        break
