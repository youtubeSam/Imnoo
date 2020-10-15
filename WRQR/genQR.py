import sys
import argparse
import re
from math import floor
import numpy as np
import qrcode
from reportlab.pdfgen.canvas import Canvas
from reportlab.lib.units import cm 


##################
## USER PARAMETERS
##################

# Information about input file
IN_ENCODING = "UTF-8"

# Information about output files
DEFAULT_FILENAME = "out"
A4 = [21*cm, 29.7*cm]

# Information about QR-Codes
N_characters_per_code = 200 
PatchSize =     7*cm
nHorizontal =   2
nVertical =     3
hspace = (A4[0]-nHorizontal*PatchSize)/(nHorizontal+1)
vspace = (A4[1]-nVertical*PatchSize)/(nVertical+1)


##################
## IMPLEMENTATION
##################

# Input argument handling
parser = argparse.ArgumentParser(
    description="Generate QR code from textfile"            
)
parser.add_argument('textfile', help="Textfile to convert")
parser.add_argument("-p", '--png', action='store_true' , help = "Generate png files")
parser.add_argument("-o", '--outputFile', default=DEFAULT_FILENAME, help = "Output filename")
args = parser.parse_args()
pf = re.compile(r"/?([^/]+)$")
TXT_FILENAME = pf.findall(args.textfile)[0] 

# Read textfile
f = open(args.textfile,"r", encoding=IN_ENCODING)
lines = f.readlines()
f.close()

# Divide textfile into partitions
partitions = [u""]
nCharacters = 0
nCode = 1

for i, l in enumerate(lines):
    for c in l:
        partitions[nCode-1] += c
        nCharacters += 1
        if nCharacters ==  N_characters_per_code:
            nCharacters = 0
            nCode += 1
            partitions.append("")

# Generate (and save if -p is used) QR-Code images
images = []
qr = qrcode.QRCode(
    version=1,
    error_correction=qrcode.constants.ERROR_CORRECT_H,
    box_size=20,
    border=4,
)

def generateCode(text, filename):
    qr.clear()
    qr.add_data(text)
    qr.make(fit=True)
    img = qr.make_image(fill_color="black", back_color="white")
    images.append(img)
    if(args.png):
        img.save(filename)

for i, p in enumerate(partitions):
    fName = args.outputFile+str(i+1)+".png" 
    text = TXT_FILENAME +"/" + str(i+1) + "/" + str(len(partitions)) + "\n"+p 
    generateCode(text, fName)

# Generate and save PDF File
canvas = Canvas(args.outputFile+".pdf")

for i, img in enumerate(images):
    j = floor((i/nHorizontal)%nVertical)+1 
    k = i%nHorizontal
    p = floor(i/nHorizontal/nVertical)
    if i != 0 and i%(nHorizontal*nVertical) == 0:
        canvas.showPage()
    canvas.drawInlineImage(
        img, 
        hspace+(hspace+PatchSize)*k, 
        A4[1]-(vspace+PatchSize)*j, 
        PatchSize, 
        PatchSize)

canvas.save()
