import re

class QRdetection:
    fname = None
    nCodes = None
    codeID = None
    text = None
    polygon = None
    isValid = False
    def __init__(self, detection):
        self.polygon = detection.polygon
        dataString = detection.data.decode('utf-8') 
        try:
            pt = re.compile(r'(.+)/(\d+)/(\d+)\n(.+)', re.DOTALL)
            matches = pt.findall(dataString)
            self.fname = matches[0][0]
            self.codeID = int(matches[0][1])
            self.nCodes = int(matches[0][2])
            self.text = matches[0][3]
            self.isValid = True
        except:
            self.text = dataString;
            self.codeID = 1
            self.nCodes = 1
