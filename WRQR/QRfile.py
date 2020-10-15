from playsound import playsound

class QRfile:
    fname = None
    detections = []
    checks = []
    nDetections = 0
    nCodes = None 
    # TODO: pass the following as parameter to reader
    doPlaySound = True;

    def __init__(self, det):
        self.fname = det.fname
        self.nCodes = det.nCodes
        self.detections = [None]*det.nCodes
        self.checks = [None]*det.nCodes
        self.addDetection(det)

    def addDetection(self, det):
        if det.fname != self.fname:
            print("Faulty filename is " + det.fname + ", expected \"" + self.fname + "\".")
            return
        idx = det.codeID-1
        if self.detections[idx] != None:
            if det.text != self.detections[idx].text:
                print("Faulty readback")
                return
            else:
                self.checks[idx] += 1
                return

        self.detections[idx] = det
        self.nDetections += 1
        self.checks[idx] = 0
        if not self.isComplete() and self.doPlaySound:
            playsound("sounds/beep.mp3")
        print("Added detection " + str(idx+1) + " / " + str(self.nCodes) + " to QR_" + self.fname + ".")
        
    def isComplete(self):
        return self.nCodes == self.nDetections

    def writeFile(self):
        if not self.isComplete():
            print("File is not complete")
            return
        else:
            out = open("QR_" + self.fname, "w", encoding = "UTF-8")
            for d in self.detections:
                out.write(d.text)
            out.close()
            if self.doPlaySound: 
                playsound("sounds/beepCompleted.mp3")
            print("Data dumped to file QR_" + self.fname)
