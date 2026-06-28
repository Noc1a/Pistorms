import os as _os, sys as _sys
_sys_dir = _os.path.normpath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'sys'))
if _sys_dir not in _sys.path: _sys.path.insert(0, _sys_dir)

from mindsensors_i2c import mindsensors_i2c


class BLOB:
    """Tracked object returned by NXTCam / NXTCam5 getBlobs()."""

    def __init__(self, color, left, top, right, bottom):
        self.color  = color
        self.left   = left
        self.top    = top
        self.right  = right
        self.bottom = bottom


class NXTCam5(mindsensors_i2c):

    CAM_ADDRESS   = 0x02
    CAM_COMMAND   = 0x41
    NumberObjects = 0x42
    Color         = 0x43
    X_Top         = 0x44
    Y_Top         = 0x45
    X_Bottom      = 0x46
    Y_Bottom      = 0x47

    def __init__(self, port, address=CAM_ADDRESS):
        port.activateCustomSensorI2C()
        mindsensors_i2c.__init__(self, address >> 1)

    def command(self, command):
        self.writeByte(self.CAM_COMMAND, int(command))

    def _cmd(self, code):
        try: self.command(code)
        except: print("Error: couldn't write command to Cam."); return ""

    def trackLine(self):             self._cmd(76)
    def trackObject(self):           self._cmd(79)
    def trackFace(self):             self._cmd(70)
    def trackEye(self):              self._cmd(101)
    def captureImage(self):          self._cmd(80)
    def captureShortVideo(self):     self._cmd(77)
    def captureContinuousVideo(self): self._cmd(82)

    def getNumberObjects(self):
        try: return self.readByte(self.NumberObjects)
        except: print("Error: Could not read from Cam."); return ""

    def getBlobs(self, blobNum=1):
        try:
            blobs = self.getNumberObjects()
            if blobNum > blobs:
                print("blobNum is greater than amount of blobs tracked.")
                return 0
            i = blobNum - 1
            return BLOB(
                self.readByte(self.Color    + i * 5),
                self.readByte(self.X_Top    + i * 5),
                self.readByte(self.Y_Top    + i * 5),
                self.readByte(self.X_Bottom + i * 5),
                self.readByte(self.Y_Bottom + i * 5),
            )
        except:
            print("Error: Could not read from Cam.")
            return ""


class NXTCam(mindsensors_i2c):

    NXTCAM_ADDRESS = (0x02)
    COMMAND       = 0x41
    NumberObjects = 0x42
    Color         = 0x43
    X_Top         = 0x44
    Y_Top         = 0x45
    X_Bottom      = 0x46
    Y_Bottom      = 0x47

    def __init__(self, port, address=NXTCAM_ADDRESS):
        port.activateCustomSensorI2C()
        mindsensors_i2c.__init__(self, address >> 1)

    def command(self, command): self.writeByte(self.COMMAND, int(command))
    def _cmd(self, code):
        try: self.command(code)
        except: print("Error: couldn't write command to Cam."); return ""

    def sortSize(self):            self._cmd(65)
    def trackObject(self):         self._cmd(66)
    def writeImageRegisters(self): self._cmd(67)
    def stopTracking(self):        self._cmd(68)
    def startTracking(self):       self._cmd(69)
    def getColorMap(self):         self._cmd(71)
    def readImageRegisters(self):  self._cmd(72)
    def illuminationOn(self):      self._cmd(73)
    def trackLine(self):           self._cmd(76)
    def ping(self):                self._cmd(80)
    def reset(self):               self._cmd(82)
    def sendColorMap(self):        self._cmd(83)
    def illuminationOff(self):     self._cmd(84)
    def sortColor(self):           self._cmd(85)
    def firmware(self):            self._cmd(86)
    def sortNone(self):            self._cmd(88)

    def getNumberObjects(self):
        try: return self.readByte(self.NumberObjects)
        except: print("Error: Could not read from Cam."); return ""

    def getBlobs(self, blobNum=1):
        try:
            blobs = self.getNumberObjects()
            if blobNum > blobs:
                print("blobNum is greater than amount of blobs tracked.")
                return 0
            i = blobNum - 1
            return BLOB(
                self.readByte(self.Color    + i * 5),
                self.readByte(self.X_Top    + i * 5),
                self.readByte(self.Y_Top    + i * 5),
                self.readByte(self.X_Bottom + i * 5),
                self.readByte(self.Y_Bottom + i * 5),
            )
        except:
            print("Error: Could not read from Cam.")
            return ""
