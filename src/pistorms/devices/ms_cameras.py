from .ms_device import MindsensorsI2CDevice


class BLOB:
    """Tracked object returned by NXTCam / NXTCam5 getBlobs()."""

    def __init__(self, color, left, top, right, bottom):
        self.color  = color
        self.left   = left
        self.top    = top
        self.right  = right
        self.bottom = bottom


class _BlobCamera(MindsensorsI2CDevice):
    """Common part of NXTCam and NXTCam5: the list of tracked objects.
    Bus errors raise OSError."""

    NumberObjects = 0x42
    Color         = 0x43
    X_Top         = 0x44
    Y_Top         = 0x45
    X_Bottom      = 0x46
    Y_Bottom      = 0x47
    BLOB_SIZE     = 5     # bytes per tracked object: color, x/y top, x/y bottom

    def getNumberObjects(self):
        return self.readByte(self.NumberObjects)

    def getBlobs(self, blobNum=1):
        """Object number blobNum (from 1), or None if fewer objects are tracked."""
        if blobNum > self.getNumberObjects():
            return None
        offset = (blobNum - 1) * self.BLOB_SIZE
        return BLOB(
            self.readByte(self.Color    + offset),
            self.readByte(self.X_Top    + offset),
            self.readByte(self.Y_Top    + offset),
            self.readByte(self.X_Bottom + offset),
            self.readByte(self.Y_Bottom + offset),
        )


class NXTCam5(_BlobCamera):

    CAM_ADDRESS   = 0x02
    CAM_COMMAND   = 0x41

    DEFAULT_ADDRESS = CAM_ADDRESS

    def trackLine(self):              self.command(76)    # 'L'
    def trackObject(self):            self.command(79)    # 'O'
    def trackFace(self):              self.command(70)    # 'F'
    def trackEye(self):               self.command(101)   # 'e'
    def captureImage(self):           self.command(80)    # 'P'
    def captureShortVideo(self):      self.command(77)    # 'M'
    def captureContinuousVideo(self): self.command(82)    # 'R'


class NXTCam(_BlobCamera):

    NXTCAM_ADDRESS = 0x02

    DEFAULT_ADDRESS = NXTCAM_ADDRESS

    def sortSize(self):            self.command(65)   # 'A'
    def trackObject(self):         self.command(66)   # 'B'
    def writeImageRegisters(self): self.command(67)   # 'C'
    def stopTracking(self):        self.command(68)   # 'D'
    def startTracking(self):       self.command(69)   # 'E'
    def getColorMap(self):         self.command(71)   # 'G'
    def readImageRegisters(self):  self.command(72)   # 'H'
    def illuminationOn(self):      self.command(73)   # 'I'
    def trackLine(self):           self.command(76)   # 'L'
    def ping(self):                self.command(80)   # 'P'
    def reset(self):               self.command(82)   # 'R'
    def sendColorMap(self):        self.command(83)   # 'S'
    def illuminationOff(self):     self.command(84)   # 'T'
    def sortColor(self):           self.command(85)   # 'U'
    def firmware(self):            self.command(86)   # 'V'
    def sortNone(self):            self.command(88)   # 'X'
