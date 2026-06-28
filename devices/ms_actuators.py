import time
import os as _os, sys as _sys
_sys_dir = _os.path.normpath(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', 'sys'))
if _sys_dir not in _sys.path: _sys.path.insert(0, _sys_dir)

from mindsensors_i2c import mindsensors_i2c


class NXTMMX(mindsensors_i2c):

    MMX_ADDRESS            = (0x06)
    MMX_VOLTAGE_MULTIPLIER = 37
    MMX_Motor_1            = 0x01
    MMX_Motor_2            = 0x02
    MMX_Motor_Both         = 0x03
    MMX_Next_Action_Float     = 0x00
    MMX_Next_Action_Brake     = 0x01
    MMX_Next_Action_BrakeHold = 0x02
    MMX_Direction_Forward  = 0x01
    MMX_Direction_Reverse  = 0x00
    MMX_Move_Relative      = 0x01
    MMX_Move_Absolute      = 0x00
    MMX_Completion_Wait_For  = 0x01
    MMX_Completion_Dont_Wait = 0x00
    MMX_Speed_Full   = 90
    MMX_Speed_Medium = 60
    MMX_Speed_Slow   = 25
    MMX_CONTROL_SPEED    = 0x01
    MMX_CONTROL_RAMP     = 0x02
    MMX_CONTROL_RELATIVE = 0x04
    MMX_CONTROL_TACHO    = 0x08
    MMX_CONTROL_BRK      = 0x10
    MMX_CONTROL_ON       = 0x20
    MMX_CONTROL_TIME     = 0x40
    MMX_CONTROL_GO       = 0x80
    MMX_COMMAND      = 0x41
    MMX_SETPT_M1     = 0x42
    MMX_SPEED_M1     = 0x46
    MMX_TIME_M1      = 0x47
    MMX_CMD_B_M1     = 0x48
    MMX_CMD_A_M1     = 0x49
    MMX_SETPT_M2     = 0x4A
    MMX_SPEED_M2     = 0x4E
    MMX_TIME_M2      = 0x4F
    MMX_CMD_B_M2     = 0x50
    MMX_CMD_A_M2     = 0x51
    MMX_POSITION_M1  = 0x62
    MMX_POSITION_M2  = 0x66
    MMX_STATUS_M1    = 0x72
    MMX_STATUS_M2    = 0x73
    MMX_TASKS_M1     = 0x76
    MMX_TASKS_M2     = 0x77
    MMX_P_Kp         = 0x7A
    MMX_P_Ki         = 0x7C
    MMX_P_Kd         = 0x7E
    MMX_S_Kp         = 0x80
    MMX_S_Ki         = 0x82
    MMX_S_Kd         = 0x84
    MMX_PASSCOUNT    = 0x86
    MMX_PASSTOLERANCE = 0x87

    def __init__(self, port, address=MMX_ADDRESS):
        port.activateCustomSensorI2C()
        mindsensors_i2c.__init__(self, address >> 1)

    def command(self, cmd): self.writeByte(self.MMX_COMMAND, int(cmd))
    def resPos(self):       self.command(82)

    def battVoltage(self):
        try: return self.readByte(self.MMX_COMMAND) * self.MMX_VOLTAGE_MULTIPLIER
        except: print("Error: Could not read voltage"); return ""

    def pos(self, motor_number):
        try:
            if motor_number == 1: return self.readLongSigned(self.MMX_POSITION_M1)
            if motor_number == 2: return self.readLongSigned(self.MMX_POSITION_M2)
        except: print("Error: Could not read encoder position"); return ""

    def setSpeed(self, motor_number, speed):
        ctrl = self.MMX_CONTROL_SPEED | self.MMX_CONTROL_BRK
        if motor_number != self.MMX_Motor_Both: ctrl |= self.MMX_CONTROL_GO
        if motor_number & 0x01: self.writeArray(self.MMX_SPEED_M1, [speed, 0, 0, ctrl])
        if motor_number & 0x02: self.writeArray(self.MMX_SPEED_M2, [speed, 0, 0, ctrl])
        if motor_number == self.MMX_Motor_Both: self.writeByte(self.MMX_COMMAND, 83)

    def _stop(self, motor_number, next_action):
        if next_action in (self.MMX_Next_Action_Brake, self.MMX_Next_Action_BrakeHold):
            cmds = {self.MMX_Motor_1: 65, self.MMX_Motor_2: 66, self.MMX_Motor_Both: 67}
        else:
            cmds = {self.MMX_Motor_1: 97, self.MMX_Motor_2: 98, self.MMX_Motor_Both: 99}
        if motor_number in cmds: self.writeByte(self.MMX_COMMAND, cmds[motor_number])

    def status(self, m):
        if m == 1: return self.readByte(self.MMX_STATUS_M1)
        if m == 2: return self.readByte(self.MMX_STATUS_M2)

    def statusBit(self, motor_number, bitno=0): return (self.status(motor_number) >> bitno) & 1
    def brake(self, m):      self._stop(m, self.MMX_Next_Action_Brake)
    def float(self, m):      self._stop(m, self.MMX_Next_Action_Float)
    def hold(self, m):       self._stop(m, self.MMX_Next_Action_BrakeHold)
    def isBusy(self, m):     return bool(self.status(m) & 0b01001011)
    def isStalled(self, m):  return self.statusBit(m, 7) == 1
    def isOverloaded(self, m): return self.statusBit(m, 5) == 1

    def waitUntilNotBusy(self, motor_number, timeout=-1):
        while self.isBusy(motor_number):
            time.sleep(0.01)
            timeout -= 1
            if timeout == 0: return 1
            if timeout < -5: timeout = -1
        return 0

    def _ticks(self, d):
        return [d % 0x100, (d % 0x10000) // 0x100,
                (d % 0x1000000) // 0x10000, d // 0x1000000]

    def _run(self, motor_number, arr):
        if motor_number & 0x01: self.writeArray(self.MMX_SETPT_M1, arr)
        if motor_number & 0x02: self.writeArray(self.MMX_SETPT_M2, arr)
        if motor_number == self.MMX_Motor_Both: self.writeByte(self.MMX_COMMAND, 83)

    def runSecs(self, motor_number, secs, speed, brakeOnCompletion=False, waitForCompletion=False):
        ctrl = self.MMX_CONTROL_SPEED | self.MMX_CONTROL_TIME
        if brakeOnCompletion: ctrl |= self.MMX_CONTROL_BRK
        if motor_number != self.MMX_Motor_Both: ctrl |= self.MMX_CONTROL_GO
        arr = [speed, secs, 0, ctrl]
        if motor_number & 0x01: self.writeArray(self.MMX_SPEED_M1, arr)
        if motor_number & 0x02: self.writeArray(self.MMX_SPEED_M2, arr)
        if motor_number == self.MMX_Motor_Both: self.writeByte(self.MMX_COMMAND, 83)
        if waitForCompletion: time.sleep(0.05); self._waitTime(motor_number)

    def runDegs(self, motor_number, degs, speed, brakeOnCompletion=False, holdOnCompletion=False, waitForCompletion=False):
        ctrl = self.MMX_CONTROL_SPEED | self.MMX_CONTROL_TACHO | self.MMX_CONTROL_RELATIVE
        if brakeOnCompletion: ctrl |= self.MMX_CONTROL_BRK
        if holdOnCompletion:  ctrl |= self.MMX_CONTROL_BRK | self.MMX_CONTROL_ON
        if motor_number != self.MMX_Motor_Both: ctrl |= self.MMX_CONTROL_GO
        self._run(motor_number, self._ticks(degs) + [speed, 0, 0, ctrl])
        if waitForCompletion: time.sleep(0.05); self._waitTacho(motor_number)

    def runRotations(self, motor_number, rotations, speed, brakeOnCompletion=False, holdOnCompletion=False, waitForCompletion=False):
        self.runDegs(motor_number, rotations * 360, speed, brakeOnCompletion, holdOnCompletion, waitForCompletion)

    def runEncoderPos(self, motor_number, pos, speed, brakeOnCompletion=False, holdOnCompletion=False, waitForCompletion=False):
        ctrl = self.MMX_CONTROL_SPEED | self.MMX_CONTROL_TACHO
        if brakeOnCompletion: ctrl |= self.MMX_CONTROL_BRK
        if holdOnCompletion:  ctrl |= self.MMX_CONTROL_BRK | self.MMX_CONTROL_ON
        if motor_number != self.MMX_Motor_Both: ctrl |= self.MMX_CONTROL_GO
        self._run(motor_number, self._ticks(pos) + [speed, 0, 0, ctrl])
        if waitForCompletion: time.sleep(0.05); self._waitTacho(motor_number)

    def _isTimeDone(self, m):
        if m == self.MMX_Motor_1:    return (self.readByte(self.MMX_STATUS_M1) & 0x40) == 0
        if m == self.MMX_Motor_2:    return (self.readByte(self.MMX_STATUS_M2) & 0x40) == 0
        if m == self.MMX_Motor_Both:
            return (self.readByte(self.MMX_STATUS_M1) & 0x40) == 0 and \
                   (self.readByte(self.MMX_STATUS_M2) & 0x40) == 0
        return False

    def _isTachoDone(self, m):
        if m == self.MMX_Motor_1:    return (self.readByte(self.MMX_STATUS_M1) & 0x08) == 0
        if m == self.MMX_Motor_2:    return (self.readByte(self.MMX_STATUS_M2) & 0x08) == 0
        if m == self.MMX_Motor_Both:
            return (self.readByte(self.MMX_STATUS_M1) & 0x08) == 0 and \
                   (self.readByte(self.MMX_STATUS_M2) & 0x08) == 0
        return False

    def _waitTime(self, m):
        while not self._isTimeDone(m):  time.sleep(0.05)

    def _waitTacho(self, m):
        while not self._isTachoDone(m): time.sleep(0.05)


class NXTServo(mindsensors_i2c):

    NXTSERVO_ADDRESS      = 0xB0
    NXTSERVO_VBATT_SCALER = 37
    NXTSERVO_COMMAND      = 0x41
    NXTSERVO_VBATT        = 0x62

    def __init__(self, port, address=NXTSERVO_ADDRESS):
        port.activateCustomSensorI2C()
        mindsensors_i2c.__init__(self, address >> 1)
        self.command('S')

    def command(self, cmd):
        try: self.writeByte(self.NXTSERVO_COMMAND, int(cmd))
        except ValueError: self.writeByte(self.NXTSERVO_COMMAND, ord(cmd))

    def battVoltage(self):
        try: return self.readByte(self.NXTSERVO_VBATT) * self.NXTSERVO_VBATT_SCALER
        except: print("Error: Could not read battery voltage"); return ""

    def setSpeed(self, servoNumber, speed):    self.writeByte(0x52 + servoNumber - 1, speed % 256)
    def setPosition(self, servoNumber, position): self.writeByte(0x5A + servoNumber - 1, position % 256)

    def runServo(self, servoNumber, position, speed=None):
        if speed: self.setSpeed(servoNumber, speed)
        self.setPosition(servoNumber, position)

    def storeInitial(self, servoNumber): self.command('I'); self.command(servoNumber)
    def reset(self):                     self.command('S')
    def stopServo(self, servoNumber):    self.setPosition(servoNumber, 0)

    def setNeutral(self, servoNumber):
        self.command(73); time.sleep(0.1); self.command(servoNumber + 48)

    def haltMacro(self):   self.command('H')
    def resumeMacro(self): self.command('R')
    def editMacro(self):   self.command('E'); self.command('m')
    def pauseMacro(self):  self.command('P')
    def gotoEEPROM(self, position): self.command('G'); self.command(position)


class PFMate(mindsensors_i2c):

    PFMATE_ADDRESS  = (0x48)
    PFMATE_FLOAT    = 0
    PFMATE_FORWARD  = 1
    PFMATE_REVERSE  = 2
    PFMATE_BRAKE    = 3
    PFMATE_COMMAND  = 0x41
    PFMATE_CHANNEL  = 0x42
    PFMATE_MOTORS   = 0x43
    PFMATE_OPER_A   = 0x44
    PFMATE_SPEED_A  = 0x45
    PFMATE_OPER_B   = 0x46
    PFMATE_SPEED_B  = 0x47

    def __init__(self, port, address=PFMATE_ADDRESS):
        port.activateCustomSensorI2C()
        mindsensors_i2c.__init__(self, address >> 1)

    def command(self, cmd): self.writeByte(self.PFMATE_COMMAND, int(cmd))
    def _go(self): time.sleep(0.1); self.command(71); time.sleep(0.1)

    def controlBothMotors(self, channel, operationA, speedA, operationB, speedB):
        self.writeArray(self.PFMATE_CHANNEL, [channel, 0x00, operationA, speedA, operationB, speedB])
        self._go()

    def controlMotorA(self, channel, operationA, speedA):
        self.writeArray(self.PFMATE_CHANNEL, [channel, 0x01, operationA, speedA]); self._go()

    def controlMotorB(self, channel, operationB, speedB):
        self.writeArray(self.PFMATE_CHANNEL, [channel, 0x02])
        self.writeArray(self.PFMATE_OPER_B, [operationB, speedB]); self._go()


class EV3Lights(mindsensors_i2c):

    EV3Lights_ADDRESS = (0x2c)
    RED   = 0x42
    GREEN = 0x43
    BLUE  = 0x44

    def __init__(self, port, address=EV3Lights_ADDRESS):
        port.activateCustomSensorI2C()
        mindsensors_i2c.__init__(self, address >> 1)

    def setColor(self, Color, intensity):
        self.writeArray(Color, [intensity])
