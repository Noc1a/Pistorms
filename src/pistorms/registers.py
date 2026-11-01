"""PiStorms register map (PiStorms Advanced Development Guide v1.0).

Constants only. Each bank (A, B) is a separate I2C device with this same map.
Multi-byte values are little-endian (least significant byte first).
"""

# I2C addresses of the two banks (8-bit form, as in the guide)
PS_A_ADDRESS = 0x34
PS_B_ADDRESS = 0x36

# ── Command register: write one of the command letters below ────────────────
PS_Command = 0x41

# ── Motors ─────────────────────────────────────────────────────────────────
# Write block of motor 1 (0x42–0x49) and motor 2 (0x4A–0x51)
PS_SetPoint_M1 = 0x42    # encoder target, 4 bytes
PS_Speed_M1    = 0x46    # speed -100…100
PS_Time_M1     = 0x47    # seconds
PS_CMDB_M1     = 0x48    # command register B, reserved: write 0
PS_CMDA_M1     = 0x49    # command register A: PS_CONTROL_… bits

PS_SetPoint_M2 = 0x4A
PS_Speed_M2    = 0x4E
PS_Time_M2     = 0x4F
PS_CMDB_M2     = 0x50
PS_CMDA_M2     = 0x51

PS_Position_M1 = 0x52    # encoder position, signed 4 bytes
PS_Position_M2 = 0x56
PS_Status_M1   = 0x5A    # status bits, see PiStormsMotor.STATUS_…
PS_Status_M2   = 0x5B
PS_Tasks_M1    = 0x5C
PS_Tasks_M2    = 0x5D

# Bits of command register A
PS_CONTROL_SPEED    = 0x01   # honour the speed register
PS_CONTROL_RAMP     = 0x02   # the board ramps the power
PS_CONTROL_RELATIVE = 0x04   # encoder target relative to the last position
PS_CONTROL_TACHO    = 0x08   # move to the encoder target
PS_CONTROL_BRK      = 0x10   # brake (1) or float (0) at the end
PS_CONTROL_ON       = 0x20   # hold the encoder position at the end
PS_CONTROL_TIME     = 0x40   # run for the time register
PS_CONTROL_GO       = 0x80   # apply the bits above now

# PID of the bank (2-byte ints); back to the defaults at power cycle
PS_P_Kp = 0x5E           # encoder position control
PS_P_Ki = 0x60
PS_P_Kd = 0x62
PS_S_Kp = 0x64           # speed control
PS_S_Ki = 0x66
PS_S_Kd = 0x68
PS_PassCount     = 0x6A
PS_PassTolerance = 0x6B

PS_ChkSum = 0x6C
PS_BattV  = 0x6E         # battery voltage: value × 0.04 V

# ── Sensor ports (port 2 registers = port 1 registers + 0x34) ───────────────
PS_S1_Mode       = 0x6F  # port type, see SensorPort.PS_SENSOR_TYPE_…
PS_S1EV_Ready    = 0x70
PS_S1EV_SensorID = 0x71  # EV3 sensor name, 16 bytes
PS_S1EV_Mode     = 0x81  # EV3 sensor mode
PS_S1EV_Length   = 0x82
PS_S1EV_Data     = 0x83  # EV3 sensor data, 32 bytes
PS_S1AN_Read     = 0x70  # NXT analog value, 2 bytes

PS_S2_Mode       = 0xA3
PS_S2EV_Ready    = 0xA4
PS_S2EV_SensorID = 0xA5
PS_S2EV_Mode     = 0xB5
PS_S2EV_Length   = 0xB6
PS_S2EV_Data     = 0xB7
PS_S2AN_Read     = 0xA4

# ── LED, buttons, touchscreen ──────────────────────────────────────────────
PS_R = 0xD7              # LED red, green, blue (either bank)
PS_G = 0xD8
PS_B = 0xD9
PS_KeyPress  = 0xDA      # bit 0 = GO button pressed now
PS_Key1Count = 0xDB      # GO button press count
PS_Key2Count = 0xDC
PS_TSX = 0xE3            # touchscreen raw X (2 bytes, bank A)
PS_TSY = 0xE5            # touchscreen raw Y

# ── Command letters (write to PS_Command) ───────────────────────────────────
R = 0x52   # reset all encoders and motor parameters (not the PID)
S = 0x53   # start both motors at once (sets GO on both)
a = 0x61   # float motor 1
b = 0x62   # float motor 2
c = 0x63   # float both motors
A = 0x41   # brake motor 1
B = 0x42   # brake motor 2
C = 0x43   # brake both motors
r = 0x72   # reset encoder of motor 1
s = 0x73   # reset encoder of motor 2
H = 0x48   # start the board power-cut countdown (not in the guide)
