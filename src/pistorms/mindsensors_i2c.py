#!/usr/bin/env python3
#
# Copyright (c) 2015 mindsensors.com
# 
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#
#mindsensors.com invests time and resources providing this open source code, 
#please support mindsensors.com  by purchasing products from mindsensors.com!
#Learn more product option visit us @  http://www.mindsensors.com/
#
# History:
# Date            Author            Comments
# 01/30/14    Deepak            Initial authoring.
# 02/03/14    Michael           Error message, signed byte, and integer fix
# 03/09/14    Nitin             Changed the 16 and 32 bit read

## @package mindsensors_i2c
# This is the i2c module for mindsensors i2c devices.

import smbus
import ctypes
import threading
import time

# Transient bus errors (e.g. Errno 121 Remote I/O) are common on the PiStorms
# bus while motors run. Every transaction is retried this many times before
# the error is raised to the caller — never silently swallowed, because a
# swallowed read used to surface as None (→ fake 0 V battery, dead odometry
# thread) and a swallowed write could drop a brake command.
I2C_RETRIES     = 3
I2C_RETRY_DELAY = 0.002   # s
SMBUS_BLOCK_MAX = 32      # smbus block transfers are limited to 32 bytes
# Every Raspberry Pi since the 1 B rev 2 exposes the GPIO header I2C on bus 1
# (bus 0 only existed on the very first boards).
I2C_BUS = 1

def _with_retries(fn, *args):
    for attempt in range(I2C_RETRIES):
        try:
            return fn(*args)
        except OSError:
            if attempt == I2C_RETRIES - 1:
                raise
            time.sleep(I2C_RETRY_DELAY)

## mindsensors_i2c: this class provides i2c functions
#  for read and write operations.
class mindsensors_i2c(object):

    ## Initialize the class with the i2c address of your device
    #  @param self The object pointer.
    #  @param i2c_address Address of your device.
    def __init__(self, i2c_address):
        self.address = i2c_address
        self.bus = smbus.SMBus(I2C_BUS)
        # None = not probed yet; True/False = device does/doesn't return the
        # same bytes for a block read as for individual byte reads.
        self._block_read_ok = None
        # One lock per device. A single transaction is already atomic in the
        # kernel, but a SEQUENCE of transactions is not: a byte-by-byte read,
        # or a motor command made of several writes (speed M1, speed M2, 'S'),
        # must not be interleaved with another thread's command — otherwise a
        # brake sent from one thread can be undone by the 'S' of a ramp step
        # running in another. Reentrant, so a locked method may call another.
        self.lock = threading.RLock()
        
    ## Prints an error message if a read error is detected
    #  @param self The object pointer.
    def errMsg(self):
        print ("Error accessing 0x%02X: Check your I2C address" % self.address)
        return -1

    ## Write a byte to your i2c device at a given location
    #  @param self The object pointer.
    #  @param reg The register to write value at.
    #  @param value Value to write.
    #  @remark Retries transient bus errors; raises OSError if all retries fail.
    def writeByte(self, reg, value):
        _with_retries(self.bus.write_byte_data, self.address, reg, value)

    ## Read an unsigned byte from your i2c device at a given location
    #  @param self The object pointer.
    #  @param reg The register to read from.
    #  @remark Retries transient bus errors; raises OSError if all retries fail.
    def readByte(self, reg):
        return _with_retries(self.bus.read_byte_data, self.address, reg)
            
    ## Read a signed byte from your i2c device at a given location
    #  @param self The object pointer.
    #  @param reg The register to read from.
    def readByteSigned(self, reg):
        a = self.readByte(reg)
        signed_a = ctypes.c_byte(a).value
        return signed_a
     
    # for read_i2c_block_data and write_i2c_block_data to work correctly,
    # ensure that i2c speed is set correctly on your pi:
    # ensure following file with contents as follows:
    #    /etc/modprobe.d/i2c.conf
    # options i2c_bcm2708 baudrate=50000
    # (without the first # and space on line above)
    #
    
    ## Read a byte array from your i2c device starting at a given location
    #  @param self The object pointer.
    #  @param reg The first register in the array to read from.
    #  @param length The length of the array.
    #  @remark Multi-byte values (encoder counts, gyro rates) must be read in
    #  one transaction: byte-by-byte, the value can change between bytes and a
    #  carry across a byte boundary gives a ±256-count glitch. A block read is
    #  atomic; if the device doesn't support it, two identical byte-by-byte
    #  passes are required instead. Raises OSError if the bus keeps failing.
    def readArray(self, reg, length):
        with self.lock:
            if self._block_read_ok is None:
                self._probe_block_read()
            if not self._block_read_ok:
                return self._readArrayConsistent(reg, length)
            result = []
            while length > 0:
                chunk_length = min(length, SMBUS_BLOCK_MAX)
                result += _with_retries(self.bus.read_i2c_block_data, self.address, reg, chunk_length)
                reg += chunk_length
                length -= chunk_length
            return result

    def _readArrayBytes(self, reg, length):
        return [self.readByte(reg + i) for i in range(length)]

    def _readArrayConsistent(self, reg, length):
        previous = self._readArrayBytes(reg, length)
        if length == 1:
            return previous
        for attempt in range(I2C_RETRIES):
            current = self._readArrayBytes(reg, length)
            if current == previous:
                return current
            previous = current
        # The value kept changing between bytes: returning it could give a
        # ±256 glitch (e.g. on an encoder), so report it like a bus error.
        raise OSError(f"I2C 0x{self.address:02X}: reg 0x{reg:02X} changed during "
                      f"{I2C_RETRIES + 1} byte-by-byte reads")

    def _probe_block_read(self):
        """Compare a block read of the (static) version string at 0x00 with
        byte reads; use block reads only if the device returns the same data."""
        try:
            blk = list(self.bus.read_i2c_block_data(self.address, 0x00, 8))
            self._block_read_ok = (blk == self._readArrayBytes(0x00, 8))
            if not self._block_read_ok:
                print("I2C 0x%02X: block read mismatch — using byte reads" % self.address)
        except OSError:
            pass   # bus glitch during the probe: stay undecided, retry next call

    ## Write a byte array from your i2c device starting at a given location
    #  @param self The object pointer.
    #  @param reg The first register in the array to write to.
    #  @param arr The array to write.
    #  @remark Retries transient bus errors; raises OSError if all retries fail.
    def writeArray(self, reg, arr):
        _with_retries(self.bus.write_i2c_block_data, self.address, reg, arr)

    ## Read a string from your i2c device starting at a given location
    #  @param self The object pointer.
    #  @param reg The first register of the string to read from.
    #  @param length The length of the string.
    def readString(self, reg, length):
        with self.lock:
            characters = [chr(self.readByte(reg + i)) for i in range(length)]
        # Shorter strings are padded with NUL bytes ('V1.20\x00\x00\x00')
        return ''.join(characters).rstrip('\x00')

    ## Read an unsigned 16 bit integer from your i2c device from a given location.  Big-endian read integers .
    #  @param self The object pointer.
    #  @param reg The first register of the first byte of the integer to read.
    def readIntegerBE(self, reg):        
        results = self.readArray( reg, 2)
        return results[1] + (results[0]<<8)
        
    ## Read an unsigned 16 bit integer from your i2c device from a given location. little endian read integers.
    #  @param self The object pointer.
    #  @param reg The first register of the first byte of the integer to read.
    def readInteger(self, reg):        
        results = self.readArray(reg, 2)
        return results[0] + (results[1]<<8)        

    ## Write an unsigned 16 bit integer from your i2c device from a given location. little endian write integers.
    #  @param self The object pointer.
    #  @param reg The first register of the first byte of the integer to write.
    #  @param int The integer to write.
    def writeInteger(self, reg, i):        
        i = int(i)
        self.writeArray(reg, [i % 256, (i >> 8) % 256])

    ## Read a signed 16 bit integer from your i2c device from a given location. Big endian read integers .
    #  @param self The object pointer.
    #  @param reg The first register of the first byte of the integer to read.
    def readIntegerSignedBE(self, reg):
        a = self.readIntegerBE(reg)
        signed_a = ctypes.c_short(a).value
        return signed_a        
    
    ## Read a signed 16 bit integer from your i2c device from a given location. little endian read integers .
    #  @param self The object pointer.
    #  @param reg The first register of the first byte of the integer to read.
    def readIntegerSigned(self, reg):
        a = self.readInteger(reg)
        signed_a = ctypes.c_short(a).value
        return signed_a

    ## Read an unsigned 32bit integer from your i2c device from a given location. Big endian read integers.
    #  @param self The object pointer.
    #  @param reg The first register of the first byte of the integer to read.
    def readLongBE(self, reg):
        
        results = self.readArray(reg,4)
        return results[3] + (results[2]<<8)+(results[1]<<16)+(results[0]<<24)           
        
    ## Read an unsigned 32bit integer from your i2c device from a given location. little endian read integers.
    #  @param self The object pointer.
    #  @param reg The first register of the first byte of the integer to read.
    def readLong(self, reg):
        results = self.readArray(reg,4)
        return results[0] + (results[1]<<8)+(results[2]<<16)+(results[3]<<24)       

    ## Read a signed 32bit integer from your i2c device from a given location. Big endian read integers .
    #  @param self The object pointer.
    #  @param reg The first register of the first byte of the integer to read.
    def readLongSignedBE(self, reg):
        a = self.readLongBE(reg)
        signed_a = ctypes.c_int32(a).value   # c_long is 64-bit on aarch64
        return signed_a

    ## Read a signed 32bit integer from your i2c device from a given location. little endian read integers .
    #  @param self The object pointer.
    #  @param reg The first register of the first byte of the integer to read.
    def readLongSigned(self, reg):
        a = self.readLong(reg)
        signed_a = ctypes.c_int32(a).value   # c_long is 64-bit on aarch64
        return signed_a

    ##  Read the firmware version of the i2c device
    #  @param self The object pointer.
    def GetFirmwareVersion(self):
        try:
            ver = self.readString(0x00, 8)
            return ver
        except OSError:
            print ("Error: Could not retrieve Firmware Version" )
            print ("Check I2C address and device connection to resolve issue")
            return ""

    ##  Read the vendor name of the i2c device
    #  @param self The object pointer.
    def GetVendorName(self):
        try:
            vendor = self.readString(0x08, 8)
            return vendor
        except OSError:
            print("Error: Could not retrieve Vendor Name")
            print ("Check I2C address and device connection to resolve issue")
            return ""
            
    ##  Read the i2c device id
    #  @param self The object pointer.
    def GetDeviceId(self):
        try:    
            device = self.readString(0x10, 8)
            return device
        except OSError:
            print ("Error: Could not retrieve Device ID")
            print ("Check I2C address and device connection to resolve issue")
            return ""




