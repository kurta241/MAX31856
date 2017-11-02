#!/usr/bin/python
#The MIT License (MIT)
#
#Copyright (c) 2017 Kurt Albrecht
#
#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:
#
#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.

'''
Class which defines interaction with the MAX31856 sensor.
library based on
  johnrbnsn/Adafruit_Python_MAX31856
  steve71/MAX31856
modified for pycom LoPy by Kurt Albrecht
'''

import math
from machine import SPI
from machine import Pin

# Thermocouple Types
MAX31856_B_TYPE = 0x0 # Read B Type Thermocouple
MAX31856_E_TYPE = 0x1 # Read E Type Thermocouple
MAX31856_J_TYPE = 0x2 # Read J Type Thermocouple
MAX31856_K_TYPE = 0x3 # Read K Type Thermocouple
MAX31856_N_TYPE = 0x4 # Read N Type Thermocouple
MAX31856_R_TYPE = 0x5 # Read R Type Thermocouple
MAX31856_S_TYPE = 0x6 # Read S Type Thermocouple
MAX31856_T_TYPE = 0x7 # Read T Type Thermocouple

class MAX31856(object):
    """Class to represent an Adafruit MAX31856 thermocouple temperature
    measurement board.
    """

    # Board Specific Constants
    MAX31856_CONST_THERM_LSB = 2**-7
    MAX31856_CONST_THERM_BITS = 19
    MAX31856_CONST_CJ_LSB = 2**-6
    MAX31856_CONST_CJ_BITS = 14

    ### Register constants, see data sheet Table 6 (in Rev. 0) for info.
    # Read Addresses
    MAX31856_REG_READ_CR0 = 0x00
    MAX31856_REG_READ_CR1 = 0x01
    MAX31856_REG_READ_MASK = 0x02
    MAX31856_REG_READ_CJHF = 0x03
    MAX31856_REG_READ_CJLF = 0x04
    MAX31856_REG_READ_LTHFTH = 0x05
    MAX31856_REG_READ_LTHFTL = 0x06
    MAX31856_REG_READ_LTLFTH = 0x07
    MAX31856_REG_READ_LTLFTL = 0x08
    MAX31856_REG_READ_CJTO = 0x09
    MAX31856_REG_READ_CJTH = 0x0A  # Cold-Junction Temperature Register, MSB
    MAX31856_REG_READ_CJTL = 0x0B  # Cold-Junction Temperature Register, LSB
    MAX31856_REG_READ_LTCBH = 0x0C # Linearized TC Temperature, Byte 2
    MAX31856_REG_READ_LTCBM = 0x0D # Linearized TC Temperature, Byte 1
    MAX31856_REG_READ_LTCBL = 0x0E # Linearized TC Temperature, Byte 0
    MAX31856_REG_READ_FAULT = 0x0F # Fault status register

    # Write Addresses
    MAX31856_REG_WRITE_CR0 = 0x80
    MAX31856_REG_WRITE_CR1 = 0x81
    MAX31856_REG_WRITE_MASK = 0x82
    MAX31856_REG_WRITE_CJHF = 0x83
    MAX31856_REG_WRITE_CJLF = 0x84
    MAX31856_REG_WRITE_LTHFTH = 0x85
    MAX31856_REG_WRITE_LTHFTL = 0x86
    MAX31856_REG_WRITE_LTLFTH = 0x87
    MAX31856_REG_WRITE_LTLFTL = 0x88
    MAX31856_REG_WRITE_CJTO = 0x89
    MAX31856_REG_WRITE_CJTH = 0x8A  # Cold-Junction Temperature Register, MSB
    MAX31856_REG_WRITE_CJTL = 0x8B  # Cold-Junction Temperature Register, LSB

    # Pre-config Register Options
    MAX31856_CR0_READ_ONE = 0x40 # One shot reading, delay approx. 200ms then read temp registers
    MAX31856_CR0_READ_CONT = 0x80 # Continuous reading, delay approx. 100ms between readings
    MAX31856_CR0_REJECT_50Hz = 0x01 # Noise rejection filter selection

    def __init__(self, tc_type=MAX31856_K_TYPE, avgsel=0x0, cs_pin='P9'):
        """Initialize MAX31856 device with hardware SPI.

        Args:
            tc_type (1-byte Hex): Type of Thermocouple.  Choose from class variables of the form
                MAX31856.MAX31856_K_TYPE.
            avgsel (1-byte Hex): Type of Averaging.  Choose from values in CR0 table of datasheet.
                Default is single sample.
            cs_pin: chip select Pin. Default P9
        """

        # initialize cs_pin in gpio mode and make it an CS output
        self.CS = Pin(cs_pin, mode=Pin.OUT)
        self.CS(True)  # init chip select
        self.spi = SPI(0, mode=SPI.MASTER, baudrate=500000, polarity=0, phase=1, firstbit=SPI.MSB)

        # Initialize control register 1
        self.tc_type = tc_type
        self.avgsel = avgsel
        self.cr1 = ((self.avgsel << 4) + self.tc_type)

        # Setup for reading continuously with K-Type thermocouple und 50Hz noise rejection
        self._write_register(self.MAX31856_REG_WRITE_CR0, self.MAX31856_CR0_READ_CONT+self.MAX31856_CR0_REJECT_50Hz)
        self._write_register(self.MAX31856_REG_WRITE_CR1, self.cr1)

    @staticmethod
    def _cj_temp_from_bytes(msb, lsb):
        # Takes in the msb and lsb from a Cold Junction (CJ) temperature reading and
        # converts it into a decimal value.
        #    msb (hex): Most significant byte of CJ temperature
        #    lsb (hex): Least significant byte of a CJ temperature

        #            (((msb w/o +/-) shifted by number of 1 byte above lsb)
        #                                  + val_low_byte)
        #                                          >> shifted back by # of dead bits
        temp_bytes = (((msb & 0x7F) << 8) + lsb) >> 2

        if msb & 0x80:
            # Negative Value.  Scale back by number of bits
            temp_bytes -= 2**(MAX31856.MAX31856_CONST_CJ_BITS -1)

        #        temp_bytes*value of lsb
        temp_c = temp_bytes*MAX31856.MAX31856_CONST_CJ_LSB

        return temp_c

    @staticmethod
    def _thermocouple_temp_from_bytes(byte0, byte1, byte2):
        # Converts the thermocouple byte values to a decimal value.
        #    byte2 (hex): Most significant byte of thermocouple temperature
        #    byte1 (hex): Middle byte of thermocouple temperature
        #    byte0 (hex): Least significant byte of a thermocouple temperature
        # temp_c (float): Temperature in degrees celsius
        #
        #            (((val_high_byte w/o +/-) shifted by 2 bytes above LSB)
        #                 + (val_mid_byte shifted by number 1 byte above LSB)
        #                                             + val_low_byte )
        #                              >> back shift by number of dead bits
        temp_bytes = (((byte2 & 0x7F) << 16) + (byte1 << 8) + byte0)
        temp_bytes = temp_bytes >> 5

        if byte2 & 0x80:
            temp_bytes -= 2**(MAX31856.MAX31856_CONST_THERM_BITS -1)

        #        temp_bytes*value of LSB
        temp_c = temp_bytes*MAX31856.MAX31856_CONST_THERM_LSB

        return temp_c

    def read_internal_temp_c(self):
        # Return internal temperature value in degrees celsius.
        # Read as a multibyte transfer to ensure both bytes are from the
        # same temperature update.

        self.CS(False)
        self.spi.write(bytes([self.MAX31856_REG_READ_CJTH])) # first read address
        val_high_byte = self.spi.read(1)[0]
        val_low_byte = self.spi.read(1)[0]
        self.CS(True)

        temp_c = MAX31856._cj_temp_from_bytes(val_high_byte, val_low_byte)
        return temp_c

    def read_temp_c(self):
        # Return the thermocouple temperature value in degrees celsius.
        # Read as a multibyte transfer to ensure all three bytes are from the
        # same temperature update.

        self.CS(False)
        self.spi.write(bytes([self.MAX31856_REG_READ_LTCBH])) # first read address
        val_high_byte = self.spi.read(1)[0]
        val_mid_byte = self.spi.read(1)[0]
        val_low_byte = self.spi.read(1)[0]
        fault = self.spi.read(1)[0]
        self.CS(True)
        # check fault byte
        if ((fault & 0x80) != 0):
            raise MAX31856Error("Cold Junction Out-of-Range")
        if ((fault & 0x40) != 0):
            raise MAX31856Error("Thermocouple Out-of-Range")
        if ((fault & 0x20) != 0):
            raise MAX31856Error("Cold-Junction High Fault")
        if ((fault & 0x10) != 0):
            raise MAX31856Error("Cold-Junction Low Fault")
        if ((fault & 0x08) != 0):
            raise MAX31856Error("Thermocouple Temperature High Fault")
        if ((fault & 0x04) != 0):
            raise MAX31856Error("Thermocouple Temperature Low Fault")
        if ((fault & 0x02) != 0):
            raise MAX31856Error("Overvoltage or Undervoltage Input Fault")
        if ((fault & 0x01) != 0):
            raise MAX31856Error("Thermocouple Open-Circuit Fault")

        temp_c = MAX31856._thermocouple_temp_from_bytes(val_low_byte, val_mid_byte, val_high_byte)
        return temp_c

    def read_fault_register(self):
        # Return bytes containing fault codes and hardware problems.
        reg = self._read_register(self.MAX31856_REG_READ_FAULT)
        return reg

    def _read_register(self, address):
        # Reads a register at address from the MAX31856
        # Args: address (8-bit Hex): Address for read register.

        self.CS(False)
        self.spi.write(bytes([address]))
        value=self.spi.read(1)[0]
        self.CS(True)
        return value

    def _write_register(self, address, write_value):
        # Writes to a register at address from the MAX31856
        # address (8-bit Hex): Address for read register.
        # write_value (8-bit Hex): Value to write to the register

        self.CS(False)
        self.spi.write(bytes([address, write_value]))
        self.CS(True)
        # print('Wrote Register: 0x{0:02X}, Value 0x{1:02X}'.format((address & 0xFF), (write_value & 0xFF)))
        return True

class MAX31856Error(Exception):
    # Constructor or Initializer
    def __init__(self, msg):
        super(MAX31856Error, self).__init__(msg)
