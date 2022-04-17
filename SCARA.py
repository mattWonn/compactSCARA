#! /usr/bin/python
#-*-coding: utf-8 -*
from abc import ABCMeta, abstractmethod
import os
import inspect

import serial
import serial.tools
import struct

class SCARA:
    # SCARA state 
    # format = error, motor1 PWM, motor2 PWM, toolData, encoder 1 counts, encoder 2 counts, z axis counts,  
    # size = 4 + 3*2 = 10
    dataFormat = struct.Struct('<BhhhBBB')
    posFormat = struct.Struct ('<bhh')    # just the arm positions
    mtrsFormat = struct.Struct ('<bxx')
    defaultPORT = '/dev/cu.usbmodem142203'
    defaultBAUD = 115200
    getStateCode = 0        # msp430 must recognize these codes,
    getPosCode = 1          # do the needful, and send back results
    setMtrCode =2
    EmStopCode = 3
    movJCode = 4
    noErrCode =  0          # first byte of results msp430 sends must be one of these error codes
    EmStoppedCode = 1       #

    def __init__(self, port, baud):
        self.serPort = port
        self.serBaud = baud
        self.ser = serial.Serial(port, baud, timeout = 10)
        buffer = self.ser.read(24)
        print(buffer)
        self.getState ()

    def getPos (self):
        self.ser.write ((SCARA.getPosCode).to_bytes(1, byteorder='little', signed=False))
        buffer=self.ser.read (5)
        print (buffer)
        posData = SCARA.posFormat.unpack (buffer)
        self.state [0] = posData [0]
        if (posData [0] == SCARA.noErrCode):
            self.state [4] = posData[1]
            self.state [5] = posData[2]
        else:
           print("Error: ", self.state [0])
           
    # 11 [0] numBytes [1]errCode, [2,3]encoder 1 counts, [4,5]encoder 2 counts, [6,7]z axis counts,
    # [8] tool Data, [9]motor1 PWM, [10]motor2 PWM

    def getState (self):
        self.ser.write ((SCARA.getStateCode).to_bytes(1, byteorder='little', signed=False))
        buffer = self.ser.read (10)
        print (buffer)
        self.state = list (SCARA.dataFormat.unpack (buffer))
        if (self.state [0] == SCARA.noErrCode):
            print (self.state)
        else:
            print ("Error: ", self.state [0])

    def setMtrs (self, M1val, M2val, doConfirm=0):
        buffer = (SCARA.setMtrCode).to_bytes(1, byteorder='little', signed=False)\
                 + (M1val).to_bytes(1, byteorder='little', signed=False)\
                 + (M2val).to_bytes(1, byteorder='little', signed=False)
        self.ser.write (buffer)
        if doConfirm:
            buffer = self.ser.read (3)
            print (buffer)
            self.state [0] = SCARA.mtrsFormat.unpack (buffer)[0]
            if (self.state [0] != SCARA.noErrCode):
                print ("Error: ", self.state [0])

    #does an emergency stop and sends back SCARA state, with error code set to ESTOP       
    def EmStop (self, doStop =1):
        self.ser.write ((SCARA.EmStopCode).to_bytes (1, byteorder = 'little', signed = False))
        errVal = int.from_bytes( self.ser.read (1), byteorder='little', signed=False)
        if (errVal != SCARA.EmStoppedCode):
            print ("SCARA was not stopped.")
       

    #does a joint interpolated movement from current joint posiitons to new joint positions
    def movJ (self, enc1, enc2, doConfirm=0):
        buffer = (SCARA.movJCode).to_bytes (1, byteorder = 'little', signed = False)\
                 + enc1.to_bytes (2, byteorder = 'little', signed = True)\
                 + enc2.to_bytes (2, byteorder = 'little', signed = True)
        self.ser.write ((SCARA.movJCode).to_bytes (1, byteorder = 'little', signed = False))
        errVal = int.from_bytes( self.ser.read (1), byteorder='little', signed=False)
        if (errVal != SCARA.EmStoppedCode):
            print ("SCARA was not stopped.")

    #def startLiveUpdate (self, doEnc, doMtr, doZ):
        

if __name__ == '__main__':
    clem = SCARA (SCARA.defaultPORT, SCARA.defaultBAUD) 
    #Clem.getPos()
    #print (clem.state)
    

    
"""
for i in range (0,160):
    serial_data = ser.read(2)#;print (serial_data)
    decVal = int.from_bytes(serial_data, byteorder='little', signed=True)
    print (decVal)









int.to_bytes(length, byteorder, *, signed=False)
#Return an array of bytes representing an integer.

-1024).to_bytes(10, byteorder='little', signed=True)
b'\x00\xfc\xff\xff\xff\xff\xff\xff\xff\xff'


import struct
...

try:
    ieee754_data = my_serial.read(4)
    my_float = struct.unpack('f', ieee754_data)
catch:
    # I/O Error, or junk data
    my_float = 0.0
And packing:

 ieee754_data = struct.pack('f', my_float)
 try:
     my_serial.write(ieee754_data)
 catch:
     pass #TODO - I/O Error
"""
