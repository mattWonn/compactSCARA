
import serial

ser = serial.Serial('/dev/cu.usbmodem141203', 19200, timeout = 5)
for i in range (0,160):
    serial_data = ser.read(2)#;print (serial_data)
    decVal = int.from_bytes(serial_data, byteorder='little', signed=True)
    print (decVal)



"""
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
