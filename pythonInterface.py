import serial
from time import sleep
ser = serial.Serial('/dev/cu.usbmodem141203', 19200, timeout = 0)
serial_data = ser.read(60);print (serial_data)
ser.write (b'\r');sleep (.05);ser.write (b'portSetup 1 1 1\r')
serial_data = ser.read(60);print (serial_data)
ser.write (b'\r');sleep (.05);ser.write (b'portSetup 1 0 2\r')
serial_data = ser.read(60);print (serial_data)
for i in range (0,10):
    ser.write (b'\r');sleep (.05);ser.write (b'writeBits 1 2 1\r')
    ser.write (b'\r');sleep (.05);ser.write (b'readBits 1 2\r')
    serial_data = ser.read(60)
    print (serial_data)
    sleep (.5)

