import serial
import struct
import matplotlib.pyplot as plt
from array import array

class SCOPE:
    defaultPORT = '/dev/cu.usbmodem142203'
    defaultBAUD = 19200
    defaultMaxSamples = 200
    initCode = 0
    setVrefCode =1
    setSampRateCode = 2
    setNumSamplesCode = 3
    getDataCode =4
    errOnlyFormat = struct.Struct ('<bxx')

    def __init__(self, port, baud, channel):
        self.serPort = port
        self.serBaud = baud
        self.max_samples = SCOPE.defaultMaxSamples
        self.ser = serial.Serial(port, baud, timeout = 5)
        self.ser.reset_input_buffer()
        buffer = (SCOPE.initCode).to_bytes(1, byteorder='little', signed=False)\
                 + (0).to_bytes(1, byteorder='little', signed=False)
        self.ser.write (buffer)
        buffer = self.ser.read (3)
        err = (SCOPE.errOnlyFormat.unpack (buffer))[0]
        if (err):
            print ("Specified channel was out of range. Try again")
        else:
            self.channel = channel
            self.reference_voltage = 2.0
            self.samp_freq = 10000
            self.n_samples = 200
            self.samp_time = 0.02
            print ("SCOPE is ready")

    def setVref (self, vRef):
        err =0
        if vRef == "1V5":
            refCode = 0
            self.reference_voltage = 1.5
        elif vRef == "2V0":
            refCode = 1
            self.reference_voltage = 2.0
        elif vRef == "2V5":
            refCode = 2
            self.reference_voltage = 2.5
        elif vRef == "3V3":
            refCode = 3
            self.reference_voltage = 3.3
        else:
           print ("Specified Vref is not available. Choose one of 1V5, 2V0, 2V5, or 3V3")
           err =1
        if (err ==0):
            buffer = (SCOPE.setVrefCode).to_bytes(1, byteorder='little', signed=False)\
                     + (refCode).to_bytes(1, byteorder='little', signed=False)
            self.ser.write (buffer)
            buffer = self.ser.read (3)
            err = (SCOPE.errOnlyFormat.unpack (buffer))[0]
            if (err):
                print ("I thought that Vref was o.k., but msp430 says otherwise.")

    # we assume clock speed is 2^20 Hz (1048576). Sampling rate is detemined by timer CCR0 values
    # Sampling rate minimum when CCR0 = 65535 ticks =  2^20/65534 = 17Hz
    # Assume at least 10 ticks for an interrupt, so Sampling max =(2^20/10) = 104 kHz 
    def setSampFreq (self, sampFreq):
        ticks = round (1048576/sampFreq)
        if (ticks < 10):
            print ("Too FAST. Max sampling frequency is 104 kHz")
        elif (ticks > 65535):
            print ("Too SLOW. Min sampling frequency is 17 Hz")
        else:
            self.samp_freq = 1048576/ticks
            buffer = (SCOPE.setSampRateCode).to_bytes(1, byteorder='little', signed=False)\
                     + (0).to_bytes(1, byteorder='little', signed=False)\
                     + (ticks).to_bytes(2, byteorder='little', signed=False)
            self.ser.write (buffer)                        
            buffer = self.ser.read(3)
            err = (SCOPE.errOnlyFormat.unpack (buffer))[0]
            if (err):
                print ("I thought that frequency was o.k., but msp430 says otherwise.")
            else:
                self.samp_time = self.n_samples/self.samp_freq
                print ("Sampling frequency:", self.samp_freq, "Recording Time:", self.samp_time, "Samples:", self.n_samples)
                

    def setSampTime (self, sampTime):
        numSamples = round (sampTime * self.samp_freq)
        if (numSamples < 10):
            print ("number of samples must be greater than 10")
        elif (numSamples > self.max_samples):
            print ("number of samples must be less than ", self.max_samples)
        else:
            self.setNumSamples (numSamples)
            print ("Sampling frequency:", self.samp_freq, "Recording Time:", self.samp_time, "Samples:", self.n_samples)

    def setNumSamples (self, numSamples):
        if (numSamples < 10):
            print ("number of samples must be greater than 10")
        elif (numSamples > self.max_samples):
            print ("number of samples must be less than ", self.max_samples)
        else:
            self.n_samples = numSamples
            buffer = (SCOPE.setSampRateCode).to_bytes(1, byteorder='little', signed=False)\
                     + (0).to_bytes(1, byteorder='little', signed=False)\
                     + (numSamples).to_bytes(2, byteorder='little', signed=False)
            self.ser.write (buffer)
            err = (SCOPE.errOnlyFormat.unpack (self.ser.read(3)))[0]
            if (err):
                print ("I thought that sample number was o.k., but msp430 says otherwise.")
            else:
                self.samp_time = self.n_samples/self.samp_freq 

    def show (self):
        self.getData ()
        sampInt = 1/self.samp_freq
        time = array ('f', (i * sampInt for i in range (self.n_samples)))
        plt.plot (time, self.data)
        plt.xlabel('time (seconds)')
        plt.ylabel ('Voltage (Volts)')
        plt.show()

    def getData (self):
        self.ser.write ((SCOPE.getDataCode).to_bytes(1, byteorder='little', signed=False))
        buffer = self.ser.read(self.n_samples * 2)
        raw = array('H',buffer)
        self.data = array ('f', raw)
        for i in range (self.n_samples):
            self.data [i] = (self.data [i]/4095)* self.reference_voltage
        self.ser.reset_input_buffer()

        
    def live (self):
        sampInt = 1/self.samp_freq
        time = array ('f', (i * sampInt for i in range (self.n_samples)))
        self.getData()
        plt.ion()
        graph = plt.plot(time, self.data)[0]
        while (1):
            self.getData()
            graph.set_ydata(self.data)
            plt.draw()
            plt.pause(0.01)

            
    def stopLive (self):
        plt.ioff()
        plt.show()


if __name__ == '__main__':
    tex = SCOPE (SCOPE.defaultPORT, SCOPE.defaultBAUD, 0)
    
