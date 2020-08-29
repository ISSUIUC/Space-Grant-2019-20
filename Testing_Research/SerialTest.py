import serial
import time
import struct
import os
import sys

#I recommend running this inside the python console. Just import this file and it runs once right away
#you can just type run multiple times and change data lists if you want
#However, this is not required, run it however you want, as long as the arduino is plugged in, its good

#data lists, every number needs to be a float
accelz = [1.1,2.2,3.3,4.4,5.5,6.6,7.7,8.8]
roll = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]
altitude = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]
velocity = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]

#speed in Hz
speed = 10

increment = 1.0 / speed
name = '/dev/ttyUSB'
for i in range(4):
    if os.path.exists(name + str(i)):
        name = name + str(i)
        break
    if i == 3:
        print('ERROR: Arudino might not be plugged in')
        sys.exit()
ser = serial.Serial(name, write_timeout = increment/4.0, timeout = increment/4.0)
ser.baudrate = 115200

def run():
    next = time.time() + increment
    length = len(accelz)
    for i in range(length):
        #uncomment the next line if arduino is sending pitot tube data, to keep code synced ---------
        #print(ser.read(2))
        ba = bytearray(struct.pack('f', accelz[i]))
        ser.write(ba)
        ba = bytearray(struct.pack('f', roll[i]))
        ser.write(ba)
        ba = bytearray(struct.pack('f', altitude[i]))
        ser.write(ba)
        ba = bytearray(struct.pack('f', velocity[i]))
        ser.write(ba)
        while time.time() < next:
            time.sleep(next - time.time())
        next = next + increment

run()
