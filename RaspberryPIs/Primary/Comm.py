from smbus import SMBus
import serial
import struct
import os

magaddr = 0x1c
gyaddr = 0x6a
praddr = 0x77
sensors = SMBus(1)

speed = 10 #in Hz
increment = 1.0 / speed
name = '/dev/ttyUSB'
ser = serial.Serial(write_timeout = increment/4.0, timeout = increment/4.0)
arduinoInit = False

def initArduino():
    name = '/dev/ttyUSB'
    for i in range(4):
        if os.path.exists(name + str(i)):
            name = name + str(i)
            break
        if i == 3:
            print('ERROR: Arudino might not be plugged in')
            return False
    ser.port = name
    ser.baudrate = 115200
    ser.open()
    ser.reset_input_buffer()
    for i in range(50):
        ser.readline()
    if not ser.is_open:
        print('ERROR: Arduino port not open')
        return False
    arduinoInit = True
    return True

def initSensors():
    #mag
    sensors.write_byte_data(magaddr, 0x20, 0b11001100)
    sensors.write_byte_data(magaddr, 0x22, 0b00000000)
    sensors.write_byte_data(magaddr, 0x23, 0b00001010)
    #gyroscope
    sensors.write_byte_data(gyaddr, 0x22, 0b00000110)
    sensors.write_byte_data(gyaddr, 0x11, 0b00000010)
    sensors.write_byte_data(gyaddr, 0x12, 0b01000000)
    sensors.write_byte_data(gyaddr, 0x10, 0b01011000)
    #accel
    sensors.write_byte_data(gyaddr, 0x21, 0b11000100) #check
    sensors.write_byte_data(gyaddr, 0x20, 0b01001000) #check
    #pressure
    sensors.write_byte_data(praddr, 0xf5, 0b00001000)
    sensors.write_byte_data(praddr, 0xf4, 0b01010011)

def stopSensors():
    sensors.write_byte_data(magaddr, 0x22, 0b00000011)
    sensors.write_byte_data(gyaddr, 0x10, 0)
    sensors.write_byte_data(gyaddr, 0x20, 0)
    sensors.write_byte_data(praddr, 0xf4, 0b01010100)

def getData():
    data = []
    data.extend(sensors.read_i2c_block_data(magaddr, 0x28, 6))
    data.extend(sensors.read_i2c_block_data(gyaddr, 0x18, 6))
    data.extend(sensors.read_i2c_block_data(gyaddr, 0x28, 6))
    data.extend(sensors.read_i2c_block_data(praddr, 0xf7, 3))
    toReturn = []
    for i in range(0, len(data) - 3, 2):
        num = (data[i] << 8) + data[i+1]
        if num > 32767:
            num = num - 65536
        toReturn.append(num)
    index = len(data) - 3
    pres = (data[index] << 12) + (data[index+1] << 4) + (data[index+2] >> 4)
    toReturn.append((pres * .33))
    return toReturn

def getPitot():
    ser.reset_input_buffer()
    raw = ser.read(2)
    return (raw[0] << 8) | raw[1]

def sendArduino(data):
    if not arduinoInit:
        return
    for i in range(len(data)):
        ba = bytearray(struct.pack('f', data[i]))
#        print(ba)
        ser.write(ba)
#    print(ser.readline())
