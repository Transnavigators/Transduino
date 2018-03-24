# test_transduino
# 
# Script to test if motor control system is working correctly
# Users can enter desired speeds for both motors and receive the counts from the encoders
#
# @author Transnavigators
* 

import smbus
import time
import struct

# initialize the bus
bus = smbus.SMBus(1)

# Arduino i2c address
address = 0x04

# commands for sending and receiving it
move_cmd = ord('m')
encoder_cmd = ord('e')

# send data to arduino
def sendSpeedToMotor(m1,m2):
    val = list(bytearray(struct.pack("=ff", m1, m2)))
    bus.write_i2c_block_data(address, move_cmd, list(val))
    
# get data from arduino
def readEncoders():
    data = bus.read_i2c_block_data(address, encoder_cmd)
    return data
    
    
while True:
    m1 = input("Enter the desired speed for motor 1 (m/s) : ")
    m2 = input("Enter the desired speed for motor 2 (m/s) : ")

    sendSpeedToMotor(float(m1),float(m2))
	
    print("Motor 1: ", m1, "Motor 2: ", m2)
	
    # sleep one second
    time.sleep(1)

    receive_data = readEncoders()
    e1,e2 = struct.unpack('ii',bytearray(receive_data[0:8]))
    
    print("Encoder Data: ", e1, e2)
