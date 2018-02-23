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
    bus.write_i2c_block_data(address, move_cmd, [m1, m2]);
    
# get data from arduino
def readEncoders():
    data = bus.read_i2c_block_data(address, encoder_cmd)
    return data
    
# Covert int to byte
def convertToByte(byte):
    if byte > 127:
        return (256-byte) * (-1)
    else:
        return byte
	
while True:
    m1 = input("Enter value for motor 1 [-127,127] : ")
    m2 = input("Enter value for motor 2 [-127,127] : ")

    m1 = convertToByte(m1)
    m2 = convertToByte(m2)
	
    sendSpeedToMotor(m1,m2)
	
    print "Motor 1: ", m1, "Motor 2: ", m2
	
    # sleep one second
    time.sleep(1)

    receive_data = readEncoders()
    e1,e2 = struct.unpack('ii',bytearray(receive_data[0:8]))
    
    print "Encoder Data: ", e1, e2
