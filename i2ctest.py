import smbus
import time

#initialize the bus
bus = smbus.SMBus(1)

# Arduino i2c address
address = 0x04

move_cmd = ord('m')
encoder_cmd = ord('e')

def writeNumber(m1,m2):
    bus.write_i2c_block_data(address, move_cmd, [m1, m2, 0]);
    
    return -1

def readNumber():
    number = bus.read_byte(address)
    # number = bus.read_byte_data(address, 1)
    return number

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
	
    writeNumber(m1,m2)
	
    print "Motor 1: ", m1, "Motor 2: ", m2
	
    # sleep one second
    time.sleep(1)

    number = readNumber()
    print "Arduino: Hey RPI, I received a digit ", number
