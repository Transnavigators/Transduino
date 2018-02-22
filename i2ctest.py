import smbus
import time

#initialize the bus
bus = smbus.SMBus(1)

# Arduino i2c address
address = 0x04

move_cmd = ord('m')
encoder_cmd = ord('e')

def writeNumber(m1,m2):
    bus.write_i2c_block_data(address, move_cmd, [value1, value2, 0]);
    
    return -1

def readNumber():
    number = bus.read_byte(address)
    # number = bus.read_byte_data(address, 1)
    return number

while True:
    m1 = input("Enter value for motor 1 [-127,127] : ")
    m2 = input("Enter value for motor 2 [-127,127] : ")

    writeNumber(m1,m2)
	
    print "Motor 1: ", m1, "Motor 2: ", m2
	
    # sleep one second
    time.sleep(1)

    number = readNumber()
    print "Arduino: Hey RPI, I received a digit ", number
