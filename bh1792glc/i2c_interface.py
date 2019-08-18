import time
import smbus
import struct

class SensorDriver(object):
    
    def __init__(self, deviceAddress, i2cBus = 1):
        if i2cBus > 1 or i2cBus < 0:
            raise
            
        if deviceAddress > 0x80:
            raise   #7bits addresses
    
        # The device to target in this instance of the class
        self.DEVICE_ADDRESS = deviceAddress
        # The designated I2C bus (raspi has just 0 or 1)
        self.i2c = smbus.SMBus(i2cBus)
        
    def write_block(self, addr, valarray = []):
        self.i2c.write_byte(self.DEVICE_ADDRESS, addr)
        #Wait tBUF here! 1.3us        #time.sleep(0.000002)
        return self.i2c.write_i2c_block_data(self.DEVICE_ADDRESS, addr, valarray)
        
    def write_byte(self, addr, val):
        arr = [val]
        return self.write_block(addr, arr)
        
    def write_short(self, addr, val):
        arr = [ (val & 0xFF), ((val >> 8) & 0xFF) ]
        return self.write_block(addr, arr)
        
    def write_word(self, addr, val):
        arr = [ (val & 0xFF), ((val >> 8) & 0xFF), ((val >> 16) & 0xFF), ((val >> 24) & 0xFF)]
        return self.write_block(addr, arr)
        
    def read_block(self, addr, size):
        self.i2c.write_byte(self.DEVICE_ADDRESS, addr)
        #Wait tBUF here! 1.3us
        res = self.i2c.read_i2c_block_data(self.DEVICE_ADDRESS, addr, size)
        if len(res) != size:
            raise
        return res
        
    def read_byte(self, addr):
        return self.read_block(addr, 1)[0]
        
    def read_short(self, addr):
        arr = self.read_block(addr, 2)
        return (arr[0] | (arr[1] << 8))
        
    def read_word(self, addr):
        arr = self.read_block(addr, 4)
        return (arr[0] | (arr[1] << 8) | (arr[2] << 16) | (arr[3] << 24))
    
    def read_register(self, addr):
        return self.read_byte(addr)