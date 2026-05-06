import smbus

def read_tof():
        bus = smbus.SMBus(1)
        data = bus.read_i2c_block_data(0x29, 0x00)
        return data