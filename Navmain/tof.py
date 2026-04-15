import smbus
import sys
import time



def read_tof():
        bus = smbus.SMBus(1)
        data = bus.read_byte_data(0x29, 0x00)
        return data

# while True:
#         value = read_data()
#         print(value)
#         time.sleep(0.2)