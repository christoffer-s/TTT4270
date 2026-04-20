import smbus2
import time
import struct
import numpy as np

MPU_ADDR = 0x68
ACCEL_XOUT_H = 0x3B
ACCEL_SCALE = 16384.0  # For +/- 2g
GYRO_SCALE = 131.0     # For +/- 250 deg/s

bus = smbus2.SMBus(1)
# Wake up & 1kHz modus (DLPF)
bus.write_byte_data(MPU_ADDR, 0x6B, 0)
bus.write_byte_data(MPU_ADDR, 0x1A, 1)

def IMU():
    raw = bus.read_i2c_block_data(MPU_ADDR, ACCEL_XOUT_H, 14)
    data = struct.unpack('>7h', bytes(raw))
    
    f_imu = data[0:3]
    w_imu = data[3:6]
    print("w {w_imu}")
    return f_imu, w_imu