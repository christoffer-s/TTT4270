#!/usr/bin/env python
import time
import serial
import pynmea2

ser = serial.Serial(
	port='/dev/ttyAMA0',
	baudrate = 9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
)

while 1:
	x=ser.readline().decode('utf-8', errors='ignore')
	if x.startswith('$'):
		try:
			ms = pynmea2.parse(x)
			if isInstance(msg, pynmea2.RMC) and msg.status == 'A':
				print('-'*20)
				print(mgs.timestamp)
				print(msg.latitude, msg.lat_dir)
				print(msg.longitude, msg.lon_dir)
				print(mgs.speed_kph)
				print('-'*20)
		except pynmea2.ParseError as e:
			continue
		except Exception as e:
			print(f"Error {e}")

	time.sleep(0.1)





