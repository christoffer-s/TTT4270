#!/usr/bin/env python
import time
import serial
import pynmea2
import csv
import numpy as np







def get_gps():
	ser = serial.Serial(
	port='/dev/ttyAMA0',
	baudrate = 9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
)
	
	x=ser.readline().decode('utf-8', errors='ignore')
	if x.startswith('$'):
		try:
			msg = pynmea2.parse(x)
#			if isInstance(msg, pynmea2.RMC): # and msg.status == 'A':
#			data_csv.append([msg.latitude, msg.longitude, msg.speed_kph])
			data_csv = np.array([msg.latitude, msg.longitude])
			gps_read = True
			# return data_csv
		except pynmea2.ParseError as e:
			print("ParseError")
		except Exception as e:
			print(f"Error {e}")
	else:
		data_csv = np.array([0.0,0.0])
		gps_read = False
		# time.sleep(0.1)
	
	file_path = 'gps_data.csv'

	with open(file_path, mode='w', newline='') as file:
		csv_writer = csv.writer(file)
		csv_writer.writerow(data_csv)
	# print("CSV file made")
	return data_csv, gps_read



print(get_gps())


