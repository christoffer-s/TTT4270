#!/usr/bin/env python
import time
import serial
import pynmea2
import csv

ser = serial.Serial(
	port='/dev/ttyAMA0',
	baudrate = 9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
)



x=ser.readline().decode('utf-8', errors='ignore')

def get_gps():
	data_csv = [["Latitude", "Longitude", "Velocity"]]
	if x.startswith('$'):
		try:
			msg = pynmea2.parse(x)
#			if isInstance(msg, pynmea2.RMC): # and msg.status == 'A':
#			data_csv.append([msg.latitude, msg.longitude, msg.speed_kph])
			data_csv.append([msg.latitude, msg.longitude])
		except pynmea2.ParseError as e:
			print("ParseError")
		except Exception as e:
			print(f"Error {e}")

		time.sleep(0.1)
	
	file_path = 'gps_data.csv'

	with open(file_path, mode='w', newline='') as file:
		csv_writer = csv.writer(file)
		csv_writer.writerows(data_csv)
	print("CSV file made")
	return data_csv






