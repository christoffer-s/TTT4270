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
	ser.write(b"$PMTK251,115200*1F\r\n") #Changes baud rate to 115200
	time.sleep(0.5)
	ser.close()
	ser = serial.Serial(
	port='/dev/ttyAMA0',
	baudrate = 9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
	)
	ser.write(b"$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n")
	time.sleep(0.1)
	ser.write(b"$PMTK220,100*2F\r\n") #Set frequency to 10 Hz (100ms interval)
	
	x=ser.readline().decode('utf-8', errors='ignore')
	if x.startswith('$'):
		try:
			msg = pynmea2.parse(x)
			data_csv = np.array([msg.latitude, msg.longitude])
		except pynmea2.ParseError as e:
			print("ParseError")
			data_csv = np.array([0.0,0.0])
		except Exception as e:
			print(f"Error {e}")
			data_csv = np.array([0.0,0.0])
	else:
		data_csv = np.array([0.0,0.0])
	
	file_path = 'gps_data.csv'
	with open(file_path, mode='w', newline='') as file:
		csv_writer = csv.writer(file)
		csv_writer.writerow(data_csv)
	return data_csv
