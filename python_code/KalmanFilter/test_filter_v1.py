# https://elec3004.uqcloud.net/2015/tutes/Kalman_and_Bayesian_Filters_in_Python.pdf
# Tar utgangspunkt i kapittell 9 på side 190

#!/usr/bin/env python
import time
import serial
import pynmea2
import numpy as np
from filterpy.kalman import KalmanFilter

ser = serial.Serial(
	port='/dev/ttyAMA0',
	baudrate = 9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
)

# ------------------------------------------------------------------
def get_gps():
	x=ser.readline().decode('utf-8', errors='ignore')
	if x.startswith('$'):
		try:
			msg = pynmea2.parse(x)
			return np.array([msg.latitude, msg.longitude])
			# if isInstance(msg, pynmea2.RMC): # and msg.status == 'A':
				# print('-'*20)
				# print(mgs.timestamp)
				# print(msg.latitude, msg.lat_dir)
				# print(msg.longitude, msg.lon_dir)
				# print(mgs.speed_kph)
				# print('-'*20)
		except pynmea2.ParseError as e:
			print("ParsError")
		except Exception as e:
			print(f"Error {e}")

	time.sleep(0.1)
	return np.array([0,0])
# ------------------------------------------------------------------

distance = 111111000 # 111.111km to calculate dx and dy
dt = 0.1

kf = KalmanFilter (dim_x=4, dim_z=2)  
# lager kalmanfilter klasse med 4 dimensjons output, x_pos, y_pos, x_vel og y_vel
# skal ha inn 3 sensormålinger, gps: latitude, longitude, velocity

kf.x = np.array([[0,0,0,0]]).T # x, v_x, y, v_y

kf.F = np.array([[1, dt, 0, 0],
                [0,1,0,0],
                [0,0,1,dt],
                [0,0,0,1]])

kf.H = np.array([[1,0,0,0],[0,0,1,0]]) 
# Konverter data, kan få til å gjøre mellom lat- og longtitude til x og y 
# ved å ta d_lat eller d_long å gange med distance?

kf.R = np.array([[2,0],[0,2]])  # Noice matrise, antar 2 meter varians i posisjon

kf.Q = np.eye(4) * 0.1 # Process noice matrise, idk hva den er må tweakes

kf.P = np.eye(4) * 100. # koovarians matrise, hvor god gjett start kondisjonene våre er

# print(kf.x)
# print(kf.H)
# print(kf.R)
# print(kf.Q)
# print(kf.P)



def kalmanFilter_update():
	gps_data = get_gps()
	kf.predict()
	kf.update(gps_data) 
	return 0
