# https://elec3004.uqcloud.net/2015/tutes/Kalman_and_Bayesian_Filters_in_Python.pdf
# Tar utgangspunkt i kapittell 9 på side 190

#!/usr/bin/env python
import time
import serial
import pynmea2
import numpy as np
from filterpy.kalman import KalmanFilter
import matplotlib.pyplot as plt
import numpy.random as random
import copy
import pyproj
import pymap3d as pm
from Navmain import Main2

P = pyproj.Proj(proj='utm', zone=31, ellps='WGS84', preserve_units=True)
G = pyproj.Geod(ellps='WGS84')

# def gps_to_LonLat(gpsLon,gpsLat):

#     return 0


def LonLat_To_XY(Lon, Lat):
    return P(Lon, Lat)    

def XY_To_LonLat(x,y):
    return P(x, y, inverse=True)    

def distance(Lat1, Lon1, Lat2, Lon2):
    return G.inv(Lon1, Lat1, Lon2, Lat2)[2]



# 1. Define the origin (0,0,0)
lat0, lon0, h0 = 63.418112, 10.402374 , 0  

def gps_to_enu():
	gps = get_gps()
	x, y = Main2.lon_lat_til_xy(gps[1], gps[0])
	return x, y
# Output: (e, n, u) in meters, where (0,0,0) is (lat0, lon0, h0)





ser = serial.Serial(
	port='/dev/ttyAMA0',
	baudrate = 9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=1
)

#------------------------------------------------------------------
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

	# time.sleep(0.1)
	return np.array([0,0])
# ------------------------------------------------------------------



def gps_to_XY():
	gps = get_gps()
	return list(LonLat_To_XY(gps[1],gps[0]))

distance = 111111000 # 111.111km to calculate dx and dy
dt = 0.1

kf = KalmanFilter (dim_x=4, dim_z=2)  
# lager kalmanfilter klasse med 4 dimensjons output, x_pos, y_pos
# skal ha inn 2 sensormålinger, gps: latitude, longitude



initPos = gps_to_enu()
kf.x = np.array([[initPos[0],initPos[1],0,0]]).T # x, y, v_x, y, v_y


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


# --------------------------------------------
# Kode for å lage random path til å teste filter

class PosSensor1(object):
	def __init__(self, pos = [0,0], vel = (0,0), noise_scale = 1.0):
		self.vel = vel
		self.noise_scale = noise_scale
		self.pos = copy.deepcopy(pos)
	def read(self):
		self.pos[0] += self.vel[0]
		self.pos[1] += self.vel[1]
		self.pos[0] += (random.uniform(0, 10.0) * 0.9 )
		self.pos[1] += (random.uniform(0, 10.0) * 0.9 )
		return [self.pos[0], self.pos[1]]

pxs, pys = [], [] # generere data for random path
s = PosSensor1([10,100], [20,1], [1.])

# ---------------------------------------------

xs, ys = [], [] # Lister med predicted data for x pos, y pos



def kalmanFilter_update(KF, posisjon): # Funksjon for å oppdatere filter og hente ut posisjon og fart data
	try:
		#gps_data = get_gps()
		KF.predict()
		KF.update(posisjon)
		# xs.append(KF.x[0,0])
		# ys.append(KF.x[2,0])
		return 0
	except Exception as e:
		print(f"Error catched: {e}")
		return 1
	


while True:
	pos = gps_to_enu()
	kalmanFilter_update(kf, pos)
	print(kf.x)



# plt.plot (xs, ys, "r--")
# plt.plot (pxs, pys)
# plt.legend(["filter", "measurement"])
# plt.savefig("Test plot")