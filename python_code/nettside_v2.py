#!/usr/bin/env python
from flask import Flask, render_template
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

def get_data():
     x=ser.readline().decode('utf-8', errors='ignore')
     if x.startswith('$'):
#                try:
         msg = pynmea2.parse(x)
#                       if isInstance(msg, pynmea2.RMC): # and msg.status == 'A                               
         print('-'*20)
         print(msg.timestamp)
         print(msg.latitude, msg.lat_dir)
         print(msg.longitude, msg.lon_dir)
         print(msg.speed_kph)
         print('-'*20)
#                except pynmea2.ParseError as e:
#			print(f"ParseError")
#                except Exception as e:
#			print(f"Error {e}")

     time.sleep(0.1)
     return {"Time": msg.timestamp, "Latitude": msg.latitude, "Longitude": msg.longitude}



app = Flask(__name__)
@app.route('/')
def index():
	data = {"Time": "20", "Latitude":21, "Longitude":44} 
	return render_template('index.html', measurements=data)

if __name__ == '__main__':
	app.run(debug=True, host='0.0.0.0')
