import sys
import os
import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from motor_ctrl import Drive, Motor

right_motor = Motor(6, 12) #Koble GPIO 6 til pin 4 på motor driver board, og GPIO 12 til pin 3
left_motor = Motor(16, 13) #Koble GPIO 16 til pin 8 på motor driver board, og GPIO 13 til pin 5
back_left_motor = Motor(17, 18) #Koble GPIO 17 til pin 12 på motor driver board, og GPIO 18 til pin 11
back_right_motor = Motor(26, 19) #Koble GPIO 26 til pin 7 på motor driver board, og GPIO 19 til pin 6

drive = Drive(right_motor, left_motor, back_left_motor, back_right_motor)

drive.drive(0.5, 0)
time.sleep(4)
drive.drive(0.5, 0.5)
time.sleep(4)
drive.drive(0.5, -0.5)
time.sleep(4)
drive.drive(-0.5, 0)
time.sleep(4)
drive.stop()