from gpiozero import PWMOutputDevice, DigitalOutputDevice
from time import sleep
'''
Everything related to motor control

On the motor driver board: 
    M1 = right motor
    M3 = left motor

'''
class Motor:
    def __init__(self, dir_pin, pwm_pin):
        self.dir = DigitalOutputDevice(dir_pin)
        self.pwm = PWMOutputDevice(pwm_pin)

    def forward(self, speed):
        self.dir.off()
        self.pwm.value = speed #speed between 0 and 1

    def backward(self, speed):
        self.dir.on()
        self.pwm.value = speed #speed between 0 and 1

    def stop(self):
        self.pwm.value = 0


right_motor = Motor(6, 12) #Koble GPIO 6 til pin 4 på motor driver board, og GPIO 12 til pin 3
left_motor = Motor(16, 13) #Koble GPIO 16 til pin 8 på motor driver board, og GPIO 13 til pin 5

class Drive:
    def __init__(self, right_motor, left_motor):
        self.right_motor = right_motor
        self.left_motor = left_motor
        self.speed = 0.5 #Default speed
        self.turn_ratio = 0.5 #Default turn ratio
    
    def set_speed(self, speed):
        self.speed = speed
    
    def forward(self):
        self.right_motor.forward(self.speed)
        self.left_motor.forward(self.speed)
    
    def backward(self):
        self.right_motor.backward(self.speed)
        self.left_motor.backward(self.speed)
    
    def drive(self, forward_speed, turn_rate):
        """
        forward_speed: -1 to 1
        turn_rate: -1 to 1 (negative = left, positive = right)
        """
        right = forward_speed - turn_rate
        left  = forward_speed + turn_rate

        # clamp mellom -1 og 1
        right = max(-1, min(1, right))
        left  = max(-1, min(1, left))

        if right >= 0:
            self.right_motor.forward(right)
        else:
            self.right_motor.backward(abs(right))
        
        if left >= 0:
            self.left_motor.forward(left)
        else:
            self.left_motor.backward(abs(left))

    def stop(self):
        self.right_motor.stop()
        self.left_motor.stop()
            

drive = Drive(right_motor, left_motor)

while True:
    drive.drive(0.5, 0) #Kjør rett frem med halv fart
    sleep(3)
    drive.stop() #Stopp
    sleep(3)
    drive.drive(-0.5, 0) #Kjør bakover med halv fart
    sleep(3)
    drive.drive(-0.5, 0.2) #Kjør bakover med halv fart
    sleep(3)
    drive.stop() #Stopp
    sleep(3)
    break