from gpiozero import PWMOutputDevice, DigitalOutputDevice
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

    def turn(self, direction, turn_ratio):
        if direction.lower() == "left":
            self.left_motor.forward(self.speed * turn_ratio) #turn_ratio between 0 and 1

        elif direction.lower() == "right":
            self.right_motor.forward(self.speed * turn_ratio)
    
    def stop(self):
        self.right_motor.stop()
        self.left_motor.stop()
            

drive = Drive(right_motor, left_motor)