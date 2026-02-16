from gpiozero import PWMOutputDevice, DigitalOutputDevice
from time import sleep
'''
Everything related to motor control
M1 = right motor
M3 = left motor

'''


def motor_init() -> None:
    ''' 
    Settting up GPIO pins for motors

    '''
    global right_motor_dir_pin
    global right_motor_pwm_pin
    global left_motor_dir_pin
    global left_motor_pwm_pin

    right_motor_dir_pin = DigitalOutputDevice(6)
    right_motor_pwm_pin = PWMOutputDevice(12)
    left_motor_dir_pin = DigitalOutputDevice(16)
    left_motor_pwm_pin = PWMOutputDevice(13)

def forward(speed: float) -> None:
    ''' Drives forward
        Input: speed [0.0 - 1.0]
    '''
    right_motor_dir_pin.off()
    left_motor_dir_pin.off()
    right_motor_pwm_pin.value = speed
    left_motor_pwm_pin.value = speed
    
def backward(speed: float) -> None:
    ''' Drives backward
        Input: speed [0.0 - 1.0]
    '''
    right_motor_dir_pin.on()
    left_motor_dir_pin.on()
    right_motor_pwm_pin.value = speed
    left_motor_pwm_pin.value = speed

def left(speed: float) -> None:
    ''' Drives left
        Input: speed [0.0 - 1.0]
    '''
    right_motor_dir_pin.off()
    left_motor_dir_pin.on()
    right_motor_pwm_pin.value = speed
    left_motor_pwm_pin.value = speed

def right(speed: float) -> None:
    ''' Drives right
        Input: speed [0.0 - 1.0]
    '''
    right_motor_dir_pin.on()
    left_motor_dir_pin.off()
    right_motor_pwm_pin.value = speed
    left_motor_pwm_pin.value = speed

def stop() -> None:
    ''' Stops the motors '''
    right_motor_pwm_pin.value = 0
    left_motor_pwm_pin.value = 0
    