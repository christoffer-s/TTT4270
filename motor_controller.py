#import RPi.GPIO as GPIO

def motor_init() -> dict:
    ''' 
    Settting up GPIO pins for motors
    Data type:  motor = {some : PIN, something : PIN, somethingelse : PIN}
    Returns:    right, Left
    '''

    right_motor = {}
    left_motor = {}
    return right_motor, left_motor


r, l = motor_init()

