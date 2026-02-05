#import RPi.GPIO as GPIO

'''
Everything related to motor controll
'''


def motor_init() -> None:
    ''' 
    Settting up GPIO pins for motors
    Data type:  motor = {some : PIN, something : PIN, somethingelse : PIN}
    Makes right_motor and left_motor global variables in motor_controller.py
    '''
    global right_motor
    global left_motor

    # junk data
    right_motor = {"direction":10}
    left_motor = {"direction":20}   


def forward(time: int) -> None:
    ''' Drives forward
        Input: time [milliseconds]
    '''
    print(right_motor["direction"], left_motor["direction"])


def backward(time: int) -> None:
    ''' Drives backward
        Input: time [milliseconds]
    '''
    print(right_motor["direction"], left_motor["direction"])


def left(time: int) -> None:
    ''' Drives left
        Input: time [milliseconds]
    '''
    print(right_motor["direction"], left_motor["direction"])


def right(time: int) -> None:
    ''' Drives right
        Input: time [milliseconds]
    '''
    print(right_motor["direction"], left_motor["direction"])
    