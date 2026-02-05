''' IVAR python script '''

import motor_ctrl

if __name__ == '__main__':
    motor_ctrl.motor_init()    # init pins for motors
    motor_ctrl.forward(100)
