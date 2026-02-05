import motor_controller 

if __name__ == '__main__':
    motor_controller.motor_init()    # init pins for motors
    motor_controller.forward(100)
