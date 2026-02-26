import time
import board
import adafruit_mpu6050
import time
import math

i2c = board.I2C() # uses board.SCL and board.SDA
mpu = adafruit_mpu6050.MPU6050(i2c)


# ------------------------------------------------

# Initialization
velocity = [0.0, 0.0, 0.0]
position = [0.0, 0.0, 0.0]
last_time = time.time()
G_MAGNITUDE = 9.81 # Magnitude of gravity

# Main loop for data acquisition and integration
def vel_pos(velocity, position, yaw_angle, last_time):
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    # Get accelerometer and gyroscope data
    accel_data = list(mpu.acceleration())
    gyro_data = list(mpu.gyro())

    # --- Dead Reckoning/Integration Logic ---
    # The primary challenge is distinguishing actual motion from gravity and sensor drift. 
    # A simple approach for a *still* or *mostly* level sensor:

    # 1. Compensate for gravity (simplistic approach, assumes z is up/down)
    #    For robust motion tracking, sensor fusion algorithms (Kalman filter, complementary filter)
    #    and rotation matrices are necessary.
    accel_x = accel_data[0] 
    accel_y = accel_data[1] 
    accel_z = accel_data[2] - G_MAGNITUDE # Subtract gravity from the Z axis

    # 2. Integrate acceleration to get velocity
    velocity[0] += accel_x * dt
    velocity[1] += accel_y * dt
    velocity[2] += accel_z * dt

    # 3. Integrate velocity to get position
    position[0] += velocity[0] * dt
    position[1] += velocity[1] * dt
    position[2] += velocity[2] * dt

    yaw_angle += gyro_data[2] * dt # Z-axis data integrated to get yaw
    
    yaw_deg = math.degrees(yaw_angle)

    # Print results (adjust as needed)
    # print(f"Time Delta: {dt:.4f}s")
    # print(f"Velocity (m/s): X={velocity[0]:.2f}, Y={velocity[1]:.2f}, Z={velocity[2]:.2f}")
    # print(f"Position (m): X={position[0]:.2f}, Y={position[1]:.2f}, Z={position[2]:.2f}")
    # print("-" * 30)

    

    # Small delay to control loop rate
    time.sleep(0.1) 
    return velocity, position, yaw_angle, last_time, yaw_deg # m/s, m, float, s, degrees
