import time
import board
import busio
import adafruit_vl6180x

#I2C pins
i2c = busio.I2C(board.SCL, board.SDA)

sensor = adafruit_vl6180x.VL6180X(i2c)

#Print er avstand og lux hvert sekund
while True:
    range_mm = sensor.range
    print(range_mm, "mm")
    # Read the light, note this requires specifying a gain value:
    # - adafruit_vl6180x.ALS_GAIN_1 = 1x
    # - adafruit_vl6180x.ALS_GAIN_1_25 = 1.25x
    # - adafruit_vl6180x.ALS_GAIN_1_67 = 1.67x
    # - adafruit_vl6180x.ALS_GAIN_2_5 = 2.5x
    # - adafruit_vl6180x.ALS_GAIN_5 = 5x
    # - adafruit_vl6180x.ALS_GAIN_10 = 10x
    # - adafruit_vl6180x.ALS_GAIN_20 = 20x
    # - adafruit_vl6180x.ALS_GAIN_40 = 40x
    light_lux = sensor.read_lux(adafruit_vl6180x.ALS_GAIN_1)
    print(f"Light (1x gain): {light_lux}lux")

    time.sleep(1.0)