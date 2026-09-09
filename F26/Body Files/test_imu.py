import time
import board
import busio
import adafruit_bno055

print("Creating I2C...")
i2c = busio.I2C(board.SCL, board.SDA)

print("Creating BNO055...")
sensor = adafruit_bno055.BNO055_I2C(i2c)

print("BNO055 connected!")

while True:
    print("Euler:", sensor.euler)
    print("Temp:", sensor.temperature)
    time.sleep(1)