# To ust this example, connect your sensor to I2C2.
# If using a grove module and beaglebone grove cape,
# use the J5 grove socket on the cape. If using the
# beaglebone green, you can use the grove I2C socket
# on the beaglebone.

import time
import python_i2c_mpu9250 as mpu

sensor = mpu.mpu9250() # default pin is AIN0
while True:
    value = sensor.read_all()
    print value
    time.sleep(0.2)
