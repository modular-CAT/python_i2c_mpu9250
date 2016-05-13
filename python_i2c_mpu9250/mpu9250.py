# Copyright (c) 2016 Daniel Smith
# Author: Daniel Smith
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import time
import smbus
import sys
import serial

bus = smbus.SMBus(1)

class mpu9250():
    # Sensor addresses
    MPU9250_I2C_ADDR = 0x68
    MAG_I2C_ADDR = 0x0C

    # Config constants (based on register map)
    GYRO_FULLSCALE_250DPS = 0x00
    GYRO_FULLSCALE_500DPS = 0x08
    GYRO_FULLSCALE_1000DPS = 0x10
    GYRO_FULLSCALE_2000DPS = 0x18

    ACC_FULLSCALE_2G = 0x00
    ACC_FULLSCALE_4G = 0x08
    ACC_FULLSCALE_8G = 0x10
    ACC_FULLSCALE_16G = 0x18

    PWR_WAKE = 0x00

    CONFIG_1KH_SAMPLE_RATE = 0x01 #see register map p.13 for more on config register!

    I2C_SLAVE0_BYPASS_ENABLE = 0x02

    MAG_SINGLE_MEASURE_MODE = 0x01
    MAG_CONT_MEASURE_MODE_1 = 0x02
    MAG_CONT_MEASURE_MODE_2 = 0x06
    MAG_EXT_TRIGGER_MEASURE_MODE = 0x04
    MAG_SELF_TEST_MODE = 0x08
    MAG_FUSE_ROM_ACCESS_MODE = 0x0f

    # MPU Register locations (based on register map)
    SAMPLEDIV_ADDR = 0x19
    CONF_ADDR = 0x1a
    GYRO_CONF_ADDR = 0x1b
    ACC_CONFIG_ADDR = 0x1c
    ACC_CONFIG2_ADDR = 0x1d

    SENSOR_BASE_ADDR = 0x3B
    ACC_BASE_ADDR = 0x3B
    TEMP_BASE_ADDR = 0x41
    GYRO_BASE_ADDR = 0x43

    PWR_MGMT_1_ADDR = 0x6b

    I2C_SLAVE0_ADDR = 0x37 # used to control magnetometer

    # Accelerometer Register Locations
    MAG_STATUS_ADDR = 0x02
    MAG_SENSOR_BASE_ADDR = 0x03
    MAG_CONTROL_ADDR = 0x0A

    # Sensor input variables
    accX = 0
    accY = 0
    accZ = 0
    gyroX = 0
    gyroY = 0
    gyroZ = 0
    tempOut = 0
    magX = 0
    magY = 0
    magZ = 0
    # Gyro offsets TODO see if these can be moved onboard the sensor
    gyrXoffs = 0
    gyrYoffs = 0
    gyrZoffs = 0


    def __init__(self):
        bus.write_byte_data(self.MPU9250_I2C_ADDR, self.PWR_MGMT_1_ADDR,  self.PWR_WAKE)
        bus.write_byte_data(self.MPU9250_I2C_ADDR, self.CONF_ADDR, self.CONFIG_1KH_SAMPLE_RATE)
        bus.write_byte_data(self.MPU9250_I2C_ADDR, self.GYRO_CONF_ADDR, self.GYRO_FULLSCALE_500DPS)
        bus.write_byte_data(self.MPU9250_I2C_ADDR, self.ACC_CONFIG_ADDR, self.ACC_FULLSCALE_4G)

        # set sample_div to divide sample rate down to 30Hz
        # f = sample rate / (sample_div + 1)
        # So if sample_div=32, f=30Hz
        bus.write_byte_data(self.MPU9250_I2C_ADDR, self.SAMPLEDIV_ADDR, 32)

        # set bypass mode for magnetometer
        bus.write_byte_data(self.MPU9250_I2C_ADDR, self.I2C_SLAVE0_ADDR, self.I2C_SLAVE0_BYPASS_ENABLE)

        self.calibrate_gyro()

    def read_all(self):
        """
        Read all sensors (3x accelerometer, 3x gyro, 3x magnetometer, 1x temperature)
        Throws an error on failure
        """
        # Read gyro, temp and accelerometer data (all in same spot)
        Data = bus.read_i2c_block_data(self.MPU9250_I2C_ADDR,self.SENSOR_BASE_ADDR)
        self.accX = Data[0]<<8 | Data[1]
        self.accY = Data[2]<<8 | Data[3]
        self.accZ = Data[4]<<8 | Data[5]

        self.tempOut = Data[6]<<8 | Data[7]

        self.gyroX = Data[8]<<8 | Data[9] - self.gyrXoffs
        self.gyroY = Data[10]<<8 | Data[11] - self.gyrYoffs
        self.gyroZ = Data[12]<<8 | Data[13] - self.gyrZoffs

        # Read magnetometer data - currently not working
        bus.write_byte_data(self.MAG_I2C_ADDR, self.MAG_CONTROL_ADDR, self.MAG_CONT_MEASURE_MODE_1)
        mag_status = bus.read_i2c_block_data(self.MAG_I2C_ADDR, self.MAG_STATUS_ADDR)
        while mag_status[0] == 0:
            mag_status = bus.read_i2c_block_data(self.MAG_I2C_ADDR, self.MAG_STATUS_ADDR) # poll until ready
        Data = bus.read_i2c_block_data(self.MAG_I2C_ADDR, self.MAG_SENSOR_BASE_ADDR) # read data
        self.magX = Data[0]<<8 | Data[1]
        self.magY = Data[2]<<8 | Data[3]
        self.magZ = Data[4]<<8 | Data[5]

    def calibrate_gyro(self):
        """
        Calibrate the gyros before use by calculating bias (subtracted from later readings).
        If this fails, it will throw an Error
        """
        xSum = 0
        ySum = 0
        zSum = 0
        cnt = 500
        for i in range(cnt):
            Data = bus.read_i2c_block_data(self.MPU9250_I2C_ADDR,self.GYRO_BASE_ADDR)
            xSum = xSum + (Data[0]<<8 | Data[1])
            ySum = xSum + (Data[2]<<8 | Data[3])
            zSum = xSum + (Data[4]<<8 | Data[5])
        self.gyrXoffs = xSum / cnt
        self.gyrYoffs = ySum / cnt
        self.gyrZoffs = zSum / cnt
        print "gyrXoffs", "gyrYoffs", "gyrZoffs = ",self.gyrXoffs,self.gyrYoffs,self.gyrZoffs
