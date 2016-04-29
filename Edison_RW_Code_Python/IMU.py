
import smbus
import time
import math
from LSM9DS0 import *

bus = smbus.SMBus(1)
LA_So = .000732 # g/LSB (16g)
M_GN = 0.48 # mgauss/LSB (12 gauss)
G_So = 0.00875 # dps/LSB (2000dps)
GYRx_offset = 109.08
GYRy_offset = -157.11
GYRz_offset = -124.2
timestart = time.time()
MAGx_bias = -18.0000
MAGx_scale = 1.0578
MAGy_bias = 61.4400
MAGy_scale = 1.0991
MAGz_bias = -126.2400
MAGz_scale = 0.8736

def writeACC(register,value):
        bus.write_byte_data(ACC_ADDRESS , register, value)
        return -1

def writeMAG(register,value):
        bus.write_byte_data(MAG_ADDRESS, register, value)
        return -1

def writeGRY(register,value):
        bus.write_byte_data(GYR_ADDRESS, register, value)
        return -1

def readACCx():
        acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_X_L_A)
        acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_X_H_A)
	acc_combined = (acc_l | acc_h <<8)

	return acc_combined  if acc_combined < 32768 else acc_combined - 65536

def readACCy():
        acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_Y_L_A)
        acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_Y_H_A)
	acc_combined = (acc_l | acc_h <<8)

	return acc_combined  if acc_combined < 32768 else acc_combined - 65536

def readACCz():
        acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_Z_L_A)
        acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_Z_H_A)
	acc_combined = (acc_l | acc_h <<8)

	return acc_combined  if acc_combined < 32768 else acc_combined - 65536

def readMAGx():
        mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_X_L_M)
        mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_X_H_M)
        mag_combined = (mag_l | mag_h <<8)

        return mag_combined  if mag_combined < 32768 else mag_combined - 65536

def readMAGy():
        mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_Y_L_M)
        mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_Y_H_M)
        mag_combined = (mag_l | mag_h <<8)

        return mag_combined  if mag_combined < 32768 else mag_combined - 65536


def readMAGz():
        mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_Z_L_M)
        mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_Z_H_M)
        mag_combined = (mag_l | mag_h <<8)

        return mag_combined  if mag_combined < 32768 else mag_combined - 65536

def readGYRx():
        gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_X_L_G)
        gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_X_H_G)
        gyr_combined = (gyr_l | gyr_h <<8)

        return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536
  
def readGYRy():
        gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_Y_L_G)
        gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_Y_H_G)
        gyr_combined = (gyr_l | gyr_h <<8)

        return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536

def readGYRz():
        gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_Z_L_G)
        gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_Z_H_G)
        gyr_combined = (gyr_l | gyr_h <<8)

        return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536

	
#initialise the accelerometer
writeACC(CTRL_REG1_XM, 0b01100111) #z,y,x axis enabled, continuos update,  100Hz data rate
writeACC(CTRL_REG2_XM, 0b00100000) #+/- 16G full scale

#initialise the magnetometer
writeMAG(CTRL_REG5_XM, 0b11110000) #Temp enable, M data rate = 50Hz
writeMAG(CTRL_REG6_XM, 0b01100000) #+/-12gauss
writeMAG(CTRL_REG7_XM, 0b00000000) #Continuous-conversion mode

#initialise the gyroscope
writeGRY(CTRL_REG1_G, 0b00001111) #Normal power mode, all axes enabled (95 Hz 12.5 cutoff)
writeGRY(CTRL_REG2_G, 0b00100001) #High-pass filter: Normal mode, 13.5 Hz
writeGRY(CTRL_REG4_G, 0b00000000) #Continuos update, 245 dps full scale


def read():
	global MAGx_scale,MAGy_scale,MAGz_scale,MAGx_bias,MAGy_bias,MAGz_bias,GYRx_offset,GYRy_offset,GYRz_offset
	a = time.time()
	ACCx = LA_So*readACCx()
	ACCy = LA_So*readACCy()
	ACCz = LA_So*readACCz()
	GYRx = G_So*(readGYRx() - GYRx_offset)
	GYRy = G_So*(readGYRy() - GYRy_offset)
	GYRz = G_So*(readGYRz() - GYRz_offset)
	MAGx = M_GN*(MAGx_scale * readMAGx()) + MAGx_bias
	MAGy = M_GN*(MAGy_scale * readMAGy()) + MAGy_bias
	MAGz = M_GN*(MAGz_scale * readMAGz()) + MAGz_bias
        IMU_val = [ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,MAGx,MAGy,MAGz]
        return IMU_val
