
import smbus
import time
import math
from LSM9DS0 import *

bus = smbus.SMBus(1)

LA_So = .000732 # g/LSB (16g)
M_GN = 0.48 # mgauss/LSB (12 gauss)
G_So = 0.07 # dps/LSB (2000dps)
GYRx_offset = 0 #3.4251
GYRy_offset = 0 #-1.6323
GYRz_offset = 0 #-10.2034
timestart = time.time()

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
writeGRY(CTRL_REG1_G, 0b00001111) #Normal power mode, all axes enabled
writeGRY(CTRL_REG4_G, 0b00110000) #Continuos update, 2000 dps full scale

MAGx_max=MAGy_max=MAGz_max=0
MAGx_min=MAGy_min=MAGz_min=0
print "calibrating 3..."
time.sleep(1)
print "calibrating 2..."
time.sleep(1)
print "calibrating 1..."
time.sleep(1)
start=time.time()
timer=0
while timer<10:
	MAGx = M_GN*readMAGx()
	MAGy = M_GN*readMAGy()
	MAGz = M_GN*readMAGz()
	if MAGx > MAGx_max : MAGx_max = MAGx 
	if MAGx < MAGx_min : MAGx_min = MAGx
	if MAGy > MAGy_max : MAGy_max = MAGy
	if MAGy < MAGy_min : MAGy_min = MAGy
	if MAGz > MAGz_max : MAGz_max = MAGz
	if MAGz < MAGz_min : MAGz_min = MAGz
	timer=time.time()-start
MAGx_bias  = (MAGx_max + MAGx_min)/2
MAGy_bias  = (MAGy_max + MAGy_min)/2
MAGz_bias  = (MAGz_max + MAGz_min)/2
MAGx_scalea = (MAGx_max - MAGx_min)/2
MAGy_scalea = (MAGy_max - MAGy_min)/2
MAGz_scalea = (MAGz_max - MAGz_min)/2
avg_scale = (MAGx_scalea+MAGy_scalea+MAGz_scalea)/3
MAGx_scale = avg_scale/MAGx_scalea
MAGy_scale = avg_scale/MAGy_scalea
MAGz_scale = avg_scale/MAGz_scalea
print "MAGx bias = %3.4f, MAGx scale = %3.4f" % (MAGx_bias,MAGx_scale)
print "MAGy bias = %3.4f, MAGy scale = %3.4f" % (MAGy_bias,MAGy_scale)
print "MAGz bias = %3.4f, MAGz scale = %3.4f" % (MAGz_bias,MAGz_scale)
