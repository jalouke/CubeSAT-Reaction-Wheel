import smbus
import mraa 
import time 
import math
import numpy as np
import scipy.signal as signal
import IMU
import sys

#########################################################
# initializing Variables

global A_motor_velocity,B_motor_velocity,C_motor_velocity,D_motor_velocity
A_motor_velocity=B_motor_velocity=C_motor_velocity=D_motor_velocity=0
A=B=C=D=E=F=G=H=I=0
P = 0.3 # proportional control value
timer = 1
output = ['A','B','C','D','E','F','G','H','I']
Pin = [14,20,0,21,36,48,47,32,46] #PWM for GP13,GP12,GP182,GP183 Gpio for GP14,GP15,GP49,GP46,GP47

##########################################################
# enabling outputs 
for x in xrange(0,4):
	output[x] = mraa.Pwm(Pin[x])
	output[x].period_us(700)
	output[x].enable(True)
	print x, output[x]
for x in xrange(4,9):
	output[x] = mraa.Gpio(Pin[x])
	output[x].dir(mraa.DIR_OUT)
	print x, output[x]
[Apwm,Bpwm,Cpwm,Dpwm,Adir,Bdir,Cdir,Ddir,mode] = output
output[8].write(1) #Set mode pin to high for pwm/direction
print Apwm
###########################################################
def A_motor_dir():
        global A_motor_speed
        if A_motor_velocity >= 0:
                 A_motor_dir = 1
                 A_motor_speed = abs(A_motor_velocity)/100
                 if A_motor_speed > 1:
                         A_motor_speed = 1
        elif A_motor_velocity < 0:
                 A_motor_dir = 0
                 A_motor_speed = abs(A_motor_velocity)/100
                 if A_motor_speed > 1:
                         A_motor_speed = 1
        Adir.write(A_motor_dir)
def B_motor_dir():
        global B_motor_speed
        if B_motor_velocity >= 0:
                 B_motor_dir = 1
                 B_motor_speed = abs(B_motor_velocity)/100
                 if B_motor_speed > 1:
                         B_motor_speed = 1
        elif B_motor_velocity < 0:
                 B_motor_dir = 0
                 B_motor_speed = abs(B_motor_velocity)/100
                 if B_motor_speed > 1:
                         B_motor_speed = 1
        Bdir.write(B_motor_dir)
def C_motor_dir():
        global C_motor_speed
        if C_motor_velocity >= 0:
                 C_motor_dir = 1
                 C_motor_speed = abs(C_motor_velocity)/100
                 if C_motor_speed > 1:
                         C_motor_speed = 1
        elif C_motor_velocity < 0:
                 C_motor_dir = 0
                 C_motor_speed = abs(C_motor_velocity)/100
                 if C_motor_speed > 1:
                         C_motor_speed = 1
        Cdir.write(C_motor_dir)
def D_motor_dir():
        global D_motor_speed
        if D_motor_velocity >= 0:
                 D_motor_dir = 1
                 D_motor_speed = abs(D_motor_velocity)/100
                 if D_motor_speed > 1:
                         D_motor_speed = 1
        elif D_motor_velocity < 0:
                 D_motor_dir = 0
                 D_motor_speed = abs(D_motor_velocity)/100
                 if D_motor_speed > 1:
                         D_motor_speed = 1
        Ddir.write(D_motor_dir)

def roll_control():
	global A_motor_velocity,B_motor_velocity,C_motor_velocity,D_motor_velocity
	t_a=time.time()
	
        [ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,MAGx,MAGy,MAGz] = IMU.read()
	
        A_motor_velocity = P*GYRz + A_motor_velocity
        B_motor_velocity = P*GYRz + B_motor_velocity
        C_motor_velocity = P*GYRz + C_motor_velocity
        D_motor_velocity = P*GYRz + D_motor_velocity
        
        A_motor_dir()
        B_motor_dir()
        C_motor_dir()
        D_motor_dir()
        
        Apwm.write(A_motor_speed)
        Bpwm.write(B_motor_speed)
        Cpwm.write(C_motor_speed)
        Dpwm.write(D_motor_speed)
	t = time.time() - timestart
	Data.write('%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\n' % (t,ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,MAGx,MAGy,MAGz,A_motor_speed))
	print '2',ACCz
	return ACCz
	if ACCz <= -2: 
        	Adir.write(0)
                Bdir.write(0)
                Cdir.write(0)
                Ddir.write(0)
                Apwm.write(0) 
                Bpwm.write(0) 
                Cpwm.write(0) 
                Dpwm.write(0) 
                sys.exit() 

filename = time.strftime("%Y-%m-%d_%H:%M:%S")
Data = open(('Drop_Test/CubeSat_Drop').__add__(filename).__add__('.csv'), 'a')
Data.write('Time,ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,MAGx,MAGy,MAGz,Motor_Speed\n')
timestart = time.time()
while True:
        [ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,MAGx,MAGy,MAGz] = IMU.read()
	print '0',ACCz
	timer = 0
	if ACCz > -.8:
		timestarta = time.time()
		time.sleep(0.1)
		while timer <= 0.8:
			Adir.write(0)
			Bdir.write(0)
			Cdir.write(0)
			Ddir.write(0)
			Apwm.write(75)
        		Bpwm.write(75)
        		Cpwm.write(75)
        		Dpwm.write(75)
			[ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,MAGx,MAGy,MAGz] = IMU.read()
			t=time.time()-timestart
			Data.write('%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\n' % (t,ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,MAGx,MAGy,MAGz))
			timer = time.time() - timestarta
			print '1',ACCz
			timer_b = time.time()
			if ACCz <= -2:
            			Adir.write(0)
        		        Bdir.write(0)
		                Cdir.write(0)
		                Ddir.write(0)
		                Apwm.write(0)
		                Bpwm.write(0)
		                Cpwm.write(0)
	        	        Dpwm.write(0)
		                sys.exit()
		timer = time.time() - timestarta
		while timer <= 1.0 & timer >= 0.8:
			ACCz = roll_control()
			timer = time.time() - timestarta
		while timer >= 1.0 & timer <= 1.8:
			Adir.write(1)
                        Bdir.write(1)
                        Cdir.write(1)
                        Ddir.write(1)
                        Apwm.write(75)
                        Bpwm.write(75)
                        Cpwm.write(75)
                        Dpwm.write(75)
                        [ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,MAGx,MAGy,MAGz] = IMU.read()
                        t=time.time()-timestart
                        Data.write('%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\n' % (t,ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,MAGx,MAGy,MAGz))
                        timer = time.time() - timestarta
		while timer > 1.8:
			ACCz = roll_control()
                        timer = time.time() - timestarta
		if ACCz <= -2:
			Adir.write(0)
        		Bdir.write(0)
        		Cdir.write(0)
        		Ddir.write(0)
        		Apwm.write(0)
        		Bpwm.write(0)
        		Cpwm.write(0)
        		Dpwm.write(0)
			sys.exit()
