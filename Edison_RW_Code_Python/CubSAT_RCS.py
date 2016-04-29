import smbus
import mraa 
import time 
import math
import numpy as np
import scipy.signal as signal
import IMU

##################################################
# initializing Variables

A_motor_velocity=B_motor_velocity=C_motor_velocity=D_motor_velocity=0
A=B=C=D=E=F=G=H=I=0
P = 0.3 # proportional control value
timer = 1
output = [A,B,C,D,E,F,G,H,I]
Pin = [14,20,0,21,36,48,47,32,46] #PWM for GP13,GP12,GP182,GP183 Gpio for GP14,GP15,GP49,GP46,GP47

######################################################
# enabling outputs 
for x in xrange(0,4):
	output[x] = mraa.Pwm(Pin[x])
	output[x].period_us(700)
	output[x].enable(True)
	#print x, output[x]
for x in xrange(4,9):
	output[x] = mraa.Gpio(Pin[x])
	output[x].dir(mraa.DIR_OUT)
	#print x, output[x]
[Apwm,Bpwm,Cpwm,Dpwm,Adir,Bdir,Cdir,Ddir,mode] = output
output[8].write(1) #Set mode pin to high for pwm/direction

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

	return [GYRz,A_motor_velocity,B_motor_velocity,C_motor_velocity,D_motor_velocity]
        
while True:
        [GYRz,A_motor_velocity,B_motor_velocity,C_motor_velocity,D_motor_velocity]=roll_control()
        print "GYRz: %3.2f, A: %3.2f, B: %3.2f, C: %3.2f, D: %3.2f" %(GYRz,A_motor_velocity,B_motor_velocity,C_motor_velocity,D_motor_velocity)

        
