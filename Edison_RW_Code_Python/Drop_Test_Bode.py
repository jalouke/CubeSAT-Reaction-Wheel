import smbus
import mraa 
import time 
import math
import numpy as np
import IMU
import sys

#########################################################
# initializing Variables
Frequency = 1 #frequency of input response in Hz
A_motor_velocity=B_motor_velocity=C_motor_velocity=D_motor_velocity=0
X_velocity=Y_velocity=Z_velocity=0
A=B=C=D=E=F=G=H=I=0
output = ['A','B','C','D','E','F','G','H','I']
Pin = [14,20,0,21,36,48,47,32,46] #PWM for GP13,GP12,GP182,GP183 Gpio for GP14,GP15,GP49,GP46,GP47
pi = 3.1415926
c_a = np.sin(45*pi/180)/2 #transformation coefficient #1
c_b = np.sin(45*pi/180)/2 #transformation coefficient #2
c_c = 1/(np.cos(53*pi/180)*4) #transformation coefficient #3
Card_to_Motor = [[c_a,c_b,c_c];[c_a,-c_b,c_c];[-c_a,-c_b,c_c];[-c_a,c_b,c_c]]; #cubesat reference frame to motor reference frame conversion (3x4)
phase=0
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
def A_motor(A_motor_velocity):
        if A_motor_velocity >= 0:
                 A_motor_dir = 1
                 A_motor_speed = abs(A_motor_velocity)
                 if A_motor_speed > 1:
                         A_motor_speed = 1
        elif A_motor_velocity < 0:
                 A_motor_dir = 0
                 A_motor_speed = abs(A_motor_velocity)
                 if A_motor_speed > 1:
                         A_motor_speed = 1
        Adir.write(A_motor_dir)
        Apwm.write(A_motor_speed)
def B_motor(B_motor_velocity):
        if B_motor_velocity >= 0:
                 B_motor_dir = 1
                 B_motor_speed = abs(B_motor_velocity)
                 if B_motor_speed > 1:
                         B_motor_speed = 1
        elif B_motor_velocity < 0:
                 B_motor_dir = 0
                 B_motor_speed = abs(B_motor_velocity)
                 if B_motor_speed > 1:
                         B_motor_speed = 1
        Bdir.write(B_motor_dir)
        Bpwm.write(B_motor_speed)
def C_motor(C_motor_velocity):
        if C_motor_velocity >= 0:
                 C_motor_dir = 1
                 C_motor_speed = abs(C_motor_velocity)
                 if C_motor_speed > 1:
                         C_motor_speed = 1
        elif C_motor_velocity < 0:
                 C_motor_dir = 0
                 C_motor_speed = abs(C_motor_velocity)
                 if C_motor_speed > 1:
                         C_motor_speed = 1
        Cdir.write(C_motor_dir)
        Cpwm.write(C_motor_speed)
def D_motor(D_motor_velocity):
        if D_motor_velocity >= 0:
                 D_motor_dir = 1
                 D_motor_speed = abs(D_motor_velocity)
                 if D_motor_speed > 1:
                         D_motor_speed = 1
        elif D_motor_velocity < 0:
                 D_motor_dir = 0
                 D_motor_speed = abs(D_motor_velocity)
                 if D_motor_speed > 1:
                         D_motor_speed = 1
        Ddir.write(D_motor_dir)
        Dpwm.write(D_motor_speed)
def shutdown():
        Adir.write(0)
        Bdir.write(0)
        Cdir.write(0)
        Ddir.write(0)
        Apwm.write(0) 
        Bpwm.write(0) 
        Cpwm.write(0) 
        Dpwm.write(0) 
        sys.exit() 
def freq_response(Frequency,timestart):
        t = time.time()-timestart
        Z_velocity = np.sin(Frequency*t*pi*2)
        [A_Motor_velocity,B_Motor_velocity,C_Motor_velocity,D_Motor_velocity]=[X_velocity,Y_velocity,Z_velocity]*Card_to_Motor
        return A_Motor_velocity,B_Motor_velocity,C_Motor_velocity,D_Motor_velocity
filename = time.strftime("%Y-%m-%d_%H-%M-%S")
Data = open(('Drop_Test/CubeSat_Drop').__add__(filename).__add__('.csv'), 'a')
Data.write('Time,ACCz,GYRx,GYRy,GYRz,MAGx,MAGy,MAGz,A_Motor_velocity,B_Motor_velocity,C_Motor_velocity,D_Motor_velocity\n')

while True:
        [ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,MAGx,MAGy,MAGz] = IMU.read()
	timestart = time.time()
	while ACCz > -.8:
                phase = 2
                [A_Motor_velocity,B_Motor_velocity,C_Motor_velocity,D_Motor_velocity] = freq_response(Frequency,timestart)
                A_motor(A_motor_velocity)
                B_motor(B_motor_velocity)
                C_motor(C_motor_velocity)
                D_motor(D_motor_velocity)
                Data.write('%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f,%5.3f\n' % (t,ACCz,GYRx,GYRy,GYRz,MAGx,MAGy,MAGz,A_Motor_velocity,B_Motor_velocity,C_Motor_velocity,D_Motor_velocity))
	if phase = 2 && ACCz <-.9:
                shutdown()
                
