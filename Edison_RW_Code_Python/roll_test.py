import smbus
import mraa 
import time 
import math
import numpy as np
import IMU
RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
Apwm=Bpwm=Cpwm=Dpwm=Adir=Bdir=Cdir=Ddir=mode=0 
output = [Apwm,Bpwm,Cpwm,Dpwm,Adir,Bdir,Cdir,Ddir,mode] 
Pin = [14,20,0,21,36,48,47,32,46] #PWM for GP13,GP12,GP182,GP183 Gpio for GP14,GP15,GP49,GP46,GP47

for x in xrange(0,4):
	output[x] = mraa.Pwm(Pin[x])
	output[x].period_us(700)
	output[x].enable(True)
	print x, output[x]
for x in xrange(4,9):
	output[x] = mraa.Gpio(Pin[x])
	output[x].dir(mraa.DIR_OUT)
	print x,output[x]

output[8].write(1) #Set Mode to high

def rampup():
        x=0
        while x <= 1:
                output[0].write(x)
                output[1].write(x)
                output[2].write(x)
                output[3].write(x)
                time.sleep(.1)
                x+=.05
		print x        
def rampdown():
        x=1
        while x >= 0:
                output[0].write(x)
                output[1].write(x)
                output[2].write(x)
                output[3].write(x)
                time.sleep(.1)
                x-=.05
		print x

while True:
        output[4].write(1)
        output[5].write(1)
        output[6].write(1)
        output[7].write(1)
	print output[4]
        rampup()
        rampdown()
        output[4].write(0)
        output[5].write(0)
        output[6].write(0)
        output[7].write(0)
        rampup()
        rampdown()

        
