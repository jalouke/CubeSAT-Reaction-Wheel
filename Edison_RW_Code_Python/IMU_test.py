#import smbus
import time
import math
import sys
import IMU
RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
while True:
        a = time.time()
        [ACCx,ACCy,ACCz,GYRx,GYRy,GYRz,MAGx,MAGy,MAGz] = IMU.read()


        AccXangle =  (math.atan2(ACCy,ACCz)+M_PI)*RAD_TO_DEG
        AccYangle =  (math.atan2(ACCz,ACCx)+M_PI/2)*RAD_TO_DEG	

        #Calculate heading
        heading = 180 * math.atan2(MAGy,MAGx)/M_PI

        if heading < 0:
                heading += 360

        b = time.time()
        LoopTime = b - a
#       print ("\033[1;34;40mAcceleration Values:")
#       print ("\033[1;34;40mACCX %5.4f, ACCy %5.4f, ACCz %5.4f" % (ACCx, ACCy, ACCz))
#       print ("\033[1;31;40mGyro Values:")
#       print ("\033[1;31;40mGYRx %5.2f, GYRy %5.2f, GYRz %5.2f" % (GYRx, GYRy, GYRz))
#       print ("\033[1;35;40mMagnetometer values:")
#       print ("\033[1;35;40mMAGx %5.2f, MAGy %5.2f, MAGz %5.2f" % (MAGx, MAGy, MAGz))	
        print ("\033[1;34;40mAcceleration angles:")
        print ("\033[1;34;40m AccXangle= %5.2f, AccYangle= %5.2f" % (AccXangle,AccYangle))
        print ("\033[1;34;40m Heading angles:")
        print ("\033[1;34;40m Heading= %5.2f") % (heading)
        time.sleep(0.3)

