# CubeSAT-Reaction-Wheel

This is the RCS system for oresat. It involves 4 moment gyros that are controlled by an Intel Edison. Attached to the Edison are 4 Sparkfun blocks: [Base](https://www.sparkfun.com/products/13045), [9 Degree of Freedom](https://www.sparkfun.com/products/13033), [GPIO](https://www.sparkfun.com/products/13038), and [Battery](https://www.sparkfun.com/products/13037).

All controller programming is done using python2.7 with some added modules. The modules being used are numpy, scipy, smbus, and mraa.

smbus is used for i2c communication to the IMU, mraa is used to access the GPIO block.

The four motors are controlled using two Pololu DRV8835 motor drivers using pwm inputs, and are powered by a 7.4V LiPo battery.

