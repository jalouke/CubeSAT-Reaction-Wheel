clear all
close all
pkg load control
pkg load signal

R = 680; #ohm
C = 1*10^(-6); #Farads

num_RC = 1;
den_RC = [R*C 1];

RC_filt = tf(num_RC, den_RC);

figure(1)
bode(RC_filt)