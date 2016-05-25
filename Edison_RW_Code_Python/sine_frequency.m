clear all
close all
frequency = 10; #frequency in Hz
t = linspace(0,3,1000);
out = sin(frequency*t*pi()*2);
figure(1)
plot(t,out)

