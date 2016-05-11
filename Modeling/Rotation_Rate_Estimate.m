close all
clear
t = [-50:0.1:50];
h = 400.0e+03;
v = 7.66e+03;
d = v.*t;
theta = atand(d./h);
theta_dot = diff(theta)./diff(t);
figure(1)
plotyy(t,d,t(1:end-1),theta_dot)
