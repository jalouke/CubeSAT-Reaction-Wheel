clear all
close all

Data = csvread("CubeSat_Drop_4-26.csv");
Data = Data(2:end,:);
time = Data(:,1);
time = time.-time(1);
ACCz = Data(:,4);
GYRz = Data(:,7);
Motor_Velocity = Data(:,12);

figure(1)
plotyy(time,GYRz,[time,time],[ACCz,Motor_Velocity])
print -dsvg Cube_drop.svg

M_Vel = Motor_Velocity(216:end);
Gyr = GYRz(216:end);
t = time(216:end);

figure(2)
[AX,H1,H2] = plotyy(t,Gyr,t,M_Vel)
AX(1) = ylim([-50,50])
print -djpeg m_vel_vs_cube_vel.jpg;