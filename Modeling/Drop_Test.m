clear all
close all

Data = csvread("CubeSat_Drop_4-26.csv");
Data = Data(2:end,:);
time = Data(:,1);
time = time.-time(1);
ACCz = Data(:,4);
GYRz = Data(:,7);
Motor_Velocity = Data(:,12);

Ic_xx = 2.45; Ic_yy = 2.49; Ic_zz = 2.52; #principal inertia values (lb*in^2)
I_rw = 0.0198; #rotational inertia of the reaction wheels (lb*in^2)
C = 0; #cost function value
b_a = 0.01; b_b = 0.01; b_c = 0.01; #damping coefficients
g_a = 0.0; g_b = 0.0; g_c = 0.0; #spring constants 
TF_cube_coeff_a = [Ic_xx/I_rw, Ic_yy/I_rw, Ic_zz/I_rw];
TF_cube_coeff_b = [b_a/I_rw, b_b/I_rw, b_c/I_rw];
TF_cube_coeff_c = [g_a/I_rw, g_b/I_rw, g_c/I_rw];
mK_a = 0.01; mK_b = 0.005; mK_c = 0.06; mK_d = 0.1;
c_a = 0.3535533905932737; c_b = 0.3535533905932737; c_c = 0.415410035280621;

s = tf('s');
%Transfer functions are 
motor_TF = tf([mK_a], [mK_b,mK_c,mK_d]);
num_cube_x = [360/(2*pi),0];
num_cube_y = [360/(2*pi),0];
num_cube_z = [360/(2*pi),0];
den_cube_x = [TF_cube_coeff_a(1),TF_cube_coeff_b(1),TF_cube_coeff_c(1)];
den_cube_y = [TF_cube_coeff_a(2),TF_cube_coeff_b(2),TF_cube_coeff_c(2)];
den_cube_z = [TF_cube_coeff_a(3),TF_cube_coeff_b(3),TF_cube_coeff_c(3)];
cube_TF_x = tf(num_cube_x, den_cube_x);
cube_TF_y = tf(num_cube_y, den_cube_y);
cube_TF_z = tf(num_cube_z, den_cube_z);

cube_sys_x = feedback(motor_TF*cube_TF_x*s);
cube_sys_y = feedback(motor_TF*cube_TF_y*s);
cube_sys_z = feedback(motor_TF*cube_TF_z*s);

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

