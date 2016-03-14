# CubeSat Reaction Wheel Model
pkg load control
clear all
close all
n=1;
Ic_xx = 2.5; Ic_yy = 2.5; Ic_zz = 2.5; #principal inertia values (lb*in^2)
I_rw = 0.0198; #rotational inertia of the reaction wheels (lb*in^2)
b_a = 1*10^-6; b_b = 1*10^-6; b_c = 1*10^-6; #damping coefficients
#Transfer Function Coefficients
TF_cube_coeff_a = [Ic_xx/I_rw, Ic_yy/I_rw, Ic_zz/I_rw];
TF_cube_coeff_b = [b_a, b_b, b_c];


#Cardinal to Motor reference frame transform coefficients
c_a = 0.3535533905932737; c_b = 0.3535533905932737; c_c = 0.415410035280621;
mech_time_constant = 48.9*10^-3 % seconds
L = .32*10^-3; %Inductance (H)
Kt = 10.4*10^-3; %Torque constant (N-m/A)
R = 1.01; %Resistance (Ohm)
I = 52.3*10^-7+(2.9*10^-4*I_rw); %Rotor inertia (kg-m2)
b = 2.6*10^-6; %Viscous Damping (N-m-s)
Kbac = (R*I)/(mech_time_constant*Kt); %Back emf constant (V/(rad/s))

num_motor = Kt;
den_motor = [I*L (b*L + I*R) (R*b + Kt*Kbac)];
sys_motor = tf(num_motor,den_motor)
s = tf('s');
#Motor Transfer Function
motor_TF_A = sys_motor;
motor_TF_B = sys_motor;
motor_TF_C = sys_motor;
motor_TF_D = sys_motor;

#Cube Dynamics
num_cube_Ax = [sind(45),0];
num_cube_Ay = [cosd(45),0];
num_cube_Az = [cosd(53),0];
den_cube_Ax = [TF_cube_coeff_a(1),TF_cube_coeff_b(1)];
den_cube_Ay = [TF_cube_coeff_a(2),TF_cube_coeff_b(2)];
den_cube_Az = [TF_cube_coeff_a(3),TF_cube_coeff_b(3)];
cube_TF_Ax = tf(num_cube_Ax, den_cube_Ax);
cube_TF_Ay = tf(num_cube_Ay, den_cube_Ay);
cube_TF_Az = tf(num_cube_Az, den_cube_Az);

num_cube_Bx = [sind(45),0];
num_cube_By = [-cosd(45),0];
num_cube_Bz = [cosd(53),0];
den_cube_Bx = [TF_cube_coeff_a(1),TF_cube_coeff_b(1)];
den_cube_By = [TF_cube_coeff_a(2),TF_cube_coeff_b(2)];
den_cube_Bz = [TF_cube_coeff_a(3),TF_cube_coeff_b(3)];
cube_TF_Bx = tf(num_cube_Bx, den_cube_Bx);
cube_TF_By = tf(num_cube_By, den_cube_By);
cube_TF_Bz = tf(num_cube_Bz, den_cube_Bz);

num_cube_Cx = [-sind(45),0];
num_cube_Cy = [-cosd(45),0];
num_cube_Cz = [cosd(53),0];
den_cube_Cx = [TF_cube_coeff_a(1),TF_cube_coeff_b(1)];
den_cube_Cy = [TF_cube_coeff_a(2),TF_cube_coeff_b(2)];
den_cube_Cz = [TF_cube_coeff_a(3),TF_cube_coeff_b(3)];
cube_TF_Cx = tf(num_cube_Cx, den_cube_Cx);
cube_TF_Cy = tf(num_cube_Cy, den_cube_Cy);
cube_TF_Cz = tf(num_cube_Cz, den_cube_Cz);

num_cube_Dx = [-sind(45),0];
num_cube_Dy = [cosd(45),0];
num_cube_Dz = [cosd(53),0];
den_cube_Dx = [TF_cube_coeff_a(1),TF_cube_coeff_b(1)];
den_cube_Dy = [TF_cube_coeff_a(2),TF_cube_coeff_b(2)];
den_cube_Dz = [TF_cube_coeff_a(3),TF_cube_coeff_b(3)];
cube_TF_Dx = tf(num_cube_Dx, den_cube_Dx);
cube_TF_Dy = tf(num_cube_Dy, den_cube_Dy);
cube_TF_Dz = tf(num_cube_Dz, den_cube_Dz);


#Step X-axis only
X_i = 0; Y_i = 0; Z_i = 0;
%X_i = 100*ones(1,length(t));
[X_i,t] = gensig('square',2,1.25,0.001);
X_i = (-X_i+1)*18;
mA_c = c_a*  X_i + c_b*  Y_i + c_c*  Z_i;
mB_c = c_a*  X_i - c_b*  Y_i + c_c*  Z_i;
mC_c = -c_a*  X_i - c_b*  Y_i + c_c*  Z_i;
mD_c = -c_a*  X_i + c_b*  Y_i + c_c*  Z_i;
#Motor response
[y_A_X,t,x] = lsim(motor_TF_A,mA_c,t);
[y_B_X,t,x] = lsim(motor_TF_B,mB_c,t);
[y_C_X,t,x] = lsim(motor_TF_C,mC_c,t);
[y_D_X,t,x] = lsim(motor_TF_D,mD_c,t);

#Cube response
[y_Ax,t,x] = lsim(cube_TF_Ax,y_A_X,t);
[y_Ay,t,x] = lsim(cube_TF_Ay,y_A_X,t);
[y_Az,t,x] = lsim(cube_TF_Az,y_A_X,t);
[y_Bx,t,x] = lsim(cube_TF_Bx,y_B_X,t);
[y_By,t,x] = lsim(cube_TF_By,y_B_X,t);
[y_Bz,t,x] = lsim(cube_TF_Bz,y_B_X,t);
[y_Cx,t,x] = lsim(cube_TF_Cx,y_C_X,t);
[y_Cy,t,x] = lsim(cube_TF_Cy,y_C_X,t);
[y_Cz,t,x] = lsim(cube_TF_Cz,y_C_X,t);
[y_Dx,t,x] = lsim(cube_TF_Dx,y_D_X,t);
[y_Dy,t,x] = lsim(cube_TF_Dy,y_D_X,t);
[y_Dz,t,x] = lsim(cube_TF_Dz,y_D_X,t);

y_X = y_Ax+y_Bx+y_Cx+y_Dx;
y_Y = y_Ay+y_By+y_Cy+y_Dy;
y_Z = y_Az+y_Bz+y_Cz+y_Dz;



yp = diff(y_A_X)./diff(t);
[y_pos,t,x] = lsim(1/s,y_X,t);

figure(1)
subplot(2,1,2)
[hAx,hLine1,hLine2] = plotyy([t],[y_A_X],t(1:end-1),yp*(I)*10^3)
ylabel(hAx(1),'Motor-A Angular Velocity (rad/s)') % left y-axis
ylabel(hAx(2),'Motor Torque (mNm)') % right y-axis
title('Motor A Response')
xlabel('Time (sec)')

subplot(2,1,1)
[hAx,hLine1,hLine2] = plotyy([t,t,t],[y_X,y_Y,y_Z],t,y_pos)
ylabel(hAx(1),'Cube Angular Velocity (rad/s)') % left y-axis
ylabel(hAx(2),'Cube Angular Position (rad)') % right y-axis
title('Cube Response')
xlabel('Time (sec)')

t = 0:0.01:1;
#Step Y-axis only
X_i = 0; Y_i = 0; Z_i = 0;
Y_i = ones(1,length(t));
mA_c = c_a*  X_i + c_b*  Y_i + c_c*  Z_i;
mB_c = c_a*  X_i - c_b*  Y_i + c_c*  Z_i;
mC_c = -c_a*  X_i - c_b*  Y_i + c_c*  Z_i;
mD_c = -c_a*  X_i + c_b*  Y_i + c_c*  Z_i;
#Motor response
[y_A_X,t,x] = lsim(motor_TF_A,mA_c,t);
[y_B_X,t,x] = lsim(motor_TF_B,mB_c,t);
[y_C_X,t,x] = lsim(motor_TF_C,mC_c,t);
[y_D_X,t,x] = lsim(motor_TF_D,mD_c,t);
figure(2)
subplot(2,1,2)
plot(t,y_A_X,t,y_B_X,t,y_C_X,t,y_D_X)
#Cube response
[y_Ax,t,x] = lsim(cube_TF_Ax,y_A_X,t);
[y_Ay,t,x] = lsim(cube_TF_Ay,y_A_X,t);
[y_Az,t,x] = lsim(cube_TF_Az,y_A_X,t);
[y_Bx,t,x] = lsim(cube_TF_Bx,y_B_X,t);
[y_By,t,x] = lsim(cube_TF_By,y_B_X,t);
[y_Bz,t,x] = lsim(cube_TF_Bz,y_B_X,t);
[y_Cx,t,x] = lsim(cube_TF_Cx,y_C_X,t);
[y_Cy,t,x] = lsim(cube_TF_Cy,y_C_X,t);
[y_Cz,t,x] = lsim(cube_TF_Cz,y_C_X,t);
[y_Dx,t,x] = lsim(cube_TF_Dx,y_D_X,t);
[y_Dy,t,x] = lsim(cube_TF_Dy,y_D_X,t);
[y_Dz,t,x] = lsim(cube_TF_Dz,y_D_X,t);

y_X = y_Ax+y_Bx+y_Cx+y_Dx;
y_Y = y_Ay+y_By+y_Cy+y_Dy;
y_Z = y_Az+y_Bz+y_Cz+y_Dz;

subplot(2,1,1)
plot(t,y_X,t,y_Y,t,y_Z)

#Step z-axis only
X_i = 0; Y_i = 0; Z_i = 0;
Z_i = ones(1,length(t));
mA_c = c_a*  X_i + c_b*  Y_i + c_c*  Z_i;
mB_c = c_a*  X_i - c_b*  Y_i + c_c*  Z_i;
mC_c = -c_a*  X_i - c_b*  Y_i + c_c*  Z_i;
mD_c = -c_a*  X_i + c_b*  Y_i + c_c*  Z_i;
#Motor response
[y_A_X,t,x] = lsim(motor_TF_A,mA_c,t);
[y_B_X,t,x] = lsim(motor_TF_B,mB_c,t);
[y_C_X,t,x] = lsim(motor_TF_C,mC_c,t);
[y_D_X,t,x] = lsim(motor_TF_D,mD_c,t);
figure(3)
subplot(2,1,2)
plot(t,y_A_X,t,y_B_X,t,y_C_X,t,y_D_X)
#Cube response
[y_Ax,t,x] = lsim(cube_TF_Ax,y_A_X,t);
[y_Ay,t,x] = lsim(cube_TF_Ay,y_A_X,t);
[y_Az,t,x] = lsim(cube_TF_Az,y_A_X,t);
[y_Bx,t,x] = lsim(cube_TF_Bx,y_B_X,t);
[y_By,t,x] = lsim(cube_TF_By,y_B_X,t);
[y_Bz,t,x] = lsim(cube_TF_Bz,y_B_X,t);
[y_Cx,t,x] = lsim(cube_TF_Cx,y_C_X,t);
[y_Cy,t,x] = lsim(cube_TF_Cy,y_C_X,t);
[y_Cz,t,x] = lsim(cube_TF_Cz,y_C_X,t);
[y_Dx,t,x] = lsim(cube_TF_Dx,y_D_X,t);
[y_Dy,t,x] = lsim(cube_TF_Dy,y_D_X,t);
[y_Dz,t,x] = lsim(cube_TF_Dz,y_D_X,t);

y_X = y_Ax+y_Bx+y_Cx+y_Dx;
y_Y = y_Ay+y_By+y_Cy+y_Dy;
y_Z = y_Az+y_Bz+y_Cz+y_Dz;

subplot(2,1,1)
plot(t,y_X,t,y_Y,t,y_Z)

#Step random direction

X_i = 0; Y_i = 0; Z_i = 0;
X_i = ones(1,length(t))*-25;
Y_i = ones(1,length(t))*70;
Z_i = ones(1,length(t))*10;
mA_c = c_a*  X_i + c_b*  Y_i + c_c*  Z_i;
mB_c = c_a*  X_i - c_b*  Y_i + c_c*  Z_i;
mC_c = -c_a*  X_i - c_b*  Y_i + c_c*  Z_i;
mD_c = -c_a*  X_i + c_b*  Y_i + c_c*  Z_i;
#Motor response
[y_A_X,t,x] = lsim(motor_TF_A,mA_c,t);
[y_B_X,t,x] = lsim(motor_TF_B,mB_c,t);
[y_C_X,t,x] = lsim(motor_TF_C,mC_c,t);
[y_D_X,t,x] = lsim(motor_TF_D,mD_c,t);
figure(4)
subplot(2,1,2)
plot(t,y_A_X,t,y_B_X,t,y_C_X,t,y_D_X)
#Cube response
[y_Ax,t,x] = lsim(cube_TF_Ax,y_A_X,t);
[y_Ay,t,x] = lsim(cube_TF_Ay,y_A_X,t);
[y_Az,t,x] = lsim(cube_TF_Az,y_A_X,t);
[y_Bx,t,x] = lsim(cube_TF_Bx,y_B_X,t);
[y_By,t,x] = lsim(cube_TF_By,y_B_X,t);
[y_Bz,t,x] = lsim(cube_TF_Bz,y_B_X,t);
[y_Cx,t,x] = lsim(cube_TF_Cx,y_C_X,t);
[y_Cy,t,x] = lsim(cube_TF_Cy,y_C_X,t);
[y_Cz,t,x] = lsim(cube_TF_Cz,y_C_X,t);
[y_Dx,t,x] = lsim(cube_TF_Dx,y_D_X,t);
[y_Dy,t,x] = lsim(cube_TF_Dy,y_D_X,t);
[y_Dz,t,x] = lsim(cube_TF_Dz,y_D_X,t);

y_X = y_Ax+y_Bx+y_Cx+y_Dx;
y_Y = y_Ay+y_By+y_Cy+y_Dy;
y_Z = y_Az+y_Bz+y_Cz+y_Dz;

subplot(2,1,1)
plot(t,y_X,t,y_Y,t,y_Z)

[y,t,x] = step(sys_motor);
figure(5)
plot(t,y)