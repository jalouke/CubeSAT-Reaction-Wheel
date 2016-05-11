# CubeSat Reaction Wheel Model
pkg load control
pkg load signal
clear all
close all
graphics_toolkit("qt")
n=1;
Ic_xx = 2.5; Ic_yy = 2.5; Ic_zz = 2.5; #principal inertia values (lb*in^2)
I_rw = 0.01; #rotational inertia of the reaction wheels (lb*in^2)
b_a = 0; b_b = 0; b_c = 0; #damping coefficients
#Transfer Function Coefficients
TF_cube_coeff_a = [Ic_xx/I_rw, Ic_yy/I_rw, Ic_zz/I_rw];
TF_cube_coeff_b = [b_a, b_b, b_c];

%controller gains

Kp = 3; %initial proportional to make margins above zero


#Cardinal to Motor reference frame transform coefficients
c_a = sind(45)/2; c_b = sind(45)/2; c_c = 1/(cosd(53)*4);
mech_time_constant = 48.9*10^-3; % seconds
L = .32*10^-3; %Inductance (H)
Kt = 10.4*10^-3; %Torque constant (N-m/A)
R = 1.01; %Resistance (Ohm)
I = 52.3*10^-7+(2.9*10^-4*I_rw); %Rotor inertia (kg-m2)
b = 2.6*10^-6; %Viscous Damping (N-m-s)
Kbac = (R*I)/(mech_time_constant*Kt); %Back emf constant (V/(rad/s))

num_motor = Kt;
den_motor = [I*L (b*L + I*R) (R*b + Kt*Kbac)];
sys_motor = tf(num_motor,den_motor);
s = tf('s');

#Cube Dynamics
num_cube_Ax = [sind(45),0];
num_cube_Ay = [cosd(45),0];
num_cube_Az = [cosd(53),0];
num_cube_Bx = [sind(45),0];
num_cube_By = [-cosd(45),0];
num_cube_Bz = [cosd(53),0];
num_cube_Cx = [-sind(45),0];
num_cube_Cy = [-cosd(45),0];
num_cube_Cz = [cosd(53),0];
num_cube_Dx = [-sind(45),0];
num_cube_Dy = [cosd(45),0];
num_cube_Dz = [cosd(53),0];
den_cube_x = [TF_cube_coeff_a(1),TF_cube_coeff_b(1)];
den_cube_y = [TF_cube_coeff_a(2),TF_cube_coeff_b(2)];
den_cube_z = [TF_cube_coeff_a(3),TF_cube_coeff_b(3)];
cube_TF_Ax = tf(num_cube_Ax, den_cube_x);
cube_TF_Ay = tf(num_cube_Ay, den_cube_y);
cube_TF_Az = tf(num_cube_Az, den_cube_z);
cube_TF_Bx = tf(num_cube_Bx, den_cube_x);
cube_TF_By = tf(num_cube_By, den_cube_y);
cube_TF_Bz = tf(num_cube_Bz, den_cube_z);
cube_TF_Cx = tf(num_cube_Cx, den_cube_x);
cube_TF_Cy = tf(num_cube_Cy, den_cube_y);
cube_TF_Cz = tf(num_cube_Cz, den_cube_z);
cube_TF_Dx = tf(num_cube_Dx, den_cube_x);
cube_TF_Dy = tf(num_cube_Dy, den_cube_y);
cube_TF_Dz = tf(num_cube_Dz, den_cube_z);

cube_TF = [[cube_TF_Ax,cube_TF_Bx,cube_TF_Cx,cube_TF_Dx];[cube_TF_Ay,cube_TF_By,cube_TF_Cy,cube_TF_Dy] ...
;[cube_TF_Az,cube_TF_Bz,cube_TF_Cz,cube_TF_Dz]];

%Cardinal to motor transform equations
%mA_c = c_a*  X_i + c_b*  Y_i + c_c*  Z_i;
%mB_c = c_a*  X_i - c_b*  Y_i + c_c*  Z_i;
%mC_c = -c_a*  X_i - c_b*  Y_i + c_c*  Z_i;
%mD_c = -c_a*  X_i + c_b*  Y_i + c_c*  Z_i;

Card_to_Motor = sys_motor * [[c_a,c_b,c_c];[c_a,-c_b,c_c];[-c_a,-c_b,c_c];[-c_a,c_b,c_c]];

Full_cube_TF = cube_TF*Card_to_Motor;
Full_cube_TF_CL = feedback(Full_cube_TF);


figure(1)
bode(Full_cube_TF(1,1))
margin(Full_cube_TF(1,1))

[X_i,t_i] = gensig('square',2,1.25,0.001);
Y_i = 0*ones(1,length(t_i))'; Z_i = 0*ones(1,length(t_i))';
X_i = 2*pi .*(-X_i+1);

[y_cube,t,x]=lsim(feedback(Full_cube_TF),[X_i,Y_i,Z_i],t_i);
figure(2)
plot(t,y_cube)

%%PI controller
%%Ki/Kp = 100
Kp_i = 10^(5/20);
Ki = 100*Kp_i;
Gc_PI = Kp+Ki/s;
Gc_PI = eye(3)*Gc_PI;
figure(3)
bode(Full_cube_TF(1,1)*Gc_PI(1,1));
margin(Full_cube_TF(1,1)*Gc_PI(1,1));
[y_Gc,t,x]=lsim(feedback(Full_cube_TF(1,1)*Gc_PI),[X_i,Y_i,Z_i],t_i);
Motor_response = feedback(Gc_PI(1,1)*Card_to_Motor,cube_TF);

[y_vel,t,x] = lsim(Motor_response(1,1),X_i,t_i);
y_acc = diff(y_vel)./diff(t);
y_torque = y_acc*(I)*10^3;
[y_pos,t,x] = lsim(1/s,y_Gc(:,1),t);

figure(4)
hAb = subplot(2,1,1)
[hAy,hLine1,hLine2] = plotyy(t_i,y_Gc(:,1),t_i,y_pos);
ylabel(hAy(1),'Cube Angular Velocity (rad/s)') % left y-axis
ylabel(hAy(2),'Cube Angular Position (rad)') % right y-axis
title('Cube Response')
xlabel('Time (sec)')
%figure(6)
hAa = subplot(2,1,2)
[hAx,hLine3,hLine4] = plotyy(t_i(1:end-1),y_vel(1:end-1),t_i(1:end-1),y_torque);
ylabel(hAx(1),'Motor-A Angular Velocity (rad/s)') % left y-axis
ylabel(hAx(2),'Motor Torque (mNm)') % right y-axis
title('Motor A Response')
xlabel('Time (sec)')

t_x = linspace(0,1,100);
Y_x = ones(1,length(t_x));
[Y_pi,t,x] = lsim(feedback(Full_cube_TF(1,1)*Gc_PI(1,1)),Y_x,t_x);
[Y_ol,t,x] = lsim(Full_cube_TF(1,1),Y_x,t_x);
figure(6)
plot(t,Y_pi,t,Y_ol);
xlabel('time (s)');
ylabel('Cube Angular Velocity (rad/s)');
title('CubeSat Reaction Wheel Velocity Response');
legend('PI controller Response','Open Loop Response');
print -dsvg CubeSat_Reaction_Wheel_Velocity_Response.svg;

Data = csvread("CubeSat_Drop_4-26.csv");
Data = Data(2:end,:);
time = Data(:,1);
time = time.-time(1);
ACCz = Data(:,4);
GYRz = Data(:,7);
Motor_Velocity = Data(:,12);

M_Vel = Motor_Velocity(216:end);
Gyr = GYRz(216:end);
t = time(216:end);

Kp_drop = 195.86;
x0 = [-5,45.712];
[y_drop,ta,x] = lsim(Full_cube_TF(1,1)*Kp_drop,M_Vel,t,x0);
figure(7)
[hAd,hLine5,hLine6] = plotyy(t,y_drop,t,M_Vel);
set(hLine5,'color','b');
hold on
ylim([-50,50]);
plot(t,Gyr,'m');
hold off
ylabel(hAd(1), 'Cube Angular Velocity (deg/s)');
ylabel(hAd(2),'Controller Input');
xlabel('time (s)');
legend('Gyro Rate','Model Output','Controller Input');
title('Drop Test Gyro Data Compared to Model');
print -dpng Gyro_Data_vs_Model.png;


