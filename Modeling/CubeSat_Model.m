# CubeSat Reaction Wheel Model
pkg load control
clear all
close all
n=1;
Ic_xx = 2.45; Ic_yy = 2.49; Ic_zz = 2.52; #principal inertia values (lb*in^2)
I_rw = 0.0198; #rotational inertia of the reaction wheels (lb*in^2)
C = 0; #cost function value
b_a = 0.01; b_b = 0.01; b_c = 0.01; #damping coefficients
g_a = 0.001; g_b = 0.001; g_c = 0.001; #spring constants 
TF_cube_coeff_a = [Ic_xx/(sind(45)*I_rw), Ic_yy/(cosd(45)*I_rw), Ic_zz/(cosd(53)*I_rw)];
TF_cube_coeff_b = [b_a/(sind(45)*I_rw), b_b/(cosd(45)*I_rw), b_c/(cosd(53)*I_rw)];
TF_cube_coeff_c = [g_a/(sind(45)*I_rw), g_b/(cosd(45)*I_rw), g_c/(cosd(53)*I_rw)];
mK_a = 0.5; mK_b = 0.05; mK_c = 0.06; mK_d = 0.01;
c_a = 0.3535533905932737; c_b = 0.3535533905932737; c_c = 0.415410035280621;
t = 0:0.02:15;

s = tf('s');
#Transfer functions are 
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

#x step with motor velocity
y_x=0;y_y=0;y_z=0;
c_x=0;c_y=0;c_z=0;
[c_x,t,x]=step(cube_sys_x,t);
[y_x,t,x]=step(motor_TF,t);
omega_a = c_a*y_x + c_b*y_y + c_c*y_z;
omega_b = c_a*y_x - c_b*y_y + c_c*y_z;
omega_c = -c_a*y_x - c_b*y_y + c_c*y_z;
omega_d = -c_a*y_x + c_b*y_y + c_c*y_z;
figure(1);
hold on;
subplot(3,2,1)
plot(t,y_x);
xlabel('seconds','FontSize',12);
ylabel("Cube Angular Velocity\n(degrees/s)",'FontSize',12);
title('step response (angular velocity) of x-axis','FontSize',12)
legend('cube-x')
subplot(3,2,2)
plot(t,omega_a,'k',t,omega_b,'r',t,omega_c,'g',t,omega_d,'b')
xlabel('seconds','FontSize',12);
ylabel("Motor Angular Velocity\n(degrees/s)",'FontSize',12);
title('Motor velocities','FontSize',12)
legend('Motor A','Motor B','Motor C','Motor D')
#y step with motor velocity
y_x=0;y_y=0;y_z=0;
c_x=0;c_y=0;c_z=0;
[c_y,t,x]=step(cube_sys_y,t);
[y_y,t,x]=step(motor_TF,t);
omega_a = c_a*y_x + c_b*y_y + c_c*y_z;
omega_b = c_a*y_x - c_b*y_y + c_c*y_z;
omega_c = -c_a*y_x - c_b*y_y + c_c*y_z;
omega_d = -c_a*y_x + c_b*y_y + c_c*y_z;
subplot(3,2,3)
plot(t,y_y);
xlabel('seconds','FontSize',12);
ylabel("Cube Angular Velocity\n(degrees/s)",'FontSize',12);
title('step response (angular velocity) of y-axis','FontSize',12)
legend('cube-y')
subplot(3,2,4)
plot(t,omega_a,'k',t,omega_b,'r',t,omega_c,'g',t,omega_d,'b')
xlabel('seconds','FontSize',12);
ylabel("Motor Angular Velocity\n(degrees/s)",'FontSize',12);
title('motor velocities','FontSize',12)
legend('Motor A','Motor B','Motor C','Motor D')
#y step with motor velocity
y_x=0;y_y=0;y_z=0;
c_x=0;c_y=0;c_z=0;
[c_z,t,x]=step(cube_sys_z,t);
[y_z,t,x]=step(motor_TF,t);
omega_a = c_a*y_x + c_b*y_y + c_c*y_z;
omega_b = c_a*y_x - c_b*y_y + c_c*y_z;
omega_c = -c_a*y_x - c_b*y_y + c_c*y_z;
omega_d = -c_a*y_x + c_b*y_y + c_c*y_z;
subplot(3,2,5)
plot(t,y_z);
xlabel('seconds','FontSize',12);
ylabel('Cube (degrees/s)','FontSize',12);
title('step response (angular velocity) of z-axis')
legend('cube-z')
subplot(3,2,6)
plot(t,omega_a,'k',t,omega_b,'r',t,omega_c,'g',t,omega_d,'b')
xlabel('seconds','FontSize',12);
ylabel('Motors (degrees/s)','FontSize',12);
title('Motor velocities','FontSize',12)
legend('Motor A','Motor B','Motor C','Motor D')
hold off;
#vector step (i+j+k) with motor velocity
y_x=0;y_y=0;y_z=0;
c_x=0;c_y=0;c_z=0;
sx = 2; sy = 15; sz = -8;
[c_x,t,x]=step(cube_sys_x*sx,t);
[c_y,t,x]=step(cube_sys_y*sy,t);
[c_z,t,x]=step(cube_sys_z*sz,t);
[y_x,t,x]=step(motor_TF*sx,t);
[y_y,t,x]=step(motor_TF*sy,t);
[y_z,t,x]=step(motor_TF*sz,t);
omega_a = c_a*y_x + c_b*y_y + c_c*y_z;
omega_b = c_a*y_x - c_b*y_y + c_c*y_z;
omega_c = -c_a*y_x - c_b*y_y + c_c*y_z;
omega_d = -c_a*y_x + c_b*y_y + c_c*y_z;
figure(2);
hold on;
subplot(1,2,1)
plot(t,c_x,'k',t,c_y,'r',t,c_z,'g');
xlabel('seconds','FontSize',12);
ylabel('Cube (degrees/s)','FontSize',12);
title('vector step response(3i+5j+2k)','FontSize',12)
legend('cube-x','cube-y','cube-z')
subplot(1,2,2)
plot(t,omega_a,'k',t,omega_b,'r',t,omega_c,'g',t,omega_d,'b')
xlabel('seconds','FontSize',12);
ylabel('Motors (degrees/s)','FontSize',12);
title('motor velocity','FontSize',12)
legend('Motor A','Motor B','Motor C','Motor D')
hold off;
#recalculate cardinal input velocity
in_x = sind(45)*(omega_a+omega_b-omega_c-omega_d);
in_y = cosd(45)*(omega_a-omega_b-omega_c+omega_d);
in_z = cosd(53)*(omega_a+omega_b+omega_c+omega_d);
figure(3);
plot(t,in_x,'k',t,in_y,'r',t,in_z,'g')
legend('in-x','in-y','in-z')

y_x =4, y_y = -23, y_z = 45
omega_a = c_a*y_x + c_b*y_y + c_c*y_z 
omega_b = c_a*y_x - c_b*y_y + c_c*y_z 
omega_c = -c_a*y_x - c_b*y_y + c_c*y_z 
omega_d = -c_a*y_x + c_b*y_y + c_c*y_z 
sind(45)*(omega_a+omega_b-omega_c-omega_d)
cosd(45)*(omega_a-omega_b-omega_c+omega_d)
cosd(53)*(omega_a+omega_b+omega_c+omega_d)
bode(motor_TF*cube_TF_x)
margin(motor_TF*cube_TF_x)
#PI controller model
#increase phase margin
#pick 1 rad/s, yields:

omega_g = .02;
G_omega_g = abs(15);
Kp_x = 10^(G_omega_g/20);
Ki_x = omega_g/10*Kp_x;
Gc_x = Kp_x + (Ki_x/s)

#open loop response
figure(4);

bode(cube_sys_x*Gc_x)
margin(cube_sys_x*Gc_x)
CL_cube_sys_x = feedback(cube_sys_x,Gc_x);
[y_cl_x,t,x]=step(CL_cube_sys_x,t);
figure(5);
plot(t,y_cl_x)


#equations for motor speed
#alpha_a = c_a*alpha_err_x + c_b*alpha_err_y + c_c*alpha_err_z - sqrt(c_d*C-c_e*alpha_err_x^2-c_f*alpha_err_y^2-c_g*alpha_err_z^2)
#alpha_b = c_a*alpha_err_x - c_b*alpha_err_y + c_c*alpha_err_z + sqrt(c_d*C-c_e*alpha_err_x^2-c_f*alpha_err_y^2-c_g*alpha_err_z^2)
#alpha_c = -c_a*alpha_err_x - c_b*alpha_err_y + c_c*alpha_err_z - sqrt(c_d*C-c_e*alpha_err_x^2-c_f*alpha_err_y^2-c_g*alpha_err_z^2)
#alpha_d = -c_a*alpha_err_x + c_b*alpha_err_y + c_c*alpha_err_z + sqrt(c_d*C-c_e*alpha_err_x^2-c_f*alpha_err_y^2-c_g*alpha_err_z^2)


