# CubeSat Reaction Wheel Model
pkg load control
close all
Ic_xx = 2.45; Ic_yy = 2.49; Ic_zz = 2.52; #principal inertia values (lb*in^2)
I_rw = 0.0198; #rotational inertia of the reaction wheels (lb*in^2)
C = 1; #cost function value
b_a = 0.01; b_b = 0.01; b_c = 0.01; #damping coefficients
g_a = 0.001; g_b = 0.001; g_c = 0.001; #spring constants 
TF_cube_coeff_a = [Ic_xx/(sind(45)*I_rw), Ic_yy/(cosd(45)*I_rw), Ic_zz/(cosd(53)*I_rw)];
TF_cube_coeff_b = [b_a/(sind(45)*I_rw), b_b/(cosd(45)*I_rw), b_c/(cosd(53)*I_rw)];
TF_cube_coeff_c = [g_a/(sind(45)*I_rw), g_b/(cosd(45)*I_rw), g_c/(cosd(53)*I_rw)];
mK_a = 0.01; mK_b = 0.005; mK_c = 0.06; mK_d = 0.1;
c_a = 0.3535533905932737; c_b = 0.3535533905932737; c_c = 0.415410035280621; c_d = 0.25; c_e = 0.125; c_f = 0.125; c_g = 0.1725654974118469;
t = 0:0.02:5;

s = tf('s');
#Transfer functions are 
motor_TF = tf([mK_a], [mK_b,mK_c,mK_d]);
num_cube_x = [0,360/(2*pi),0];
num_cube_y = [0,360/(2*pi),0];
num_cube_z = [0,360/(2*pi),0];
den_cube_x = [TF_cube_coeff_a(1),TF_cube_coeff_b(1),TF_cube_coeff_c(1)];
den_cube_y = [TF_cube_coeff_a(2),TF_cube_coeff_b(2),TF_cube_coeff_c(2)];
den_cube_z = [TF_cube_coeff_a(3),TF_cube_coeff_b(3),TF_cube_coeff_c(3)];
cube_TF_x = tf(num_cube_x, den_cube_x);
cube_TF_y = tf(num_cube_y, den_cube_y);
cube_TF_z = tf(num_cube_z, den_cube_z);

cube_sys_x = motor_TF*cube_TF_x*s;
cube_sys_y = motor_TF*cube_TF_y*s;
cube_sys_z = motor_TF*cube_TF_z*s;

[y_x,t,x]=step(cube_sys_x,t);
[y_y,t,x]=step(cube_sys_y,t);
[y_z,t,x]=step(cube_sys_z,t);
figure(1)
plot(t,y_x,'k',t,y_y,'r',t,y_z,'g')
xlabel('seconds');
ylabel('degrees/s');
title('step response (angular velocity) of cardinal axes')
legend('x','y','z')
#x step with motor velocity
y_x=y_y=y_z=0;
[y_x,t,x]=step(cube_sys_x,t);
omega_a = c_a*y_x + c_b*y_y + c_c*y_z - sqrt(c_d*C-c_e*y_x.^2-c_f*y_y.^2-c_g*y_z.^2);
omega_b = c_a*y_x - c_b*y_y + c_c*y_z + sqrt(c_d*C-c_e*y_x.^2-c_f*y_y.^2-c_g*y_z.^2);
omega_c = -c_a*y_x - c_b*y_y + c_c*y_z - sqrt(c_d*C-c_e*y_x.^2-c_f*y_y.^2-c_g*y_z.^2);
omega_d = -c_a*y_x + c_b*y_y + c_c*y_z + sqrt(c_d*C-c_e*y_x.^2-c_f*y_y.^2-c_g*y_z.^2);
figure(2)
hold on;
ax =plotyy (t,y_x,t,omega_a);

xlabel('seconds');
ylabel(ax(1),'Cube (degrees/s)');
ylabel(ax(2),'Motors (degrees/s)');
title('step response (angular velocity) of x-axis and motor velocities')
legend('cube-x','Motor A','Motor B','Motor C','Motor D')
#plot(t,omega_b,'g',t,omega_c,'b',t,omega_d,'y')
hold off;




#PI controller model
#to increase phase margin, the crossover frequency must move left
#pick 0.01 rad/s, yields:
#{
omega_g = 0.01;
G_omega_g = abs(-18);
Kp_x = 10^(-G_omega_g/20);
Ki_x = omega_g/10*Kp_x;
Gc_x = Kp_x + (Ki_x/s)

#open loop response
figure(3)
bode(cube_TF_x*Gc_x)
margin(cube_TF_x*Gc_x)
CL_cube_TF_x = feedback(cube_TF_x*Gc_x,1);
[y_cl_x,t,x]=step(CL_cube_TF_x*360/(2*pi),t);
figure(4)
plot(t,y_cl_x)
#}

#equations for motor speed
#alpha_a = c_a*alpha_err_x + c_b*alpha_err_y + c_c*alpha_err_z - sqrt(c_d*C-c_e*alpha_err_x^2-c_f*alpha_err_y^2-c_g*alpha_err_z^2)
#alpha_b = c_a*alpha_err_x - c_b*alpha_err_y + c_c*alpha_err_z + sqrt(c_d*C-c_e*alpha_err_x^2-c_f*alpha_err_y^2-c_g*alpha_err_z^2)
#alpha_c = -c_a*alpha_err_x - c_b*alpha_err_y + c_c*alpha_err_z - sqrt(c_d*C-c_e*alpha_err_x^2-c_f*alpha_err_y^2-c_g*alpha_err_z^2)
#alpha_d = -c_a*alpha_err_x + c_b*alpha_err_y + c_c*alpha_err_z + sqrt(c_d*C-c_e*alpha_err_x^2-c_f*alpha_err_y^2-c_g*alpha_err_z^2)


