clear all
close all
graphics_toolkit("gnuplot")
t=[1:.01:3];
t_i = [0:.02:2];
figure(4)
subplot(2,1,1)
plotyy(t,sin(t),t,cos(t))
%ylabel(hAy(1),'Cube Angular Velocity (rad/s)') % left y-axis
%ylabel(hAy(2),'Cube Angular Position (rad)') % right y-axis
%title('Cube Response')
%xlabel('Time (sec)')
%figure(5)
subplot(2,1,2)
plotyy(t_i,sin(t_i)+1,t_i,sin(pi*t_i)+1)
%ylabel(hAx(1),'Motor-A Angular Velocity (rad/s)') % left y-axis
%ylabel(hAx(2),'Motor Torque (mNm)') % right y-axis
%title('Motor A Response')
%xlabel('Time (sec)')