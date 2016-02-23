d = 1.5; #in
rpm_rad = 0.104719755 #rad/s / rpm
omega = 22000; #rpm
c = pi*d #circumference
v=(d/2)*omega*rpm_rad #edge velocity
t=(c/12)/v #time per window

