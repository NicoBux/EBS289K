function [xp,yp,thetap] = kinematic_ss(velocity,omega,radius,width,int_time,left_slip,right_slip,angle_delta,x,y,theta)

% Unicycle Kinematic Model with Slipping and Skidding 
% OUTPUTS: Vehicle pose [x prime, y prime, and theta prime]
% INPUTS: Vehicle speed (v), angular speed (w), wheel radius (r), width
% (l), integration time (dt), left and right slips (sl, sr), angle delta of
% slipe (d), actual pose (x, y, theta)

Vl = (2*velocity-omega*width)/2; %Left wheel velocity
Vr = omega*width+Vl; %Right wheel velocity
wr = Vr/radius; %Right wheel angular velocity
wl = Vl/radius; %Left wheel angular velocity
vl = (1-left_slip)*radius*wl; %Left wheel velocity given slip
vr = (1-right_slip)*radius*wr; %Right wheel velocity given slip
VL = (vl+vr)/2; %Vehicle longitudinal velocity
w_2 = (vr-vl)/width; %Vehicle angular velocity
Vy = tan(angle_delta)*VL; %Skid lateral velocity

xp = x+int_time*(VL*cos(theta)-Vy*sin(theta));
yp = y+int_time*(VL*sin(theta)+Vy*cos(theta));
thetap = theta+w_2*int_time;