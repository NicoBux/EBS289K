function [xp,yp,thetap] = kinematic_ss(v,w,r,l,dt,sl,sr,d,x,y,theta)

% Unicycle Kinematic Model with Slipping and Skidding 
% OUTPUTS: Vehicle pose [x prime, y prime, and theta prime]
% INPUTS: Vehicle speed (v), angular speed (w), wheel radius (r), width
% (l), integration time (dt), left and right slips (sl, sr), angle delta of
% slipe (d), actual pose (x, y, theta)

Vl = (2*v-w*l)/2; %Left wheel velocity
Vr = w*l+Vl; %Right wheel velocity
wr = Vr/r; %Right wheel angular velocity
wl = Vl/r; %Left wheel angular velocity
vl = (1-sl)*r*wl; %Left wheel velocity given slip
vr = (1-sr)*r*wr; %Right wheel velocity given slip
VL = (vl+vr)/2; %Vehicle longitudinal velocity
w_2 = (vr-vl)/l; %Vehicle angular velocity
Vy = tan(d)*VL; %Skid lateral velocity

xp = x+dt*(VL*cos(theta)-Vy*sin(theta));
yp = y+dt*(VL*sin(theta)+Vy*cos(theta));
thetap = theta+w_2*dt;