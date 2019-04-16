%% Homewrok Assignment #1
% Student: Guilherme De Moura Araujo
% Professor: Stavros Vougioukas

%% Part 1a - Drive through a square of 5x5
%INPUTS
D = 5;
v = 1; % linear velocity
w = pi/2; % angular velocity
wi = 2.0; % vehicle width
r = 0.5; % wheel radius
l = 4; % vehicle length
dt = 0.05; % delta time
sl = 0; %left slip
sr = 0; % right sleep
d = deg2rad(0); % delta (angle)
x = 0; % initial x pose
y = 0; % initial y pose
theta = deg2rad(0);
proute = [x,y];
tractor = draw_tractor(wi,l,r); % draw a vehicle element
% initial conditions prior to start the open loop
[oldframe,proute] = move_robot(tractor,proute,x,y,theta,tractor);
xp = x;
yp = y;
thetap = theta;

% Open loop
for i=0:dt:D/v-dt  %First side
    [xp,yp,thetap] = kinematic_ss(v,0,r,wi,dt,sl,sr,0,xp,yp,thetap);
    [oldframe,proute] = move_robot(oldframe,proute,xp,yp,thetap,tractor);
end

for i=0:dt:(pi/2)/w-dt %Vehicle turning
    [xp,yp,thetap] = kinematic_ss(0,w,r,wi,dt,sl,sr,0,xp,yp,thetap);
    [oldframe,proute] = move_robot(oldframe,proute,xp,yp,thetap,tractor);
end

for i=0:dt:D/v-dt  %Second side (notice that now theta = 90)
    [xp,yp,thetap] = kinematic_ss(v,0,r,l,dt,sl,sr,0,xp,yp,thetap);
    [oldframe,proute] = move_robot(oldframe,proute,xp,yp,thetap,tractor);
end

for i=0:dt:(pi/2)/w-dt %Vehicle turning
    [xp,yp,thetap] = kinematic_ss(0,w,r,wi,dt,sl,sr,0,xp,yp,thetap);
    [oldframe,proute] = move_robot(oldframe,proute,xp,yp,thetap,tractor);
end

for i=0:dt:D/v-dt   %Third side (theta = 180)
    [xp,yp,thetap] = kinematic_ss(v,0,r,l,dt,sl,sr,0,xp,yp,thetap);
    [oldframe,proute] = move_robot(oldframe,proute,xp,yp,thetap,tractor);
end

for i=0:dt:(pi/2)/w-dt %Vehicle turning
    [xp,yp,thetap] = kinematic_ss(0,w,r,wi,dt,sl,sr,0,xp,yp,thetap);
    [oldframe,proute] = move_robot(oldframe,proute,xp,yp,thetap,tractor);
end

for i=0:dt:D/v-dt %Third side (theta = 270)
    [xp,yp,thetap] = kinematic_ss(v,0,r,l,dt,sl,sr,0,xp,yp,thetap);
    [oldframe,proute] = move_robot(oldframe,proute,xp,yp,thetap,tractor);
end

%% Part 1b - Drive through a circle of radius 5
%INPUTS
clear all;
clc;
v = 1; % m/s
R = 5; % meters
w = v/R; % radians/s
r = 0.5; % m
l = 4; % m
wi = 2; % m
dt = 0.05; % s
sl = 0; % 0%
sr = 0; % 0%
d = deg2rad(0); % rad
x = 0; % m
y = 0; % m
theta = deg2rad(0); % rad
proute = [x,y];
tractor = draw_tractor(wi,l,r);
[oldframe,proute] = move_robot(tractor,proute,x,y,theta,tractor);
xp = x;
yp = y;
thetap = theta;


for i=0:dt:2*pi/w % Distance = 2pi, t = d/w
    [xp,yp,thetap] = kinematic_ss(v,w,r,l,dt,sl,sr,0,xp,yp,thetap);
    [oldframe,proute] = move_robot(oldframe,proute,xp,yp,thetap,tractor);
end
%% Part 2a - Different parameters
sl = 0.1;
sr = 0.2;
R = 5;
for i=0:dt:2*pi/w % Given that the distance is 2*pi*r and w = 1/5 then t = 10*pi
    [xp,yp,thetap] = kinematic_ss(v,w,r,l,dt,sl,sr,0,xp,yp,thetap);
    [oldframe,proute] = move_robot(oldframe,proute,xp,yp,thetap,tractor);
end
%% Part 2b
sl = 0;
sr = 0;
d = deg2rad(5);

for i=0:dt:2*pi/w % Given that the distance is 2*pi*r and w = 1/5 then t = 10*pi
    [xp,yp,thetap] = kinematic_ss(v,w,r,l,dt,sl,sr,0,xp,yp,thetap);
    [oldframe,proute] = move_robot(oldframe,proute,xp,yp,thetap,tractor);
end
%% Part 2c
sl = 0.1;
sr = 0.2;
d = deg2rad(5);

for i=0:dt:2*pi/w % Given that the distance is 2*pi*r and w = 1/5 then t = 10*pi
    [xp,yp,thetap] = kinematic_ss(v,w,r,l,dt,sl,sr,0,xp,yp,thetap);
    [oldframe,proute] = move_robot(oldframe,proute,xp,yp,thetap,tractor);
end