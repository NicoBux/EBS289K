%% Assignment #1
% Students: Guilherme De Moura Araujo & Nicolas Buxbaum
% Professor: Stavros Vougioukas
close all; clear all; clc;
global route
global n
%global oldframe
%% Part 1a 
%INPUTS
D = 5; % (m)
v = 1; % linear velocity (m/s)
w = pi/2; % angular velocity (rad/s)
wi = 2.0; % vehicle width (m)
r = 0.5; % wheel radius (m)
l = 3; % vehicle length (m)
dt = 0.05; % delta time (s)
sl = 0; %left slip (%)
sr = 0; % right sleep (%)
d = deg2rad(0); % delta (rad)
x = 0; % initial x pose
y = 0; % initial y pose
theta = deg2rad(0);
%route = [x,y];
route = zeros(2+4*(D/v/dt)+3*((pi/2)/w/dt),2);
tractor = draw_tractor(wi,l); % draw a vehicle element
% initial conditions prior to start the open loop
xp = x;
yp = y;
thetap = theta;
% Open loop
for i=0:dt:D/v-dt  %First side
    [xp,yp,thetap] = kinematic_ss(v,0,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:(pi/2)/w-dt %Vehicle turning
    [xp,yp,thetap] = kinematic_ss(0,w,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:D/v-dt  %Second side (notice that now theta = 90)
    [xp,yp,thetap] = kinematic_ss(v,0,r,l,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:(pi/2)/w-dt %Vehicle turning
    [xp,yp,thetap] = kinematic_ss(0,w,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:D/v-dt   %Third side (theta = 180)
    [xp,yp,thetap] = kinematic_ss(v,0,r,l,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:(pi/2)/w-dt %Vehicle turning
    [xp,yp,thetap] = kinematic_ss(0,w,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:D/v-dt %Third side (theta = 270)
    [xp,yp,thetap] = kinematic_ss(v,0,r,l,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end
route1 = route;
%% Part 1b 
%INPUTS

n = 1;
v = 1; % m/s
R = 5; % meters
w = v/R; % radians/s
r = 0.5; % m
l = 3; % m
wi = 2; % m
dt = 0.05; % s
sl = 0; % %
sr = 0; % %
d = deg2rad(0); % rad
x = 0; % m
y = 0; % m
theta = deg2rad(0); % rad
route = [x,y];
tractor = draw_tractor(wi,l);
move_robot(x,y,theta,tractor);
xp = x;
yp = y;
thetap = theta;


for i=0:dt:2*pi/w % Distance = 2pi, t = d/w
    [xp,yp,thetap] = kinematic_ss(v,w,r,l,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end
%% Part 2a 
n = 1;
D = 5;
v = 1; % linear velocity
w = pi/2; % angular velocity
wi = 2.0; % vehicle width
r = 0.5; % wheel radius
l = 3; % vehicle length
dt = 0.05; % delta time
sl = 0.1; %left slip
sr = 0.2; % right sleep
d = deg2rad(0); % delta (angle)
x = 0; % initial x pose
y = 0; % initial y pose
theta = deg2rad(0);
route = zeros(2+4*(D/v/dt)+3*((pi/2)/w/dt),2);
tractor = draw_tractor(wi,l); % draw a vehicle element
% initial conditions prior to start the open loop
xp = x;
yp = y;
thetap = theta;
% Open loop
for i=0:dt:D/v-dt  %First side
    [xp,yp,thetap] = kinematic_ss(v,0,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:(pi/2)/w-dt %Vehicle turning
    [xp,yp,thetap] = kinematic_ss(0,w,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:D/v-dt  %Second side (notice that now theta = 90)
    [xp,yp,thetap] = kinematic_ss(v,0,r,l,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:(pi/2)/w-dt %Vehicle turning
    [xp,yp,thetap] = kinematic_ss(0,w,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:D/v-dt   %Third side (theta = 180)
    [xp,yp,thetap] = kinematic_ss(v,0,r,l,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:(pi/2)/w-dt %Vehicle turning
    [xp,yp,thetap] = kinematic_ss(0,w,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:D/v-dt %Fourth side (theta = 270)
    [xp,yp,thetap] = kinematic_ss(v,0,r,l,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end
route2 = route;
%% Part 2b
n = 1;
D = 5;
v = 1; % linear velocity
w = pi/2; % angular velocity
wi = 2.0; % vehicle width
r = 0.5; % wheel radius
l = 3; % vehicle length
dt = 0.05; % delta time
sl = 0.0; %left slip
sr = 0.0; % right sleep
d = deg2rad(5); % delta (angle)
x = 0; % initial x pose
y = 0; % initial y pose
theta = deg2rad(0);
route = zeros(2+4*(D/v/dt)+3*((pi/2)/w/dt),2);
tractor = draw_tractor(wi,l); % draw a vehicle element
% initial conditions prior to start the open loop
xp = x;
yp = y;
thetap = theta;
% Open loop
for i=0:dt:D/v-dt  %First side
    [xp,yp,thetap] = kinematic_ss(v,0,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:(pi/2)/w-dt %Vehicle turning
    [xp,yp,thetap] = kinematic_ss(0,w,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:D/v-dt  %Second side (notice that now theta = 90)
    [xp,yp,thetap] = kinematic_ss(v,0,r,l,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:(pi/2)/w-dt %Vehicle turning
    [xp,yp,thetap] = kinematic_ss(0,w,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:D/v-dt   %Third side (theta = 180)
    [xp,yp,thetap] = kinematic_ss(v,0,r,l,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:(pi/2)/w-dt %Vehicle turning
    [xp,yp,thetap] = kinematic_ss(0,w,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:D/v-dt %Fourth side (theta = 270)
    [xp,yp,thetap] = kinematic_ss(v,0,r,l,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end
route3 = route;
%% Part 2c
n = 1;
D = 5;
v = 1; % linear velocity
w = pi/2; % angular velocity
wi = 2.0; % vehicle width
r = 0.5; % wheel radius
l = 3; % vehicle length
dt = 0.05; % delta time
sl = 0.1; %left slip
sr = 0.2; % right sleep
d = deg2rad(5); % delta (angle)
x = 0; % initial x pose
y = 0; % initial y pose
theta = deg2rad(0);
route = zeros(2+4*(D/v/dt)+3*((pi/2)/w/dt),2);
tractor = draw_tractor(wi,l); % draw a vehicle element
% initial conditions prior to start the open loop
xp = x;
yp = y;
thetap = theta;
% Open loop
for i=0:dt:D/v-dt  %First side
    [xp,yp,thetap] = kinematic_ss(v,0,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:(pi/2)/w-dt %Vehicle turning
    [xp,yp,thetap] = kinematic_ss(0,w,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:D/v-dt  %Second side (notice that now theta = 90)
    [xp,yp,thetap] = kinematic_ss(v,0,r,l,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:(pi/2)/w-dt %Vehicle turning
    [xp,yp,thetap] = kinematic_ss(0,w,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:D/v-dt   %Third side (theta = 180)
    [xp,yp,thetap] = kinematic_ss(v,0,r,l,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:(pi/2)/w-dt %Vehicle turning
    [xp,yp,thetap] = kinematic_ss(0,w,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end

for i=0:dt:D/v-dt %Fourth side (theta = 270)
    [xp,yp,thetap] = kinematic_ss(v,0,r,l,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
end
route4 = route;
%% 3. PLOTS
figure;
plot(route1(:,1),route1(:,2),'b-');
hold on
plot(route2(:,1),route2(:,2),'r-');
legend('Sr = 0, Sl = 0, d = 0','Sr = 0.1, Sl = 0.2, d = 0');

figure;
plot(route1(:,1),route1(:,2),'b-');
hold on
plot(route3(:,1),route3(:,2),'g-');
legend('Sr = 0, Sl = 0, d = 0','Sr = 0, Sl = 0, d = 5');

figure;
plot(route1(:,1),route1(:,2),'b-');
hold on
plot(route4(:,1),route4(:,2),'c-');
legend('Sr = 0, Sl = 0, d = 0','Sr = 0.1, Sl = 0.2, d = 5');