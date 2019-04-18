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
p1 = 0; p2 = 0; p3 = 0; p4 = 0; p5 = 0; p6 = 0; p7 = 0;
% Open loop
for i=0:dt:D/v-dt  %First side
    [xp,yp,thetap] = kinematic_ss(v,0,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
    p1 = p1+1;
end

for i=0:dt:(pi/2)/w-dt %Vehicle turning
    [xp,yp,thetap] = kinematic_ss(0,w,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
    p2 = p2+1;
end

for i=0:dt:D/v-dt  %Second side (notice that now theta = 90)
    [xp,yp,thetap] = kinematic_ss(v,0,r,l,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
    p3 = p3+1;
end

for i=0:dt:(pi/2)/w-dt %Vehicle turning
    [xp,yp,thetap] = kinematic_ss(0,w,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
    p4 = p4+1;
end

for i=0:dt:D/v-dt   %Third side (theta = 180)
    [xp,yp,thetap] = kinematic_ss(v,0,r,l,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
    p5 = p5+1;
end

for i=0:dt:(pi/2)/w-dt %Vehicle turning
    [xp,yp,thetap] = kinematic_ss(0,w,r,wi,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
    p6 = p6+1;
end

for i=0:dt:D/v-dt %Third side (theta = 270)
    [xp,yp,thetap] = kinematic_ss(v,0,r,l,dt,sl,sr,d,xp,yp,thetap);
    move_robot(xp,yp,thetap,tractor);
    p7 = p7+1;
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
plot(route1(:,1),route1(:,2),'b-')
hold on
plot(route2(:,1),route2(:,2),'r-')
plot(route3(:,1),route3(:,2),'g-')
plot(route4(:,1),route4(:,2),'c-')
legend('Sr = 0, Sl = 0, d = 0','Sr = 0.1, Sl = 0.2, d = 0','Sr = 0, Sl = 0, d = 5','Sr = 0.1, Sl = 0.2, d = 5')

%% 3a Plot Comments
% 
% <<finalplot.jpg>>
% 
% *sl=0, sr=0, ?=0* is a no slip and no skip condition. As expected, the
% kinematic model yields positions that precisely follow the shape of a 5m
% square. 
% *sl=0.1, sr=0.2, ?=0* is a slip no skid condition. Normally in a straght
% path the vehicle anglular velocity is 0. However with the presence of uneven slip
% in both wheels, the vehicle turns due to non-equal left and right wheel angular velocities resulting in a changing theta value.  
% *sl=0.1, sr=0.2, ?=5o* is a slip and skid condition. The robot
% expereinces both a changing theta during the straight line path and a
% lateral velocity Vy.
% *sl=0, sr=0, ?=5o* is a no skip skid condition. The vehicle path is at
% angle with the desired path due to the presence of lateral velocity Vy.
% With both slips zero theta is zero and the vehicle angular velocity is
% also 0 (and the robot travels the correct longitudinal distance). 
%% 4. Integration Step ?t
% With an increased integration step, the kinematic model creates longer
% linear movements between points. The robot also is plotted roughly ten
% times as fast. The effect is most noticible in the circular path, shown
% in the figure below. 
%
% <<circeldeltaT.jpg>>
% 

