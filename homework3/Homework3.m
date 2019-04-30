clear all; close all; clc;
%%
global dT; global DT;
global n;

n = 1; %Initializes a new drawing
w = 1.5; %Tractor width [m]
r = 0.5; %Tractor wheel radius [m]
L = 2.5; % Wheel base [m]
tractor = draw_tractor(w,L); % draw a vehicle element
gamma_max = pi/4; %radians
v_max = 5; %Maximum vehicle speed [m/s]
R = 2.5; %Tractor turning radius [m]
dT = 0.001; DT =  0.1; %% mode integration step & controller integratin step [s]
s = 0.0; %Slip [%]
tal_v = 0; tal_gamma = 0; %Controller delay times [s]
delta1 = 0*pi/180; delta2 = 0*pi/180; %Skid factors [%]
constraints = [gamma_max, v_max]; %Constraints (negative part is built in
%the bycicle model function)
u=[atan(L/R), v_max]'; %Desired state [radians] / [m/s]
q = [15,5,pi/2,5,0]; %Current state [x,y,theta,velocity,gamma] [m,m,radians,m/s,radians]
Ld = 2; %Look ahead distance [m]
x = zeros(1,length(0:0.1:2*pi)); %x coordinate of path
y = zeros(1,length(0:0.1:2*pi)); %y coordinate of path
k=1;
for i=0:0.1:2*pi
x(k) = 9+5*sin(i);
y(k) = 7-5*cos(i);
k=k+1;
end
cur = 5; %curvature [m]
path(:,1) = x;
path(:,2) = y;
k = 1; %Just an index
MAX = 200; %Maximum number of steps allowed for the simulation
error = zeros(1,length(0:DT:2*pi*cur/v_max)); %error tracking
for i = 0:DT:60-DT
    [u(1),error(k)]  = purePursuit(q,L,Ld,path); %Update gamma based on purePursuit
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
    k = k+1;
    if k>MAX %Simulation break criteria
        break
    end
end
plot(x,y,'bo')
figure()
plot(1:length(error),error);
xlabel('Steps');
ylabel('Error (m)');
%% Part 2 - Lane-change path
n = 1; %Initializes a new drawing
Ld = 5; %Look ahead distance [m]
clear path;
u=[0, v_max]'; %Desired state [radians] / [m/s]
q = [0,0,0,5,0]; %Current state [x,y,theta,velocity,gamma] [m,m,radians,m/s,radians]

space = 0.2; %Point spacing
x1 = 0:space:10-space; y1 = zeros(1,10/space); %First line
x2 = 10*ones(1,5/space); y2 = 0:space:5-space; %Second line
x3 = 10:space:20; y3 = 5*ones(1,10/space+1); %Third line
x = [x1,x2,x3]; y = [y1,y2,y3]; %Path with waypoints
path(:,1) = x;
path(:,2) = y;
k = 1;
MAX = 50; %Maximum number of steps allowed for the simulation
error = zeros(1,length(0:DT:5));
for i = 0:DT:60-DT
    [u(1),error(k)] = purePursuit(q,L,Ld,path);
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
    k = k+1;
    if k>MAX
        break
    end
end
plot(x,y,'bo');
figure()
plot(1:length(error),error);
xlabel('Steps');
ylabel('Error (m)');
%% Part 3 - Introduce tal_v and tal_gamma
n = 1; %Initializes a new drawing
Ld = 5; %Look ahead distance [m]
tal_v = 0.15; tal_gamma = 0.5; %Controller delay times [s]
u=[0, v_max]'; %Desired state [radians] / [m/s]
q = [0,0,0,5,0]; %Current state [x,y,theta,velocity,gamma] [m,m,radians,m/s,radians]

k = 1;
MAX = 53; %Maximum number of steps allowed for the simulation
error = zeros(1,length(0:DT:5));
for i = 0:DT:60-DT
    [u(1),error(k)] = purePursuit(q,L,Ld,path);
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
    k = k+1;
    if k>MAX
        break
    end
end
plot(x,y,'bo');
figure()
plot(1:length(error),error);
xlabel('Steps');
ylabel('Error (m)');

%% Part 4 - Double steering time lag
n = 1; %Initializes a new drawing
Ld = 5; %Look ahead distance [m]
tal_v = 0.15; tal_gamma = 1; %Controller delay times [s]
u=[0, v_max]'; %Desired state [radians] / [m/s]
q = [0,0,0,5,0]; %Current state [x,y,theta,velocity,gamma] [m,m,radians,m/s,radians]

space = 0.2; %Point spacing
x1 = 0:space:10-space; y1 = zeros(1,10/space); %First line
x2 = 10*ones(1,5/space); y2 = 0:space:5-space; %Second line
x3 = 10:space:20; y3 = 5*ones(1,10/space+1); %Third line
x = [x1,x2,x3]; y = [y1,y2,y3]; %Path with waypoints
path(:,1) = x;
path(:,2) = y;
k = 1;
MAX = 57; %Maximum number of steps allowed for the simulation
error = zeros(1,length(0:DT:5));
for i = 0:DT:60-DT
    [u(1),error(k)] = purePursuit(q,L,Ld,path);
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
    k = k+1;
    if k>MAX
        break
    end
end
plot(x,y,'bo');
figure()
plot(1:length(error),error);
xlabel('Steps');
ylabel('Error (m)');
%% Part 5 - Tighten Steering Angle to 35 degress
tal_v = 0.0; tal_gamma = 0.0; %Controller delay times [s]
n = 1; %Initializes a new drawing
Ld = 5; %Look ahead distance [m]
u=[0, v_max]'; %Desired state [radians] / [m/s]
q = [0,0,0,5,0]; %Current state [x,y,theta,velocity,gamma] [m,m,radians,m/s,radians]
gamma_max = 35*pi/180; %radians
constraints = [gamma_max, v_max]; %Constraints (negative part is built in
%the bycicle model function)

space = 0.2; %Point spacing
x1 = 0:space:10-space; y1 = zeros(1,10/space); %First line
x2 = 10*ones(1,5/space); y2 = 0:space:5-space; %Second line
x3 = 10:space:20; y3 = 5*ones(1,10/space+1); %Third line
x = [x1,x2,x3]; y = [y1,y2,y3]; %Path with waypoints
path(:,1) = x;
path(:,2) = y;
k = 1;
MAX = 51; %Maximum number of steps allowed for the simulation
error = zeros(1,length(0:DT:5));
for i = 0:DT:60-DT
    [u(1),error(k)] = purePursuit(q,L,Ld,path);
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
    k = k+1;
    if k>MAX
        break
    end
end
plot(x,y,'bo');
figure()
plot(1:length(error),error);
xlabel('Steps');
ylabel('Error (m)');
%% Part 6 - Introduce skidding & Slip

n = 1; %Initializes a new drawing
Ld = 5; %Look ahead distance [m]
tal_v = 0.0; tal_gamma = 0.0; %Controller delay times [s]
u=[0, v_max]'; %Desired state [radians] / [m/s]
q = [0,0,0,5,0]; %Current state [x,y,theta,velocity,gamma] [m,m,radians,m/s,radians]
gamma_max = 45*pi/180; %radians
constraints = [gamma_max, v_max]; %Constraints (negative part is built in
%the bycicle model function)
s = 0.15; %Slip [%]
delta1 = 0*pi/180; delta2 = 0*pi/180; %Skid factors [%]

space = 0.2; %Point spacing
x1 = 0:space:10-space; y1 = zeros(1,10/space); %First line
x2 = 10*ones(1,5/space); y2 = 0:space:5-space; %Second line
x3 = 10:space:20; y3 = 5*ones(1,10/space+1); %Third line
x = [x1,x2,x3]; y = [y1,y2,y3]; %Path with waypoints
path(:,1) = x;
path(:,2) = y;
k = 1;
MAX = 51; %Maximum number of steps allowed for the simulation
error = zeros(1,length(0:DT:5));
for i = 0:DT:60-DT
    [u(1),error(k)] = purePursuit(q,L,Ld,path);
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
    k = k+1;
    if k>MAX
        break
    end
end
plot(x,y,'bo');
figure()
plot(1:length(error),error);
xlabel('Steps');
ylabel('Error (m)');