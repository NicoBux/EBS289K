clear all; close all; clc;
%%
global dT; global DT;
global n;

n = 1; %Initializes a new drawing
w = 1.5;
r = 0.5; %m
L = 2.5; %m
tractor = draw_tractor(w,L); % draw a vehicle element
gamma_max = pi/4; %radians
v_max = 5; %m/s
R = 2.5;
dT = 0.001; DT =  0.1; %% mode integration step & controller integratin step
s = 0.0;
tal_v = 0; tal_gamma = 0;
delta1 = 0*pi/180; delta2 = 0*pi/180;
constraints = [gamma_max, v_max];
u=[atan(L/R), v_max]';
q = [15,5,pi/2,5,0];
Ld = 2;
k = 1;
x = zeros(1,length(0:0.1:2*pi));
y = zeros(1,length(0:0.1:2*pi));
for i=0:0.1:2*pi
x(k) = 9+5*sin(i);
y(k) = 7-5*cos(i);
k=k+1;
end
cur = 5;
path(:,1) = x;
path(:,2) = y;
%path = [2,2;5,5;10,6;];
%plot(x,y);
k = 1;
error = zeros(1,length(0:DT:2*pi*cur/v_max));
for i = 0:DT:60-DT
    [u(1),error(k)]  = purePursuit(q,L,Ld,path);
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
    k = k+1;
    if k>=length(x)
        break
    end
end
plot(x,y,'bo')
figure()
plot(1:length(error),error);
%% Part 2
%plot(x,y);
n = 1;
Ld = 5;
clear path;
u=[0, v_max]';
q = [0,0,0,5,0];
space = 0.2;
x1 = 0:space:10-space; y1 = zeros(1,10/space);
x2 = 10*ones(1,5/space); y2 = 0:space:5-space;
x3 = 10:space:20; y3 = 5*ones(1,10/space+1);
x = [x1,x2,x3]; y = [y1,y2,y3];
path(:,1) = x;
path(:,2) = y;
%path = [0,0;10,0;10,5;20,5];
k = 1;
error = zeros(1,length(0:DT:5));
for i = 0:DT:60-DT
    [u(1),error(k)] = purePursuit(q,L,Ld,path);
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
    k = k+1;
    if k>=length(x)
        break
    end
end
plot(x,y,'bo');
figure()
plot(1:length(error),error);