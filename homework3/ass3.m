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
dT = 0.01; DT =  0.01; %% mode integration step & controller integratin step
s = 0.0;
tal_v = 0; tal_gamma = 0;
delta1 = 0*pi/180; delta2 = 0*pi/180;
constraints = [gamma_max, v_max];
u=[atan(L/R), v_max]';
Umin= - u;
q = [1,1,0,5,pi];
Ld = 2;
path = [2,2;5,5;10,6;];

for i = 0:dT:2
    [q(5),error]  = purePursuitController(q,L,Ld,path);
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
end