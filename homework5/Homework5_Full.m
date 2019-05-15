close all; clear all; clc;
global bitmap route;
route = [5,4];
q = [5,4,pi/2,1,0];
T = [10,10];
R = 500; C = 500; Xmax = 30; Ymax =30; wSize = [3,3]; Fcr = 1; Fct = 1;
bitmap = zeros(R,C);
dT = 0.001; DT =  0.1;
wi = 2.5; %Tractor width [m]
L = 3; % Wheel base [m]
tractor = draw_tractor(wi,L); % draw a vehicle element
gamma_max = 60*pi/180; %radians
Rmin = L/tan(gamma_max); %Tractor turning radius [m]
s = 0.0; %Slip [%]
tal_v = 0; tal_gamma = 0; %Controller delay times [s]
delta1 = 0*pi/180; delta2 = 0*pi/180; %Skid factors [%]
v_max = 1; %Maximum allowed velocity [m/s]
constraints = [gamma_max, v_max]; %Constraints (negative part is built in
%the bycicle model function)
u = [0, v_max]'; %Desired state [radians] / [m/s]
% pathx = 5*ones(1,54);
% pathy = q(2):0.3:T(2);
% path(1,:) = pathx;
% path(2,:) = pathy;
%path = [0,0;10,10]';
%gamma=zeros(1,length(1:dT:30));
omegap = 0;
k = 1;
for i = 1:DT:30-DT
gamma(k) = virtualForceField(q,R,C,Xmax,Ymax,T,wSize,Fcr,Fct,omegap);
u(1) = atan(q(4)*gamma(k)/L);
k = k+1;
q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
move_robot(q(1),q(2),q(3),tractor,0);
hold on;
track = route;
end