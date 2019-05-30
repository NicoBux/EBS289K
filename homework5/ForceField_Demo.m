%% EBS289K - Agricultural Robotics and Automation - Spring 2019 
% Homework Assignment #5 - Demo: Force Fields
% Students: Guilherme De Moura Araujo & Nicolas Buxbaum
% Professor: Stavros Vougioukas
%% Start
close all; clear all; clc;
addpath(genpath('geom2d'));
%%
global bitmap bitodds route;
q = [6,6,-pi/2,1,0]; % Initial Pose
route = [q(1),q(2)];
wSize = [3,3]; Fcr = 1; Fct = 1; % Search window & force constants
T = [7,16]; % Target location
R = 500; C = 500; Xmax = 30; Ymax = 30; % Physical & Discretized space
bitmap = zeros(R,C);
bitodds = ones(R,C);
dT = 0.001; DT =  0.01;
wi = 2.5; %Tractor width [m]
L = 3; % Wheel base [m]
tractor = draw_tractor(wi,L); % draw a vehicle element
gamma_max = 60*pi/180; %radians
s = 0.0; %Slip [%]
tal_v = 0; tal_gamma = 0; %Controller delay times [s]
delta1 = 0*pi/180; delta2 = 0*pi/180; %Skid factors [%]
v_max = 1; %Maximum allowed velocity [m/s]
constraints = [gamma_max, v_max]; %Constraints 
u = [0, v_max]'; %Desired state [radians] / [m/s]
k = 1;
for i = 1:DT:30-DT
[omega(k)] = virtualForceField(q,R,C,Xmax,Ymax,T,wSize,Fcr,Fct);
u(1) = atan(q(4)*omega(k)/L);
k = k+1;
q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
move_robot(q(1),q(2),q(3),tractor,1);
if abs(q(1)-T(1))<0.1 && abs(q(2)-T(2))<0.1
    break
end
end
%close all
plot(route(end-k+2:end,1),route(end-k+2:end,2))
hold on;
plot(T(1),T(2),'o');