%% EBS289K - Agricultural Robotics and Automation - Spring 2019 
% Homework Assignment #5 - Part 3: Scenario 1
% Students: Guilherme De Moura Araujo & Nicolas Buxbaum
% Professor: Stavros Vougioukas

%% Start
clear all; clc; close all;
addpath(genpath('geom2d'));
%% Code
global bitmap bitodds route rangeMax;

%lidar values
rangeMax = 30; angleSpan = pi; angleStep = angleSpan/360; 

Xmax = 50; Ymax = 50; R = 150; C = 150;
%physical dimensions of space; rows and columns -> discretization of
%physical space in a grid

map=zeros(R, C); bitmap = 0.0* ones(R, C); %initialize as empty
q = [25,0,pi/2,1,0]; T = [25,30]; %initial pose; target location
route = [q(1),q(2)]; bitodds = ones(R,C);

% Create test rectangular obstacle
Xsw = 25.5; Ysw = 20; Xne = Xsw + 1; Yne = Ysw + 1;
[Isw, Jsw] = XYtoIJ(Xsw, Ysw, Xmax, Ymax, R, C);
[Ine, Jne] = XYtoIJ(Xne, Yne, Xmax, Ymax, R, C);
map(Ine:Isw, Jsw:Jne) = 1;

% Navigation parameters
wi = 1.5; L = 2.5; % Tractor width [m] & Wheel base [m]
tractor = draw_tractor(wi,L); % draw a vehicle element
gamma_max = 45*pi/180; s = 0.0;delta1 = 0*pi/180; delta2 = 0*pi/360; 
%maximum steering angle [radians]; Slip [%]; Skid factors [%]
tal_v = 0; tal_gamma = 0; %Controller delay times [s]
v_max = 1; constraints = [gamma_max, v_max]; u = [0, v_max]';
%Maximum allowed velocity [m/s]; Constraints; Desired state [radians & m/s]
dT = 0.001; DT =  0.01; %Controller and Euler integration time steps
%% Main Loop
% Force field parameters
wSize = [4,4]; Fcr = 1/3; Fct = 1; %seach window & force constants
omega = zeros(length(1:DT:31));
k = 1;
for j=1:DT:33
    Tl = SE2([q(1) q(2) q(3)]);
    p = laserScanner(angleSpan, angleStep, rangeMax, Tl.T, map, Xmax, Ymax);  
    for i=1:length(p)
        angle = p(i,1); range = p(i,2);
        % handle infinite range
        if(isinf(range)) 
            range = rangeMax+1;
        end
        n = updateLaserBeamGrid(angle, range, Tl.T, R, C, Xmax, Ymax, 0.95);
    end
    [omega(k)] = virtualForceField(q,R,C,Xmax,Ymax,T,wSize,Fcr,Fct);
    u(1) = atan(q(4)*omega(k)/L);
    k = k+1
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor,0);
    if abs(q(1)-T(1))<0.3 && abs(q(2)-T(2))<0.3
        break
    end
end

%% PLOTING

plot(route(end-k+2:end,1),route(end-k+2:end,2));
hold on;
plot(T(1),T(2),'o');
patch([25.5,25.5,26.5,26.5,25.5],[20,21,21,20,20],'black');
legend('Robot path','Goal point','Obstacle');
axis square
axis equal
xlim([20 30]);
ylim([0 31]);