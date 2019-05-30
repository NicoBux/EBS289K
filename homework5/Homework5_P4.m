%% EBS289K - Agricultural Robotics and Automation - Spring 2019 
% Homework Assignment #5 - Part 4: Scenario 2
% Students: Guilherme De Moura Araujo & Nicolas Buxbaum
% Professor: Stavros Vougioukas

%% Start
clear all; clc; close all;
addpath(genpath('geom2d'));
global bitmap bitodds route rangeMax;
%% Lidar/Field specs

Xmax = 50; Ymax = 50; R = 350; C = 350; %physical dimensions of space;
% rows and columns -> discretization of physical space in a grid 
rangeMax = 20; angleSpan = pi; angleStep = angleSpan/360; % Lidar specs

%% Obstacle making

Xsw(1) = 23.2; Ysw(1) = 15; Xne(1) = Xsw(1) + 0.3; Yne(1) = Ysw(1) + 0.3;
map = zeros(R,C);

for i = 2:4
Xsw(i) = Xsw(i-1)-0.5;
Ysw(i) = Ysw(i-1)+sqrt(3)/2;
Xne(i) = Xsw(i) + 0.3;
Yne(i) = Ysw(i) + 0.3;
end

sw = [Xsw;Ysw];
ne = [Xne;Yne];

sw(1,:) = sw(1,:)-0.3;
ne(1,:) = ne(1,:)-0.3;

for i=1:4
    [Isw(i), Jsw(i)] = XYtoIJ(Xsw(i), Ysw(i), Xmax, Ymax, R, C);
    [Ine(i), Jne(i)] = XYtoIJ(Xne(i), Yne(i), Xmax, Ymax, R, C);
    map(Ine(i):Isw(i), Jsw(i):Jne(i)) = 1;
end

sw2(1,:) = sw(1,:)+3*sqrt(3)/2; sw2(2,:) = sw(2,:)+1.5;
ne2(1,:) = ne(1,:)+3*sqrt(3)/2; ne2(2,:) = ne(2,:)+1.5;

for i=1:4
    Xsw2(i) = sw2(1,i);
    Ysw2(i) = sw2(2,i);
    Xne2(i) = ne2(1,i);
    Yne2(i) = ne2(2,i);
end

for i=1:4
    [Isw2(i), Jsw2(i)] = XYtoIJ(Xsw2(i), Ysw2(i), Xmax, Ymax, R, C);
    [Ine2(i), Jne2(i)] = XYtoIJ(Xne2(i), Yne2(i), Xmax, Ymax, R, C);
    map(Ine2(i):Isw2(i), Jsw2(i):Jne2(i)) = 1;
end

% imshow(map)

%% Main Loop
% Navigation parameters
q = [25.0,0,pi/2,1,0];       
T = [q(1),30];
route = [q(1),q(2)];
bitodds = ones(R,C); bitmap = 0.5*ones(R,C);
wi = 1.5; L = 2.5; % Tractor width [m] & Wheel base [m]
tractor = draw_tractor(wi,L); % draw a vehicle element
gamma_max = 45*pi/180; s = 0.0;delta1 = 0*pi/180; delta2 = 0*pi/360; 
%maximum steering angle [radians]; Slip [%]; Skid factors [%]
tal_v = 0; tal_gamma = 0; %Controller delay times [s]
v_max = 1; constraints = [gamma_max, v_max]; u = [0, v_max]';
%Maximum allowed velocity [m/s]; Constraints; Desired state [radians & m/s]
k = 1; %previous steering command issued
dT = 0.001; DT =  0.01; %Controller and Euler integration time steps
    
% Force field parameters
wSize = [7,7]; Fcr = 1/1.558; Fct = 2.5; %seach window & force constants
omega = zeros(length(1:DT:31));
% Navigation loop
for j=1:DT:34
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
close all;
%% PLOTING
plot(route(end-k+2:end,1),route(end-k+2:end,2),'k-');
hold on;
plot(T(1),T(2),'ro');
axis square
axis equal
xlim([20 30]);
ylim([0 31]);

for i=1:4
    patch([sw(1,i) sw(1,i) ne(1,i) ne(1,i)],[sw(2,i) ne(2,i) ne(2,i) sw(2,i)],'black');
end
for i=1:4
    patch([sw2(1,i) sw2(1,i) ne2(1,i) ne2(1,i)],[sw2(2,i) ne2(2,i) ne2(2,i) sw2(2,i)],'black');
end
legend('Robot path','Goal point','Obstacles');