%% Homework 2 - EBS289K
clear all; close all; clc;
global dT; global DT;
global n;
global route;

%% Part 1 - row center distance 2m 

n = 1; 
figure(1);

R = 2; % turning radius (m)
w = 1.5; % vehicle width (m)
L = 2.5; % vehicle length (m)
tractor = draw_tractor(w,L); % draw a vehicle element
gamma_max = pi/4; % tunring radius max (radians)
v_max = 5; % max velocity (m/s)
dT = 0.01; % Euller integration (s)
DT =  0.01; % Controller integration (s) 
s = 0.0; % slip (0<s<1)  
tal_v = 0; % (s)
tal_gamma = 0; % (s)
delta1 = 0*pi/180; %(rad)
delta2 = 0*pi/180; %(rad)
constraints = [gamma_max, v_max];
u=[atan(L/R), v_max]';
Umin= - u;

% initial state
q(1,1)=0; q(2,1)=0; q(3,1)=pi/2; q(4,1) = 0; q(5,1)=0;

for i=0:dT:pi*R/v_max
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
end
title('row distance 2m');
%% Part 1 - Row Distance 3m 

n = 1;
figure(2);

R = 3; % turning radius (m)

w = 1.5; % vehicle width (m)
L = 2.5; % vehicle length (m)
tractor = draw_tractor(w,L); % draw a vehicle element
gamma_max = pi/4; % tunring radius max (radians)
v_max = 5; % max velocity (m/s)
dT = 0.01; % Euller integration (s)
DT =  0.01; % Controller integration (s) 
s = 0.0; % slip (%) 
tal_v = 0; % (s)
tal_gamma = 0; % (s)
delta1 = 0*pi/180; %(rad)
delta2 = 0*pi/180; %(rad)
constraints = [gamma_max, v_max];
u=[atan(L/R), v_max]';
Umin= - u;

% initial state
q(1,1)=0; q(2,1)=0; q(3,1)=pi/2; q(4,1) = 0; q(5,1)=0;

for i=0:dT:pi*R/v_max
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
end


title('row distance 3m');
%% Part 2 - slip angles=4 deg, row distance = 3m 


n = 1;
figure(3);
R = 3;
w = 1.5; % vehicle width (m)
L = 2.5; % vehicle length (m)
tractor = draw_tractor(w,L); % draw a vehicle element
gamma_max = pi/4; % tunring radius max (radians)
v_max = 5; % max velocity (m/s)
dT = 0.01; % Euller integration (s)
DT =  0.01; % Controller integration (s) 
s = 0.0; % slip (%) 
tal_v = 0; % (s)
tal_gamma = 0; % (s)
delta1 = 4*pi/180; %(rad)
delta2 = 4*pi/180; %(rad)
constraints = [gamma_max, v_max];
u=[atan(L/R), v_max]';
Umin= - u;

% initial state
q(1,1)=0; q(2,1)=0; q(3,1)=pi/2; q(4,1) = 0; q(5,1)=0;

for i=0:dT:pi*R/v_max
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
end
title('slip angles=4 deg, row distance = 3m');

%% part 2 - increasing slip angles 


n = 1;
figure(4);
R = 3;
w = 1.5; % vehicle width (m)
L = 2.5; % vehicle length (m)
tractor = draw_tractor(w,L); % draw a vehicle element
gamma_max = pi/4; % tunring radius max (radians)
v_max = 5; % max velocity (m/s)
dT = 0.01; % Euller integration (s)
DT =  0.01; % Controller integration (s) 
s = 0.0; % slip (%) 
tal_v = 0; % (s)
tal_gamma = 0; % (s)
delta1 = 4*pi/180; %(rad)
delta2 = 4*pi/180; %(rad)
constraints = [gamma_max, v_max];
u=[atan(L/R), v_max]';
Umin= - u;

% initial state
q(1,1)=0; q(2,1)=0; q(3,1)=pi/2; q(4,1) = 0; q(5,1)=0;

for i=0:dT:pi*R/v_max
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
end
route2_1 = route;

hold on 
n = 1;

R = 3;
w = 1.5; % vehicle width (m)
L = 2.5; % vehicle length (m)
tractor = draw_tractor(w,L); % draw a vehicle element
gamma_max = pi/4; % tunring radius max (radians)
v_max = 5; % max velocity (m/s)
dT = 0.01; % Euller integration (s)
DT =  0.01; % Controller integration (s) 
s = 0.0; % slip (%) 
tal_v = 0; % (s)
tal_gamma = 0; % (s)
delta1 = 8*pi/180; %(rad)
delta2 = 8*pi/180; %(rad)
constraints = [gamma_max, v_max];
u=[atan(L/R), v_max]';
Umin= - u;

% initial state
q(1,1)=0; q(2,1)=0; q(3,1)=pi/2; q(4,1) = 0; q(5,1)=0;

for i=0:dT:pi*R/v_max
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
end

route2_2 = route;

figure(11);
title('increasing slip angles');
hold on;
plot(route2_1(:,1),route2_1(:,2));
plot(route2_2(:,1),route2_2(:,2));
legend("slip=4 deg", "slip= 8 deg");

%% part 2 - s=0.1, d1=d2=0 


n = 1;
figure(5);
R = 3;
w = 1.5; % vehicle width (m)
L = 2.5; % vehicle length (m)
tractor = draw_tractor(w,L); % draw a vehicle element
gamma_max = pi/4; % tunring radius max (radians)
v_max = 5; % max velocity (m/s)
dT = 0.01; % Euller integration (s)
DT =  0.01; % Controller integration (s) 
s = 0.1; % slip (0<s<1) 
tal_v = 0; % (s)
tal_gamma = 0; % (s)
delta1 = 0*pi/180; %(rad)
delta2 = 0*pi/180; %(rad)
constraints = [gamma_max, v_max];
u=[atan(L/R), v_max]';
Umin= - u;

% initial state
q(1,1)=0; q(2,1)=0; q(3,1)=pi/2; q(4,1) = 0; q(5,1)=0;

for i=0:dT:pi*R/v_max
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
end
title('s=10%, d1=d2=0 ');


%% part 3 - speed lag tv=0 


n = 1;
figure(7);
R = 3;
w = 1.5; % vehicle width (m)
L = 2.5; % vehicle length (m)
tractor = draw_tractor(w,L); % draw a vehicle element
gamma_max = pi/4; % tunring radius max (radians)
v_max = 5; % max velocity (m/s)
dT = 0.01; % Euller integration (s)
DT =  0.01; % Controller integration (s) 
s = 0.0; % slip (0<s<1) 
tal_v = 0; % (s)
tal_gamma = 0; % (s)
delta1 = 0*pi/180; %(rad)
delta2 = 0*pi/180; %(rad)
constraints = [gamma_max, v_max];
u=[atan(L/R), v_max]';
Umin= - u;

% initial state
q(1,1)=0; q(2,1)=0; q(3,1)=pi/2; q(4,1) = 0; q(5,1)=0;

for i=0:dT:pi*R/v_max
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
end

route3_1 = route;

% initial state
n = 1;
tal_gamma = 0.5;
q(1,1)=0; q(2,1)=0; q(3,1)=pi/2; q(4,1) = 0; q(5,1)=0;

for i=0:dT:pi*R/v_max
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
end

route3_2 = route;

% initial state
n = 1;
tal_gamma = 1;
q(1,1)=0; q(2,1)=0; q(3,1)=pi/2; q(4,1) = 0; q(5,1)=0;

for i=0:dT:pi*R/v_max
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
end

route3_3 = route;

% initial state
n = 1;
tal_gamma = 1.5;
q(1,1)=0; q(2,1)=0; q(3,1)=pi/2; q(4,1) = 0; q(5,1)=0;

for i=0:dT:pi*R/v_max
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
end

route3_4 = route;

% initial state
n = 1;
tal_gamma = 2;
q(1,1)=0; q(2,1)=0; q(3,1)=pi/2; q(4,1) = 0; q(5,1)=0;

for i=0:dT:pi*R/v_max
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
end

route3_5 = route;

figure(8);
hold on
plot(route3_1(:,1),route3_1(:,2));
plot(route3_2(:,1),route3_2(:,2));
plot(route3_3(:,1),route3_3(:,2));
plot(route3_4(:,1),route3_4(:,2));
plot(route3_5(:,1),route3_5(:,2));
title('speed lag tv=0');
legend("tv=0", "tv=0.5","tv=1","tv=1.5","tv=2.0");

%% part 3 - speed lag tv=1 


n = 1;

R = 3;
w = 1.5; % vehicle width (m)
L = 2.5; % vehicle length (m)
tractor = draw_tractor(w,L); % draw a vehicle element
gamma_max = pi/4; % tunring radius max (radians)
v_max = 5; % max velocity (m/s)
dT = 0.01; % Euller integration (s)
DT =  0.01; % Controller integration (s) 
s = 0.0; % slip (0<s<1) 
tal_v = 1; % (s)
tal_gamma = 0; % (s)
delta1 = 0*pi/180; %(rad)
delta2 = 0*pi/180; %(rad)
constraints = [gamma_max, v_max];
u=[atan(L/R), v_max]';
Umin= - u;

% initial state
q(1,1)=0; q(2,1)=0; q(3,1)=pi/2; q(4,1) = 0; q(5,1)=0;

for i=0:dT:pi*R/v_max
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
end

route3_1 = route;

% initial state
n = 1;
tal_gamma = 0.5;
q(1,1)=0; q(2,1)=0; q(3,1)=pi/2; q(4,1) = 0; q(5,1)=0;

for i=0:dT:pi*R/v_max
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
end

route3_2 = route;

% initial state
n = 1;
tal_gamma = 1;
q(1,1)=0; q(2,1)=0; q(3,1)=pi/2; q(4,1) = 0; q(5,1)=0;

for i=0:dT:pi*R/v_max
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
end

route3_3 = route;

% initial state
n = 1;
tal_gamma = 1.5;
q(1,1)=0; q(2,1)=0; q(3,1)=pi/2; q(4,1) = 0; q(5,1)=0;

for i=0:dT:pi*R/v_max
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
end

route3_4 = route;

% initial state
n = 1;
tal_gamma = 2;
q(1,1)=0; q(2,1)=0; q(3,1)=pi/2; q(4,1) = 0; q(5,1)=0;

for i=0:dT:pi*R/v_max
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    move_robot(q(1),q(2),q(3),tractor);
end

route3_5 = route;

figure(10);
hold on
plot(route3_1(:,1),route3_1(:,2));
plot(route3_2(:,1),route3_2(:,2));
plot(route3_3(:,1),route3_3(:,2));
plot(route3_4(:,1),route3_4(:,2));
plot(route3_5(:,1),route3_5(:,2));
title('speed lag tv=1');
legend("tv=0", "tv=0.5","tv=1","tv=1.5","tv=2.0");