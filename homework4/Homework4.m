%% Homework #4
close all; clear all; clc;

%addpath(genpath('functions'));

global N; global W; global RL; global n; global dT; global DT; global Rmin;
global npoints;

npoints = 80;
N = 10; %number of rows
W = 2.5; %m, row width
RL = 20; %row length m
x = [-W, W/2:W:(N)*W, W/2:W:(N)*W, -W];
y = [RL/2, zeros(1,N), RL*ones(1,N), RL/2];
xy = [x;y].';

%RL = 2;
%n = 1; %Initializes a new drawing
wi = 2.5; %Tractor width [m]
r = 0.5; %Tractor wheel radius [m]
L = 3; % Wheel base [m]
%Ld = 1.5;
tractor = draw_tractor(wi,L); % draw a vehicle element
gamma_max = 60*pi/180; %radians
v_max = 1; %Maximum vehicle speed [m/s]
Rmin = L/tan(gamma_max); %Tractor turning radius [m]
dT = 0.001; DT =  0.1; %% mode integration step & controller integratin step [s]
s = 0.0; %Slip [%]
tal_v = 0; tal_gamma = 0; %Controller delay times [s]
delta1 = 0*pi/180; delta2 = 0*pi/180; %Skid factors [%]
constraints = [gamma_max, v_max]; %Constraints (negative part is built in
%the bycicle model function)

%% Compute non-turning costs aij 
%Create cost table DMAT
%Moving from ANY lower headland to ANY upper headland
%Only possible if both nodes are in the same field row

huge = 1E10;
DMAT = zeros(2*N+2,2*N+2);

for i = 2:N+1
    for j = N+2:2*N+1
        if (j-i) == N
            DMAT(i,j) = -huge/2;
            DMAT(j,i) = -huge/2;
        else
            DMAT(i,j) = huge;
            DMAT(j,i) = huge;
        end
    end
end
%% Compute turning costs

for i = 2:N
    for j = i+1:N+1
        d = abs(i-j);
        if(Rmin<=d*W/2) %PI Turn
            DMAT(i,j) = d*W + ((pi-2)*Rmin);
        else %omega turn
            cost = ((2*Rmin+d*W)^2)/(8*Rmin^2);
            DMAT(i,j) = 3*pi*Rmin - 2*Rmin*acos(1-cost);
        end
        DMAT(j,i) = DMAT(i,j);
        DMAT(i+N,j+N) = DMAT(i,j); %make sure the corresponding ones are equal as well
        DMAT(j+N,i+N) = DMAT(i,j); %make sure the corresponding ones are equal as well
    end
end
%% Compute costs aij involving start/end nodes
%Approximate using the Manhattan distance
%|x1-x2|+|y1+y2|
for i = 2:2*N+1
    DMAT(1,i) = abs(x(1)-x(i))+abs(y(1)-y(i));
    DMAT(i,1) = DMAT(1,i);
    DMAT(2*N+2,i) = abs(x(2*N+2)-x(i))+abs(y(2*N+2)-y(i));
    DMAT(i,2*N+2) = DMAT(2*N+2,i);
end
%% Make start and end nodes big
DMAT(1,2*N+2) = huge; %going from 1 - 2*N+2 huge
DMAT(2*N+2,1) = huge; %going from 2*N+2 - 1 huge

%%
t = cputime;
resultStruct = tspof_ga('XY',xy,'DMAT',DMAT,'SHOWRESULT',false,'SHOWWAITBAR',false,'SHOWPROG',false);
E = cputime-t;
route = [1 resultStruct.optRoute 2*N+2];
resultStruct.minDist;
%%
% route(end)=length(route);
% %[~,routesort]=sort(route); %Get the order of B
% route=original(route);

%% Path making
path = [];
j = 0;
%for i = 1:2*N+1
for i = 1:length(route)-1
    if i<length(route)%i<2*N+2
        d = route(i)-route(i+1);
        current = xy(route(i),:);
        next = xy(route(i+1),:);
    else
        d = 0;
        current = xy(route(i),:);
        next = 0;
    end
    
    if i == 1
            path = [current(1)*ones(1,npoints);linspace(current(2),next(2),npoints)];
            turnpath = PiTurn(current,next,route(i+1));
            path = [path turnpath];
       
    elseif i==length(route)-1 %i==2*N+1
        pathturn = PiTurn(current,next,route(i));
        pathstraight = [next(1)*ones(1,npoints);linspace(current(2),next(2),npoints)];
        pathn = [pathturn pathstraight];
        path = [path pathn];
    else
        if abs(d)==N
            pathn = [current(1)*ones(1,npoints);linspace(current(2),next(2),npoints)];
        else
            if abs(route(i)-route(i+1))==1 %||  L/tan(gammaMax) < W*abs(route(i)-route(i+1)) %if turning radius is less than the row width * distance over
                pathn = OmegaTurn(current,next,route(i),route(i+1));
                pathn=pathn';
            else
                pathn = PiTurn(current,next,route(i+1));
            end
        end
        path = [path pathn];
    end
    
end

%%
clc;
R = 2.5;
x = path(1,1);
y = path(2,1);
theta = atan2(path(2,2)-y,path(1,2)-x);
v_max = 0.8;
u=[atan(L/R), v_max]'; %Desired state [radians] / [m/s]
q = [x,y,theta,1,0]; %Current state [x,y,theta,velocity,gamma] [m,m,radians,m/s,radians]
Ld = 2.3; %Look ahead distance [m]
k = 1;
MAX = 3000;
n = 1;
error = zeros(1,min(MAX,length(0:DT:300-DT))); %error tracking
for i = 0:DT:300-DT
    [u(1),error(k)]  = purePursuitController(q,L,Ld,path'); %Update gamma based on purePursuitController
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    hold on;
    plot(path(1,:),path(2,:),'ro');
    move_robot(q(1),q(2),q(3),tractor);
    k = k+1;
    if k>MAX %Simulation break criteria
        break
    end
end

hold on
plot(path(1,:),path(2,:),'ro');