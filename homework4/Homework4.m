%% Homework #4
close all; clear all; clc;

global N; global W; global RL; global n; global dT; global DT; global Rmin;
global npoints;

npoints = 50;
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

DMAT = costMatrix(N,W,xy);
%%
t = cputime;
resultStruct = tspof_ga('XY',xy,'DMAT',DMAT,'SHOWRESULT',false,'SHOWWAITBAR',false,'SHOWPROG',false);
E = cputime-t;
route = [1 resultStruct.optRoute 2*N+2];
resultStruct.minDist;

path = pathGen(route,xy,npoints);
%%
clc;
R = 2.5;
x = path(1,1);
y = path(2,1);
theta = atan2(path(2,2)-y,path(1,2)-x);
v_max = 0.8;
u=[atan(L/R), v_max]'; %Desired state [radians] / [m/s]
q = [x,y,theta,1,0]; %Current state [x,y,theta,velocity,gamma] [m,m,radians,m/s,radians]
Ld = 0.7; %Look ahead distance [m]
k = 1;
MAX = 3740;
n = 1;
error = zeros(1,min(MAX,length(0:DT:300-DT))); %error tracking
for i = 0:DT:400-DT
    [u(1),error(k)]  = purePursuitController(q,L,Ld,path'); %Update gamma based on purePursuitController
    q = bycicle_model(u,q,dT,DT,L,s,tal_v,tal_gamma,delta1,delta2,constraints);
    hold on;
    plot(path(1,:),path(2,:),'yo');
    move_robot(q(1),q(2),q(3),tractor);
    k = k+1
    if k>=MAX %Simulation break criteria
        break
    end
end

hold on
%plot(path(1,:),path(2,:),'ro');
figure();
plot(1:k-1,error);
xlabel('Steps');
ylabel('Error [m]');
mean_error = mean(error);
x1 = 1; x2 = length(error);
x_mean = [x1 x2];
y_mean = [mean_error mean_error];
hold on;
plot(x_mean,y_mean,'r-');
legend('Error [m]','Mean error [m]');
max_error = max(error(1:k-1));
percentile_error = prctile(error(1:k-1),95);
rmse_error = rms(error(1:k-1));
fprintf('The maximum error is %.3f m\n', max_error);
fprintf('The 95th percentile error is %.3f m\n', percentile_error);
fprintf('The RMSE of error is %.3f m\n', rmse_error);