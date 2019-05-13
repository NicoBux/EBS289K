
%clear all; 
global tractor; % just a simple geometric model
global dT; %integration interval
global DT; %control interval
global epsilon_dist; %distance from goal point considered close enough to stop robot (goal reached)
global epsilon_speed; %almost zero speed

% state is 
% q(1) -> x
% q(2) -> y
% q(3) -> theta (orientation in world frame)
% q(4) -> phi (steering angle)
% q(5) -> linear velocity

% control inputs are:
% u(1) -> desired steering angle
% u(2) -> desired linear velocity

epsilon_dist = 0.1; % (m)
epsilon_speed = 0.02; % (m/s)
STEER_INPUT = 1; SPEED_INPUT=2; %just a name instead of using index 
ROBOT_SPEED = 5; % speed is 5th element of state vector
dT = 0.005; % (s) model integration step
DT =  0.01; % (s) controller integration step
T = 60.0; % simulation time (s)
N = T/DT; % simulation steps

q = zeros(5, N); % state vectors over N time steps
u = zeros(2, N); % input vectors over N time steps
e = zeros(1, N); %cross track error

%vehicle parameters
buildTractor; % populates tractor array with points (a triangle!)
L = 2.5 ; % robot wheelbase (distance between front an rear wheels) in meters; triangle doesn't match this...
Vmax = 1;  %maximum vehicle speed (m/s)
gammaMax = 45*pi/180;  % max steering angle (rads)
tau_g = .15; % steering time lag (s)
tau_v = 0.5; % velocity time lag (s)

% state constraints
Qmax(1) = Inf; Qmax(2) = Inf; Qmax(3) = Inf; 
Qmax(4) = gammaMax; Qmax(5) = Vmax;
Qmin = -Qmax; % symmetrical negative constraints for minimum values

% Control constraints
Umax=[gammaMax Vmax]';
Umin= - Umax;

% disturbances 
delta1 = -0.0; delta2 = 0.0; slip = 0.0;

% initial state
q(1,1)=10; q(2,1)=3; q(3,1)=pi/4; 

% DESIRED GOAL POINT
xd = 5.0;  yd= 5.0;  %(m)

%controller gains
Kv = 0.8; % speed gain
Kh = 6; % heading gain

figure(1); hold on; axis equal; wTr=eye(3,3); wTr_old=wTr;
k=1;   % time step
for t=0:DT:T-DT
   %navigation controller
   ex = xd - q(1,k);  ey = yd - q(2,k); % instantaneous errors in x and y axes
   th_d = atan2(ey, ex); % desired  direction of motion to reach xd,yd
   if (abs(ex) + abs(ey) > epsilon_dist)  %move if robot position is not close enough to the goal position
       u(SPEED_INPUT,k) = Kv * sqrt(ex^2 + ey^2); %(m/s)
   else
       u(SPEED_INPUT,k)=0;
   end
   u(STEER_INPUT,k) = Kh * angdiff(th_d, q(3,k)); %heading error (rads)
  
    %send speed and angle setpoints to low-level steer and speed controllers
    % and integrate kinematic model for DT
    q(:, k+1) = robot_bike_dyn(q(:,k), u(:,k), Umin, Umax, Qmin, Qmax, L, tau_g, tau_v, delta1, delta2, slip);
    
    plot(q(1,k), q(2,k),'.');  % plot trace of robot's frame origin
    k=k+1; %increase time step index
   if (abs(ex) + abs(ey) < epsilon_dist && q(ROBOT_SPEED,k) < epsilon_speed) %command robot to stop
       break  % exit main loop (stop moving) because robot is close enough to the goal position
   end
   % animate tractor
      wTr=transl2(q(1,k),q(2,k))*trot2(q(3,k));
      %SE2([q(1,k) q(2,k) q(3,k)]); % transformation of pose at step k
      plotTractor(tractor, wTr_old, 'w'); %erase previously plotted robot at step k-1 (draw in white color!)
      plotTractor(tractor, wTr, 'r'); pause(0.001); % draw robot at k step
      wTr_old = wTr; %keep the transformation at pose k, to erase it in the next time step.
end
%draw tractor at initial pose (it was deleted during animation)
wTr=transl2(q(1,1),q(2,1))*trot2(q(3,1));
%SE2([q(1,1) q(2,1) q(3,1)]);
plotTractor(tractor, wTr, 'r'); 

X=sprintf('Errors (cm): x = %.2f, y = %.2f \nError distance = %.2f', 100*ex, 100*ey, 100*sqrt(ex^2+ey^2));
disp(X);


