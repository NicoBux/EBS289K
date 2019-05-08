function move_robot(x,y,theta,tractor)

% DEFINITION
% Takes object (tractor), and current pose (x, y, theta) in order to
% animate the trajectory of the robot.

global route % Internal variable that stores the traversed path.
persistent oldframe % Local variable that stores the previous tractor
% pose (for animation purposes).

global n % Global variable that sets if the algorith should start a new frame

if isempty(n)
    n = 1;
end

if n == 1
    clf('reset');
end

if isempty(oldframe)
    oldframe = tractor;
end

% In order to allow rotate and translation modifies object shape
tractor(:,3) = 1;

% Transformation matrix
T = [cos(theta),- sin(theta),x;sin(theta),cos(theta),y;0,0,1];

for i=1:length(tractor)-1
    new_frame(i,:) = T*tractor(i,:)';
end

center_frame = T*tractor(length(tractor),:)'; % Keeps track of robot

oldframe(i,1) = oldframe(1,1);
oldframe(i,2) = oldframe(1,2);

% Erase the previous position
plot(oldframe(:,1),oldframe(:,2),'w');
hold on;

% Plots current position
plot(new_frame(:,1),new_frame(:,2),'r');

% Takes current position as the previous for the next integration
oldframe = new_frame;

% Set grids nice to see
xlim([-30 30]);
ylim([-30 30]);
axis square;
%grid('on');

% Updates the route vector (tractor motion)
route(n+1,1) = center_frame(1,1);
route(n+1,2) = center_frame(2,1);
n = n+1;
% Keep track of the center point of the robot
plot(route(2:n,1),route(2:n,2),'k-');

drawnow