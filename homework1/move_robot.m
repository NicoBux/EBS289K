function [new_frame, proute] = move_robot(old_frame,route,x,y,theta,tractor)
%% Comments
% Takes object (tractor), past (old_frame) and current pose (x, y, theta)
% in order to simulate the movement of the robot

% In order to allow rotate and translation modifies object shape
tractor(:,3)=1;

% Transformation matrix
T = [cos(theta),- sin(theta),x;sin(theta),cos(theta),y;0,0,1];

for i=1:length(tractor)-1
new_frame(i,:) = T*tractor(i,:)';
end

center_frame = T*tractor(length(tractor),:)';

old_frame(i,1) = old_frame(1,1);
old_frame(i,2) = old_frame(1,2);

% Erase the previous position
plot(old_frame(:,1),old_frame(:,2),'w');
hold on;

% Plots current position
plot(new_frame(:,1),new_frame(:,2),'r');
%patch(new_frame(:,1),new_frame(:,2),'r');
%plot(center_frame(1,1),center_frame(2,1),'ko');

xlim([-12 12]);
ylim([-12 12]);
%grid('on');
proute = route;
proute(length(route)+1,1) = center_frame(1,1);
proute(length(route)+1,2) = center_frame(2,1);

% Keep track of the center point of the robot
plot(proute(:,1),proute(:,2),'k-');

drawnow