function new_frame = move_robot(old_frame,x,y,theta,tractor)
tractor(:,3)=1;
T = [cos(theta),- sin(theta),x;sin(theta),cos(theta),y;0,0,1];
for i=1:length(tractor)-1
new_frame(i,:) = T*tractor(i,:)';
end
center_frame = T*tractor(i+1,:)';

old_frame(i,1) = old_frame(1,1);
old_frame(i,2) = old_frame(1,2);

plot(old_frame(:,1),old_frame(:,2),'w');
hold on;
plot(new_frame(:,1),new_frame(:,2),'r');
%patch(new_frame(:,1),new_frame(:,2),'r');
plot(center_frame(1,1),center_frame(2,1),'ko');
xlim([-12 12]);
ylim([-12 12]);
%grid('on');
drawnow