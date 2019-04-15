function tractor = draw_tractor(w,l,r)
vehicle(1,1) = 0; % lower left
vehicle(1,2) = 0;
vehicle(2,1) = 0; % upper left
vehicle(2,2) = w;
vehicle(3,1) = l; % upper right
vehicle(3,2) = w;
vehicle(4,1) = l;
vehicle(4,2) = 0; % lower right;
vehicle(5,1) = 0;
vehicle(5,2) = 0;
vehicle(6,1) = l/2;
vehicle(6,2) = w/2;


% w = 2.0;
% l = 6;
theta = deg2rad(270);
T = [cos(theta),- sin(theta);sin(theta),cos(theta)];

e = 0.3;
[tractor] = [0,0;0,0.75*l;0.3*w,0.75*l;0.3*w,l;0.7*w,l;0.7*w,0.75*l;w,0.75*l;w,0;0,0;0,0];
tractor (:,1) = tractor(:,1) - w/2;
tractor (:,2) = tractor(:,2)-l/2;

[wheels] = [0,0;0.3*w,l;0.7*w,l;w,0];
for i=1:9
    tractor(i,:) = T*tractor(i,:)'; 
end
tractor(10,1) = 0;
tractor(10,2) = 0;
for i=1:4
    wheels(i,:) = T*wheels(i,:)';
end
x=tractor(:,1);
y=tractor(:,2);
plot(x,y,'r-');
hold on;
% xw=wheels(:,1);
% yw=wheels(:,2);
% plot(xw,yw,'ko');

% 
% [wheels] = [0,r/2;-e,r/2;-e,-r/2;0,-r/2;0,0];
% xw = wheels(:,1);
% yw = wheels(:,2);
% plot(xw,yw,'r-');

[wheels2] = [w,r/2;w+e,r/2;w+e,-r/2;w,-r/2;w,0];
xw2 = wheels2(:,1);
yw2 = wheels2(:,2);
% plot(xw2,yw2,'r-');

[wheels3] = [0.3*w,l-r/2;0.3*w-e,l-r/2;0.3*w-e,l+r/2;0.3*w,l+r/2;0.3*w,l];
xw3 = wheels3(:,1);
yw3 = wheels3(:,2);
% plot(xw3,yw3,'r-');

[wheels4] = [0.7*w,l-r/2;0.7*w+e,l-r/2;0.7*w+e,l+r/2;0.7*w,l+r/2;0.7*w,l];
xw4 = wheels4(:,1);
yw4 = wheels4(:,2);
% plot(xw4,yw4,'r-');