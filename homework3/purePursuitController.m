function [gamma, error] = purePursuitController(q, L, Ld, path)
global n
% Find nearest path point
first = n-1;
previousIndex = n-1;
if first<=0
    first =1;
    previousIndex = 1;
end
startMin = Inf;
stride = 12; %nP = 35, %pindex = 13;
endI = min(length(path), previousIndex+stride); %25
startI = max(n-1-stride, 1); % 12

error = Inf;
row_close = 0;
row_goal = 0;

for i=startI:endI
    dx = q(1)-path(i,1);
    dy = q(2)-path(i,2);
    di = sqrt(dx*dx + dy*dy);
    if di < error
        error = di;
        row_close = i; % Path matrix row value of closest path point
    end
end

% look for the next (forward) point on the path closest to the lookahead
% distance
% optional optimzation would be to stop looking for points once two
% consectuive err's are increasing 
error = Inf;

for i=startI:endI % start search one point after the closest point 
    dx = path(row_close,1)-path(i,1);
    dy = path(row_close,2)-path(i,2);
    di = sqrt(dx*dx + dy*dy); %di now is the distance between path[x,y] and
    %the nearest point to the robot.We want this distance to be equal Ld
    err = abs(Ld-di); %Ld-di should be 0.
    %if i>=n %Don't consider points that we already traveled by
            if err<error %Find path[x,y] that minimizes (Ld-di).
            error = err;
            row_goal = i;
            end
    %end
end

x = q(1);
y = q(2);
% ey = abs(path(row_goal,2)-y);
% k = 2*ey/(Ld*Ld);
% gamma = atan(k*L);
theta = q(3);
T = [cos(theta),- sin(theta),x;sin(theta),cos(theta),y;0,0,1]; % Transformation matrix based on vehicle position (q vector) 
T = inv(T);
 goal = [path(row_goal,1);path(row_goal,2);1];
% 
 goal_T = T * goal;
% 
 k = (2*goal_T(2))/(Ld^2);
 gamma = atan(k*L);



