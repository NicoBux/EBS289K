function [angle, error] = purePursuitController(q, L, Ld, path)

error = Inf;
row_close = 0;
row_goal = 0;
for i=1:length(path)
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
for i=row_close+1:length(path) % start search one point after the closest point 
    dx = path(row_close,1)-path(i,1);
    dy = path(row_close,2)-path(i,2);
    di = sqrt(dx*dx + dy*dy);
    err = abs(Ld-di);
    if err<error
        error = err;
        row_goal = i;
    end
end

x = q(1);
y = q(2);
theta = q(3);
T = [cos(theta),- sin(theta),x;sin(theta),cos(theta),y;0,0,1]; % Transformation matrix based on vehicle position (q vector) 

goal = [path(row_goal,1);path(row_goal,2);1]

goal_T = T * goal

k = (2*goal_T(2))/(Ld^2);
gamma = atan(k*L)



