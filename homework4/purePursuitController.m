function [gamma, error] = purePursuitController(q, L, Ld, path)

% DEFINITION:
% Pure Pursuit Controller, calculates the steering angle at which the
% vehicle should turn in order to achieve the waypoints in a pre-defined
% path.

% INPUTS:
% q(1) = x; q(2) = y; q(3) = theta; q(4) = velocity; q(5) = gamma ->
% current state.
% L = vehicle wheel base [m]; Ld = Look ahead distance [m]; path = set of
% geographic coordinates of the waypoints in a path.

% OUTPUTS:
% gamma = steering angle (rad); error = error from the robot to the closest
% waypoint on the path.


persistent previous; % Local variable used to store the last goal point

if isempty(previous)
    previous = 1; % Initializes the variable
end

%% Find nearest path point

% Converts path to robot coordinate frame
T = transl2(q(1),q(2))*trot2(q(3)); % Transformation from robot to world
T = inv(T); % Inverse is transformation from world to robot frame
path(:,3)=1; % Algebrae trick to allow the multiplication of matrices
robotFrame = T*path'; %Final path in robot frame coordinates

%% Find the goal point (Look ahead distance)

nP = length(path); % Find the number of points in Path
first = previous;
range = 15; % In order to optimize the search for the goal point and do not
% allow the algorithm to look for waypoints that are further ahead of the
% current position, a range of feasible waypoints is defined.

startIndex = max(first-range, 1); % The begin of the search is either the
% first waypont or the previous goal point minus the range.
endIndex = min(nP, first+range); % It finishes the search either from the
% total length of the path or from the previous goal point plus the range.

err = inf; row_goal = 0;
di = zeros(1,nP);
distance = zeros(1,nP);

for i = startIndex:endIndex % Reduced the range for the search of goal point
    dx = robotFrame(1,i); dy = robotFrame(2,i); 
    di(i) = sqrt(dx*dx+dy*dy); % Find distance of robot(0,0)to points in
    %path (in robot frame)
    distance(i) = abs(di(i) - Ld); % Idea is to find a waypoint such that
    %the distance from this waypoint to the robot = Look ahead
    dmin = min(di(di>0)); %Selects point that minimizes (di-Ld)
    
    if (dx > 0) && (di(i) < Ld) && (distance(i) <= err) %Check if point is
        %in front of robot
        row_goal = i; % this index represents the look ahead point (goal)
    end
    err = distance(i); % If the point is feasible then updates the error
    %tracking
end

% If no points is in front of robot, move to the closest point
if row_goal == 0 
    ey = dmin;
else 
    ey = robotFrame(2,row_goal);
end

%% Calculate OUTPUTS

gamma = atan(2*ey*L/Ld^2);
error = dmin; % Tracking error
previous = row_goal; % Updates the position of the previous goal point to
%refine the search in the next iteration.