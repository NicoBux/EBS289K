function path = Straight(current,next,sensitivity,p)

% DEFINITION:
% Generates waypoints on a straight line given the current and next
% position of the vehicle, a sensitivity parameter and the information if a
% previous pi turn happened.

% INPUTS: current = Coordinates of current pose; next = Coordinates of next
% pose; sensitivity = number of points to be generated at each section of
% the path; p = boolean to check a pi turn was made previously.

% OUTPUTS: Path = Set of waypoints which the robot should traverse.

%% Calculations

if p == 0
    path = [current(1)*ones(1,sensitivity);linspace(current(2),next(2),sensitivity)];
else
    path = [next(1)*ones(1,sensitivity);linspace(current(2),next(2),sensitivity)];
end