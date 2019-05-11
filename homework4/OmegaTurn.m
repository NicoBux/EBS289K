function path = OmegaTurn(current,row,nextrow)

% DEFINITION:
% Generates waypoints on an omega turn given the current and next
% coordinates of the vehicle, and the current and next rows.

% INPUTS: current = Coordinates of current pose; next = Coordinates of next
% pose; row = index of the current row; nextrow = index of the next row.

% OUTPUTS: Path = Set of waypoints which the robot should traverse.

%% Explanation:
% Idea is to break down the maneuver into three sections: First turn,
% second turn and third turn. Calculates waypoints for all three
% sections and then concatenates them.

global N Rmin W sensitivity;

if row >= N+2 % Upper headland
        xstart = current(1);
        ystart = current(2); 
else % Lower headland
        xstart = current(1);
        ystart = current(2); 
end 
    
gamma = acos(1-((2*Rmin+W)^2)/(8*Rmin^2)); % Angle between centers of all 3 turn segments 
alpha = (pi - gamma)/2; % Corner angles of base of triangle connecting centers of all 3 turn segments 

a1 = alpha/sensitivity; % Angular step size for the first part of the turn 
a2 = (2*pi - gamma)/sensitivity; % Angular step size for the second part of the turn 
a3 = a1; % Angular step size for the third part of the turn 

% Calculate sign for path calculations (assuming CLOCKWISE turn as default)
if row > N+1 % Positive if at an upper headland
    sign = 1; 
else % Negative if at an lower headland
    sign = -1; 
end 

% First turn
j = 1;
turn1x = zeros(1, length(0:a1:alpha));
turn1y = turn1x;
for i = 0:a1:alpha %calculate (x,y) for first segment of pi turn 
    turn1x(j) = xstart - sign*Rmin + sign*Rmin*(cos(i)); 
    turn1y(j) = ystart + sign*Rmin*sin(i); 
    j = j + 1; 
end 

% Second turn
j = 1;
turn2x = zeros(1, length((3*pi/2 - gamma/2):-a2:(-pi/2 + gamma/2)));
turn2y = turn2x;
for i = (3*pi/2 - gamma/2):-a2:(-pi/2 + gamma/2)
    turn2x(j) = xstart + sign*W/2 + sign*Rmin*cos(i); 
    turn2y(j) = ystart + sign*2*Rmin*sin(alpha) + sign*Rmin*sin(i); 
    j = j + 1; 
end

% Third turn
j = 1;
turn3x = zeros(1, length(alpha:-a3:0));
turn3y = turn3x;
for i = alpha:-a3:0 
    turn3x(j) = xstart + sign*W + sign*Rmin - sign*Rmin*cos(i); 
    turn3y(j) = ystart + sign*Rmin*sin(alpha) - sign*Rmin*sin(alpha - i); 
    j = j + 1; 
end 

xpoints = [turn1x'; turn2x'; turn3x']; 
ypoints = [turn1y'; turn2y'; turn3y']; 

% Inverts vector around if turn is counterclockwise
if (row >= (N+2)) && (nextrow < row) 
    for i = 1:length(xpoints)
        xpoints(i) = xpoints(1) + (xpoints(1) - xpoints(i)); 
    end 
elseif (row <= (N+1)) && (nextrow > row) 
    for i = 1:length(xpoints) 
        xpoints(i) = xpoints(1) + (xpoints(1) - xpoints(i)); 
    end 
end
path = [xpoints, ypoints];
