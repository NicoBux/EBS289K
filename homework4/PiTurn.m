function [turn] = PiTurn(current,next,row,sensitivity)

% DEFINITION:
% Generates waypoints on a pi turn given the current and next coordinates
% of the vehicle, the current row, and a sensitivity parameter.

% INPUTS: current = Coordinates of current pose; next = Coordinates of next
% pose; row = index of the current row; sensitivity = number of points to
% be generated at each section of the path.

% OUTPUTS: Path = Set of waypoints which the robot should traverse.

%% Explanation:
% Idea is to break down the maneuver into three sections: First turn,
% straight line and second turn. Calculates waypoints for all three
% sections and then concatenates them.

global RL N Rmin

if row >= N+2 % Upper headland
    if current(1)-next(1)>0 % Going right to left at an upper headland
        alpha = linspace(pi/2,0,sensitivity); % First turn
        x1 = Rmin*sin(alpha)+current(1)-Rmin;
        y1 = Rmin*cos(alpha)+RL;

        beta = linspace(0,-pi/2,sensitivity); % Second turn
        x3 = (Rmin*sin(beta))+next(1)+Rmin;
        y3 = Rmin*cos(beta)+RL;

        x2 = linspace(x1(end),x3(1),sensitivity); % Straight path
        y2 = linspace(y1(end),y1(end),sensitivity);

        turn = [x1 x2 x3; y1 y2 y3];
    else
        % Going left to right at an upper headland
        alpha = linspace(-pi/2,0,sensitivity); % First turn
        x1 = (Rmin*sin(alpha))+current(1)+Rmin;
        y1 = Rmin*cos(alpha)+RL;

        beta = linspace(0,pi/2,sensitivity); % Second turn
        x3 = (Rmin*sin(beta))+next(1)-Rmin;
        y3 = Rmin*cos(beta)+RL;

        x2 = linspace(x1(end),x3(1),sensitivity); % Straight path
        y2 = linspace(y1(end),y1(end),sensitivity);

        turn = [x1 x2 x3; y1 y2 y3];
    end
else % At an lower headland
    if current(1)-next(1)>0 % Right to left
        alpha = linspace(pi/2,pi,sensitivity); % First turn
        x1 = Rmin*sin(alpha)+current(1)-Rmin;
        y1 = Rmin*cos(alpha);

        beta = linspace(pi,3*pi/2,sensitivity); % Second turn
        x3 = (Rmin*sin(beta))+next(1)+Rmin;
        y3 = Rmin*cos(beta);

        x2 = linspace(x1(end),x3(1),sensitivity); % Straight path
        y2 = linspace(y1(end),y1(end),sensitivity);

        turn = [x1 x2 x3; y1 y2 y3];
    else % Left to right
        alpha = linspace(3*pi/2,pi,100); % First turn
        x1 = Rmin*sin(alpha)+current(1)+Rmin;
        y1 = Rmin*cos(alpha);

        beta = linspace(pi,pi/2,sensitivity); % Second turn
        x3 = (Rmin*sin(beta))+next(1)-Rmin;
        y3 = Rmin*cos(beta);

        x2 = linspace(x1(end),x3(1),sensitivity); % Straight path
        y2 = linspace(y1(end),y1(end),sensitivity);

        turn = [x1 x2 x3; y1 y2 y3];
    end
end
