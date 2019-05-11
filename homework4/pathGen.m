function path = pathGen(route,nodes)

% DEFINITION:
% Generates waypoints based on a given optimal route, nodes and a
% sensitivity parameter.

% INPUTS: N = Number of rows; route = Optimal route given by tspof_ga;
% sensitivity = number of points to be generated at each section of the
% path; nodes = geographic coordinates of the upper and lower headlands of
% the field.

% OUTPUTS: Path = Set of waypoints which the robot should traverse.

%% Path making
global N;

path = [];

for i = 1:length(route)-1
    if i<length(route)
        d = route(i)-route(i+1); % Tracking of relative position
        current = nodes(route(i),:); % Current node coordinates
        next = nodes(route(i+1),:); % Next node coordinates
    else
        d = 0;
        current = nodes(route(i),:);
        next = 0; % In case the current node is the last one
    end
    
    if i == 1 % First node
            straight = Straight(current,next,0); % Straigh line until turn
            turn = PiTurn(current,next,route(i+1)); % Gets waypoints for a pi turn
            path = [straight turn]; % Concatenates the straight line with the turn
       
    elseif i == length(route)-1 % Last but one node - This will lead to the final node
        turn = PiTurn(current,next,route(i));
        straight = Straight(current,next,1);
        pathn = [turn straight];
        path = [path pathn];
    else % All other nodes
        if abs(d)== N % Always when abs(d) = N we are traveling between nodes in a single row (straight line)
            pathn = Straight(current,next,0);
        else
            if abs(route(i)-route(i+1))== 1 % Case in which the turning radius is less than the w*d - In this case two nodes in consecutive rows (condition in which an omega turn is used)
                pathn = OmegaTurn(current,route(i),route(i+1));
                pathn=pathn';
            else % If not straight nor Omega turn then Pi turn
                pathn = PiTurn(current,next,route(i+1));
            end
        end
        path = [path pathn];
    end
    
end