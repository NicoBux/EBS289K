function nodes = Nodes(N,W,RL, nodes_per_row)

% DEFINITION:
% Takes various inputs to create nodes at field rows.

% INPUTS:
% N = number of rows; W = row width [m]; LD = row length [m];
% nodes_per_row = nodes per row (either 2 or 3).

% OUTPUTS:
% nodes = geographic coordinates of the nodes in the field rows.
%% 
if nodes_per_row == 2
    x = [-W, 0:W:(N-1)*W, 0:W:(N-1)*W, -W];
    y = [3/2, zeros(1,N), RL*ones(1,N), 3/2];
    nodes = [x;y].';
end

if nodes_per_row == 3
    x = [-W, 0:W:(N-1)*W, 0:W:(N-1)*W, 0:W:(N-1)*W, -W];
    y = [3/2, zeros(1,N), RL/2*ones(1,N), RL*ones(1,N), 3/2];
    nodes = [x;y].';
end