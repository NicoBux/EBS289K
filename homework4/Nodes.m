function nodes = Nodes(N,W,RL,start,final,row_start,nodes_per_row)

% DEFINITION:
% Takes various inputs to create nodes at field rows.

% INPUTS:
% N = number of rows; W = row width [m]; LD = row length [m]; start =
% initial vehicle position; final = final vehicle position; row_start =
% begin of the field to be cultivated; nodes_per_row = nodes per row
% (either 2 or 3).

% OUTPUTS:
% nodes = geographic coordinates of the nodes in the field rows.
%% 
xstart = start(1,1); ystart = start(1,2);
xend = final(1,1); yend = final(1,2);
xrow = row_start(1,1); yrow = row_start(1,2);

if nodes_per_row == 2
    x = [xstart, xrow:W:(N-1)*W+xrow, xrow:W:(N-1)*W+xrow, xend];
    y = [ystart, yrow*ones(1,N), (RL+yrow)*ones(1,N), yend];
    nodes = [x;y].';
end

if nodes_per_row == 3
    x = [xstart, xrow:W:(N-1)*W+xrow, xrow:W:(N-1)*W+xrow, xrow:W:(N-1)*W+xrow, xend];
    y = [ystart, yrow*ones(1,N), (yrow+RL/2)*ones(1,N), (RL+yrow)*ones(1,N), yend];
    nodes = [x;y].';
end