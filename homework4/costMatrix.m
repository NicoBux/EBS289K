function DMAT = costMatrix(N,W,nodes)

% DEFINITION:
% Creates a cost matrix for the robot traversal over a path based on pi
% turn and omega turn maneuvers.

% INPUTS: N = Number of rows; nodes = Set of coordinates which the robot
% has to traverse.

% OUTPUTS: DMAT = Cost matrix

%% Variables
global Rmin

if size(nodes,1) == 2
    nodes = nodes';
end

x = nodes(:,1); y = nodes(:,2);
%% Compute non-turning costs aij

% Moving from any lower headland to any upper headland
% Only possible if both nodes are in the same row

huge = 1E10;
DMAT = zeros(2*N+2,2*N+2);

for i = 2:N+1
    for j = N+2:2*N+1
        if (j-i) == N
            DMAT(i,j) = -huge/2;
            DMAT(j,i) = -huge/2;
        else
            DMAT(i,j) = huge;
            DMAT(j,i) = huge;
        end
    end
end

%% Compute turning costs

for i = 2:N
    for j = i+1:N+1
        d = abs(i-j);
        if(Rmin<=d*W/2) % Pi turn
            DMAT(i,j) = d*W + ((pi-2)*Rmin);
        else % Omega turn
            cost = ((2*Rmin+d*W)^2)/(8*Rmin^2);
            DMAT(i,j) = 3*pi*Rmin - 2*Rmin*acos(1-cost);
        end
        DMAT(j,i) = DMAT(i,j);
        DMAT(i+N,j+N) = DMAT(i,j);
        DMAT(j+N,i+N) = DMAT(i,j);
    end
end

%% Compute costs aij involving first and last nodes
% Approximate using the Manhattan distance: |x1-x2|+|y1+y2|
for i = 2:2*N+1
    DMAT(1,i) = abs(x(1)-x(i))+abs(y(1)-y(i));
    DMAT(i,1) = DMAT(1,i);
    DMAT(2*N+2,i) = abs(x(2*N+2)-x(i))+abs(y(2*N+2)-y(i));
    DMAT(i,2*N+2) = DMAT(2*N+2,i);
end
%% Make start and end nodes big
DMAT(1,2*N+2) = huge; %going from 1 to 2*N+2 huge
DMAT(2*N+2,1) = huge; %going from 2*N+2 to 1 huge