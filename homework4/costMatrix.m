function DMAT = costMatrix(N,Rmin,W,x,y)

%% Compute non-turning costs aij 
%Create cost table DMAT
%Moving from ANY lower headland to ANY upper headland
%Only possible if both nodes are in the same field row

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
        if(Rmin<=d*W/2) %PI Turn
            DMAT(i,j) = d*W + ((pi-2)*Rmin);
        else %omega turn
            cost = ((2*Rmin+d*W)^2)/(8*Rmin^2);
            DMAT(i,j) = 3*pi*Rmin - 2*Rmin*acos(1-cost);
        end
        DMAT(j,i) = DMAT(i,j);
        DMAT(i+N,j+N) = DMAT(i,j); %make sure the corresponding ones are equal as well
        DMAT(j+N,i+N) = DMAT(i,j); %make sure the corresponding ones are equal as well
    end
end
%% Compute costs aij involving start/end nodes
%Approximate using the Manhattan distance
%|x1-x2|+|y1+y2|
for i = 2:2*N+1
    DMAT(1,i) = abs(x(1)-x(i))+abs(y(1)-y(i));
    DMAT(i,1) = DMAT(1,i);
    DMAT(2*N+2,i) = abs(x(2*N+2)-x(i))+abs(y(2*N+2)-y(i));
    DMAT(i,2*N+2) = DMAT(2*N+2,i);
end
%% Make start and end nodes big
DMAT(1,2*N+2) = huge; %going from 1 - 2*N+2 huge
DMAT(2*N+2,1) = huge; %going from 2*N+2 - 1 huge