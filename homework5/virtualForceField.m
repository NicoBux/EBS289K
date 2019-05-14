function p = virtualForceField(angle, range, Tl, R, C, Xmax, Ymax, a)

% DEFINITION:
% Function assumes a laser scanner with a pose in world coordinates defined
% by Tl. It shoots rays from -angleSpan/2 to +angleSpan/2 with step
% angleStep. Given a bitmap (boolean occupancy grid with obstacles) with
% origin at (0,0) and uper NE corner (Xmax, Ymax), the result is an
% occupancy map with probabilities of pixel being full.

% INPUTS: angle = angle span from laser scanner [deg]; range = maximum
% distance reached by the laser scanner [m]; Tl = Transfrom from laser
% scanner to world frame; R and C = Rows and Columns -> discretization of
% physical space in a grid [pixels]; Xmax and Ymax: Physical dimensions of
% space [m]; a = sensor accuracy.

% OUTPUTS:
% p = Number of updated pixels; bitmap = occupancy grid of field

%% 
% x and y are robot coordinates in the world frame, wX and wY are window
% size in (min-max) in world frame; wij are window size in pixels; robotij
% are robot coordinates in pixels
wSize = [3.3 3.3];
x = 1;
y = 2;
%[roboti,roboti] = XYtoIJ(x,y,Xmax,Ymax,R,C);
wXmin = x-1.65; wYmin = y-1.65; wXmax = x+1.65; wYmax = y+1.65;
[wijMin(1,1),wijMin(1,2)] = XYtoIJ(wXmin,wYmin,Xmax,Ymax,R,C);
[wijMax(1,1),wijMax(1,2)] = XYtoIJ(wXmax,wYmax,Xmax,Ymax,R,C);

%% Calculation of Repulsive forcers
for i=wijMin:wijMax
    for j=wijMin:wijMax
        if bitmap(i,j) > 0.8 %meaning there's an object
            [wX,wY] = IJtoXY(i,j,Xmax,Ymax,R,C); %coordinates in m of current position of the window
            dx = abs(wX-x);
            dy = abs(wY-y);
            d = sqrt(dx*dx+dy*dy); %distance from current window pixel to the robot in m
            F(i,j,1) = Fcr*bitmap(i,j)/(d*d)*(dx/d); %force component in xhat
            F(i,j,2) = Fcr*bitmap(i,j)/(d*d)*(dy/d); %force component in yhat
        end
    end
end
Fr(1) = sum(F(:,:,1)); %sum of force in xhat
Fr(2) = sum(F(:,:,2)); %sum of force in yhat

%% Calculation of Attraction forcers
for i=wijMin:wijMax
    for j=wijMin:wijMax
        if bitmap(i,j) > 0.8 %meaning there's an object
            [wX,wY] = IJtoXY(i,j,Xmax,Ymax,R,C); %coordinates in m of current position of the window
            dx = abs(wX-x);
            dy = abs(wY-y);
            d = sqrt(dx*dx+dy*dy); %distance from current window pixel to the robot in m
            F(i,j,1) = Fcr*bitmap(i,j)/(d*d)*(dx/d); %force component in xhat
            F(i,j,2) = Fcr*bitmap(i,j)/(d*d)*(dy/d); %force component in yhat
        end
    end
end
Fr(1) = sum(F(:,:,1)); %sum of force in xhat
Fr(2) = sum(F(:,:,2)); %sum of force in yhat
