function [omega] = virtualForceField(q,R,C,Xmax,Ymax,T,wSize,Fcr,Fct)

% DEFINITION:
% Function assumes an attractive force field and a repulsive force field
% around the robot and calculate the steering angle given for reactive
% navigation.

% INPUTS: q = robot pose [x,y,theta]; R and C = Rows and Columns
% -> discretization of physical space in a grid [pixels];
% Xmax and Ymax: Physical dimensions of space [m]; T = target location
% (x,y); wSize = search window size [m x m]; Fcr and Fa = Force constants.

% OUTPUTS:
% omega = Steering command [rad/s];

%% Code

global bitmap bitodds;
persistent omegap % Local variable that stores the previous issued steering
% command.

if isempty(bitmap)
    bitmap = zeros(R,C);
end

if isempty(omegap)
    omegap = 0;
end

x = q(1); y = q(2); theta = q(3);
wXmin = x-wSize(1); wYmin = y-wSize(2); wXmax = x+wSize(1); wYmax = y+wSize(2);
if wXmin<0
    wXmin = 0;
end
if wYmin <0
    wYmin = 0;
end
[iMin,jMin] = XYtoIJ(wXmin,wYmin,Xmax,Ymax,R,C);% SW corner of window
[iMax,jMax] = XYtoIJ(wXmax,wYmax,Xmax,Ymax,R,C);% NE corner of window
iMin2 = min(iMin,iMax); iMax2 = max(iMin,iMax);
jMin2 = min(jMin,jMax); jMax2 = max(jMin,jMax);

%% Calculation of Repulsive forcers

Frx = zeros(iMax2-iMin2,jMax2-jMin2);
Fry = zeros(iMax2-iMin2,jMax2-jMin2);
for i=iMin2:iMax2
    for j=jMin2:jMax2
        bitmap(i,j) = bitodds(i,j)/(1+bitodds(i,j));
        if bitmap(i,j) > 0.8 %meaning there's inddeed an object
            [wX,wY] = IJtoXY(i,j,Xmax,Ymax,R,C); %coordinates in m of current position of the window
            dx = wX-x;
            dy = wY-y;
            d = sqrt(dx*dx+dy*dy); %distance from current window pixel to the robot in m
            Frx(i,j) = Fcr*bitmap(i,j)/(d*d)*(dx/d); %force component in xhat
            Fry(i,j) = Fcr*bitmap(i,j)/(d*d)*(dy/d); %force component in yhat
        end
    end
end
Fr(1) = sum(Frx(:)); %sum of force in xhat
Fr(2) = -sum(Fry(:)); %sum of force in yhat

%w = 1.01; %force gain;
%Fr(1) = w*Fr(1)+(1-w)*Fr(1)*-cos(theta); %fine tune of forces
%Fr(2) = w*Fr(2)+(1-w)*Fr(2)*-cos(theta); %fine tune of forces

%% Calculation of Attraction forcers

xT = T(1); yT = T(2);
dxT = (xT-x); dyT = (yT-y);
dT = sqrt(dxT*dxT+dyT*dyT);
Fax = Fct*dxT/dT; %sum of force in xhat
Fay = Fct*dyT/dT; %sum of force in yhat

FT(1) = Fax+Fr(1); %Resultant in xhat
FT(2) = Fay+Fr(2); %Resultant in yhat

%% Calculation of Resultant Force and Steering Command

mag = sqrt(FT(1)*FT(1)+FT(2)*FT(2)); %Force magnitude
Fx = FT(1)/mag;
Fy = FT(2)/mag;

delta = atan2(Fy,Fx);
Ks = 4; % Steering gain
omega = Ks*angdiff(delta,theta);
T = 0.1; %sampling rate;
tal = 0.1; %time constant of the lowpass filter
omega = (T*omega+(tal-T)*omegap)/tal;
omegap = omega;
