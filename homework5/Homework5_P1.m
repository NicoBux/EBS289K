%% EBS289K - Agricultural Robotics and Automation - Spring 2019 
% Homework Assignment #5 - Part1: Occupancy Grids
% Students: Guilherme De Moura Araujo & Nicolas Buxbaum
% Professor: Stavros Vougioukas

%% Start
clear all; clc; close all;
addpath(genpath('geom2d'));
%% Code
global bitmap bitodds;
global rangeMax;

%lidar values
rangeMax = 200; angleSpan = pi; angleStep = angleSpan/720; 

Xmax = 150; Ymax = 150; %physical dimensions of space (m)

R = 500; C = 500; %rows and columns; discretization of physical space in a grid
map=zeros(R, C);
bitmap = 0.0* ones(R, C); %initialize as empty

% Create test rectangular obstacle
Xsw=70; Ysw = 30;
Xne=Xsw + 30; Yne= Ysw + 20;
[Isw, Jsw] = XYtoIJ(Xsw, Ysw, Xmax, Ymax, R, C);
[Ine, Jne] = XYtoIJ(Xne, Yne, Xmax, Ymax, R, C);
map(Ine:Isw, Jsw:Jne) = 1;

%select locations to use the lidar sensor
corners = [5,5,pi/4;5,Ymax-5,-pi/4;Xmax-5,5,3*pi/4;Xmax-5,Ymax-5,-3*pi/4];
centers = [Xmax/2,5,pi/2;5,Ymax/2,0;Xmax/2,Ymax-5,-pi/2;Xmax-5,Ymax/2,pi];
points = [corners;centers]; % 8 points strategically spread in the field
%plot(points(:,1),points(:,2),'o')

%updates the robot map (bitmap)

for k=1:length(points)
    Tl = SE2([points(k,1) points(k,2) points(k,3)]);
    p = laserScanner(angleSpan, angleStep, rangeMax, Tl.T, map, Xmax, Ymax);  
    for i=1:length(p)
        angle = p(i,1); range = p(i,2);
        % handle infinite range
        if(isinf(range)) 
            range = rangeMax+1;
        end
        n = updateLaserBeamGrid(angle, range, Tl.T, R, C, Xmax, Ymax, 0.95);
    end
end

for i = 1:length(points)
    [points(i,1),points(i,2)] = XYtoIJ(points(i,1),points(i,2),Xmax,Ymax,R,C);
end

for i=1:R
    for j=1:C
        bitmap(i,j) = bitodds(i,j)/(1+bitodds(i,j));
    end
end
%% Post processing image analysis

% The idea here is to apply image processing techniques to filter the 
% original robot map, aiming to obtain a better representation of the
% environment.

se = strel('disk',1,0);
bitpos = imopen(bitmap,se);
subplot(1,2,2)
imagesc(bitpos);
colorbar
hold on
plot(points(:,1),points(:,2),'ro')
axis square
title('Processed image');
legend('Laser Scanner locations');

subplot(1,2,1)
imagesc(bitmap)
colorbar
hold on
plot(points(:,1),points(:,2),'ro')
axis square
title('Original image');
legend('Laser Scanner locations');