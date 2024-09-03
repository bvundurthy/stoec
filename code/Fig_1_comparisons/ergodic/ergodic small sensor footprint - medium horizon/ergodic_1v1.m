close all
clear variables
clc

L1 = 100;
L2 = 100; 

workspace_bounds = [0 L1; 
                    0 L2];  

xDelta = 1; 
yDelta = 1; 
xRange = 0: xDelta: L1 - xDelta; 
yRange = 0: yDelta: L2 - yDelta; 

[X, Y] = meshgrid(xRange, yRange); 

peak1_center = [50 50]; 
peak1_covari = 5*eye(2); 

% peak2_center = [80 60]; 
% peak2_covari = 70*eye(2); 
% 
% peak3_center = [30 80];
% peak3_covari = 50*eye(2); 

gauss1 = mvnpdf([X(:), Y(:)], peak1_center, peak1_covari); 
info_map = reshape(gauss1, size(X)); 

figure (1)
% hold on
mesh(X, Y, info_map); 

figure (2)
hold on
contour(info_map)
