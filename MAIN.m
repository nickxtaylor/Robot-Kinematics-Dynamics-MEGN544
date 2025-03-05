%% MEGN 544 PROJECT DELIVERABLE 1

% housekeeping 
clear all; clc; close all;

%% load points  

% load given points into workspace
load('points2D.mat');
 
% save variables from .mat file 
save('points2D','points_C','points_S','points_M','points_all')

% calculate scaling factor
x0 = -0.01; % final x value of starting location  
xf = 0.14; % final x value of ending location
scale = (0.14 - (-0.01)) / (points_all(end, 1) - points_all(1, 1));

%%

% rescale and translate points into correct x, z position
points_proj = orient(points_all, scale, -0.06, 0.4);

% make points 3D at y = -0.3 plane
points3D = make3D(points_proj);

% plot function
get_plot(points3D);

%%

%%

%%

%%

%%

%%

%%

%%

%%

%%

%%

%%

%%

%%

%%

%%