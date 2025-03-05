% function [points3D] = make3D(points)
%
% this function maps a set of points from 2d to 3d. The given set of 2d
% points maps x values to x values, and y values to z values in 3d. All new
% points in 3d are placed along the y = 0.3 plane
%
% points = points in 2d to be mapped to 3d
%
% points3D = set of points to be plotted in 3d spelling out CSM
%
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [points3D] = make3D(points)

%
points3D = zeros(length(points), 3);

%
for i = 1:length(points)

    %
    points3D(i, :) = [points(i, 1), -0.3, points(i, 2)];

end

end