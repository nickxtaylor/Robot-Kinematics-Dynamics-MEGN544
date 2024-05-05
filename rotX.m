% function [Rx] = rotX(theta) rotation about x axis
% 
% this function builds a 3x3 rotation matrix theta radians about the x axis
%
% Rx = 3x3 rotation matrix
%
% theta = angle in radians rotating about x axis
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [Rx] = rotX(theta)

% 
c = cos(theta);
s = sin(theta);

%
Rx = [1 0 0;
    0 c -s;
    0 s c];

end