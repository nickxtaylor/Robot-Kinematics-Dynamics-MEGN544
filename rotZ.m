% function [Rz] = rotZ(theta) rotation about z axis
% 
% this function builds a 3x3 rotation matrix theta radians about the z axis
%
% Ry = 3x3 rotation matrix
%
% theta = angle in radians rotating about z axis
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [Rz] = rotZ(theta)

% 
c = cos(theta);
s = sin(theta);

%
Rz = [c -s 0;
    s c 0;
    0 0 1];

end