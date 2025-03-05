% function function [Ry] = rotY(theta) rotation about y axis
% 
% this function builds a 3x3 rotation matrix theta radians about the y axis
%
% Ry = 3x3 rotation matrix
%
% theta = angle in radians rotating about y axis
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [Ry] = rotY(theta)

% 
c = cos(theta);
s = sin(theta);

%
Ry = [c 0 s;
    0 1 0;
    -s 0 c];

end