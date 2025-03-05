% function [T] = dhTransform(a, d, alpha, theta) takes a given row or DH
% paramters and outputs a transformation matrix to get from one node to
% the next
%
% T = 4x4 transformation matrix from link i to link i+1
%
% a = translation along x hat
% d = trranslation along z hat
% alpha = rotation about x hat
% theta = rotation along z hat
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [T] = dhTransform(a, d, alpha, theta)

% 
transz = [eye(3), [0 0 d]';
    0 0 0 1]; 
transx = [eye(3), [a 0 0]';
    0 0 0 1];
Rz = [rotZ(theta), [0 0 0]';
    0 0 0 1];
Rx = [rotX(alpha), [0 0 0]';
    0 0 0 1];

%
T = transz * Rz * transx * Rx;

end