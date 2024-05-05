% function [R] = quat2Rot(Q) maps quartenion to rotation
% 
% this function takes a quaternion and maps it to a 3x3 rotation matrix
%
% R = 3x3 rotation matrix
%
% Q = quaternion
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [R] = quat2Rot(Q)

%
q0 = Q(1);
q = Q(2:4);

%
R = (q0^2 - (q' * q)) * eye(3) + 2 * q0 * cpMap(q) + 2 * (q * q');

end