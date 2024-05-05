% function [R] = rpy2Rot(roll, pitch, yaw) roll pitch yaw values to
% rotation
% 
% this function takes angles roll pitch and yaw and outputs 1 3x3 matrix
% which encompasses the 3 different rotations about the body x y z axis
%
% R = 3x3 rotation matrix
%
% roll = magnitude of rotation about body x hat
% pitch = magnitude of rotation about body y hat
% yaw = magnitude of rotation about body z hat
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [R] = rpy2Rot(roll, pitch, yaw)

%
R = rotZ(yaw) * rotY(pitch) * rotX(roll);

end