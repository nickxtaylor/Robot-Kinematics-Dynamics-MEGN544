% function [dual_quat] = pq2DualQuat(pos, quat) quat and disp to dual_quat
% 
% this function takes a quaternion mapping to a rotation and a given
% position vector and turrns that into a dual quaternion in a structure
% form
%
% dual_quat = dual quaternion
%
% pos = position vector
% quat = quaternion mapping to 3x3 rotation matrix
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [dual_quat] = pq2DualQuat(pos, quat)

%
dual_quat.rot = quat;

%
dual_quat.disp = 0.5 * multiplyQuat([0; pos], quat);

end