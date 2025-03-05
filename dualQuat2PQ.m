% function [pos, quat] = dualQuat2PQ(dual_quat)
%
% inverse dual quaternion solution mapping a dq to 3d space
%
% pos = position vector corresponding to dual_quat
% quat = rotation quaternion that maps to rotation matrix in 3d space
%
% dual_quat = dual quaternion as a structure with rot and disp members
%
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [pos, quat] = dualQuat2PQ(dual_quat)

%
Q = dual_quat.rot;
D = dual_quat.disp; 
Q_conj = [Q(1); -Q(2); -Q(3); -Q(4)];
d = 2 * multiplyQuat(D, Q_conj);

%
pos = d(2:4);
quat = Q;

end