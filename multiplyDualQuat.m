% function [dual_quat] = multiplyDualQuat(dq_left, dq_right) multiply dual
% quaternions
%
% function [dual_quat] = multiplyDualQuat(dq_left, dq_right) this function
% multiplies two dual quaternions and maintains those values as a structure
%
% dual_quat = product of two dual quaternions given as inputs
%
% dq_left = first qual quaternion
% dq_right = second dual quaternion
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [dual_quat] = multiplyDualQuat(dq_left, dq_right)

%
q1 = dq_left.rot;
d1 = dq_left.disp;

% 
q2 = dq_right.rot;
d2 = dq_right.disp;

% 
dual_quat.rot = multiplyQuat(q1, q2);
dual_quat.disp = multiplyQuat(d1, q2) + multiplyQuat(q1, d2);

end