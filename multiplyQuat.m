% function [P] = multiplyQuat(Q1, Q2) multiply quaternions
%
% function [P] = multiplyQuat(Q1, Q2) this function
% multiplies two quaternions and outputs the product
%
% P = product of two quaternions given as inputs
%
% Q1 = first quaternion
% Q2 = second quaternion
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [P] = multiplyQuat(Q1, Q2) % P = Q1 * Q2 --> 1 = left; 2 = right

% 
q01 = Q1(1);
q02 = Q2(1);

%
q1 = Q1(2:4);
q2 = Q2(2:4);

% FROM LECTURE 4 SLIDE 3
p0 = q01 * q02 - (q1' * q2);
p = q02 * q1 + q01 * q2 + cpMap(q1) * q2;
P = [p0; p];

end