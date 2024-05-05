% function [Q] = rot2Quat(R)
% 
% this function takes a 3x3 rotation matrix and maps it to a quaternion
%
% R = 3x3 rotation matrix
%
% Q = quaternion
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [Q] = rot2Quat(R)

% check
dir = [(R(3,2) - R(2,3)), (R(1,3) - R(3,1)), (R(2,1) - R(1,2))]';
theta = atan2((0.5 * norm(dir)), ((trace(R) - 1) / (2)));

% 
q0 = sqrt((1 + trace(R)) / (4));

%
q0_check = cos(theta / 2);

if(q0 == -q0_check)

    %
    q0 = -q0;

end

%
if(q0 == 0)

    %
    r = rotm2axang(R);
    q = sin(r(4) / 2) .* (r(1:3));

    % 
    Q = [0 q]';

else

    %
    q1 = (R(3,2) - R(2,3)) / (4*q0);
    q2 = (R(1,3) - R(3,1)) / (4*q0);
    q3 = (R(2,1) - R(1,2)) / (4*q0);

    %
    Q = [q0 q1 q2 q3]';

end

end