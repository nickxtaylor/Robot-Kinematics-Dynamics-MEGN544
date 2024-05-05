% function [roll, pitch, yaw] = rot2RPY(R) rotation to roll pitch yaw
% values
% 
% this function takes a 3x3 rotation matrix and informs what angles were
% rolled, pitched and yawed
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

function [roll, pitch, yaw] = rot2RPY(R)

%
roll = zeros(2, 1);
pitch = roll;
yaw = roll;

% 
p1 = atan2(-R(3, 1), sqrt((R(1, 1))^2 + (R(2, 1))^2));
p2 = atan2(-R(3, 1), -sqrt((R(1, 1))^2 + (R(2, 1))^2));
pitch(1) = p1;
pitch(2) = p2;

%
check = sign(R(3, 1));

%
if(p1 < (pi/2 + 0.05) && p1 > (pi/2 - 0.05))

    %
    roll(1) = -check * atan2(R(1, 2), R(2, 2));
    roll(2) = roll(1);

    %
    yaw(1:2) = [0, 0]';

elseif(p1 < (-pi/2 + 0.05) && p1 > (-pi/2 - 0.05))

    %
    roll(1) = -check * atan2(R(1, 2), R(2, 2));
    roll(2) = roll(1);

    %
    yaw(1:2) = [0, 0]';

else

    %
    roll(1) = atan2(((R(3, 2)) / cos(p1)), ((R(3, 3)) / cos(p1)));
    roll(2) = atan2(((R(3, 2)) / cos(p2)), ((R(3, 3)) / cos(p2)));
    
    %
    y = atan2(((R(2, 1)) / (cos(p1))), ((R(1, 1)) / (cos(p1))));
    yaw(1) = atan2(((R(2, 1)) / (cos(p1))), ((R(1, 1)) / (cos(p1))));
    yaw(2) = atan2(((R(2, 1)) / (cos(p2))), ((R(1, 1)) / (cos(p2))));

end

end

%%

%%

%%

%%

%%

%%