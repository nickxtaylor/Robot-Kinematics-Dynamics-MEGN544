% function [R] = angleAxis2Rot(Omega) --> change angle axis to rotation
% matrix
%
% Returns the rotation matrix encoded by an angle-
% axis (Omega = theta*k) rotation of theta
% radians about the unit vector k axis
% 
%
% R = rotation matrix translating to Omega
%
% Omega = vector containing magnitude and direction of a given rotation
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [R] = angleAxis2Rot(Omega)

%
theta = norm(Omega);

%
if(theta ~= 0)

    %
    k = Omega / theta;
    R = axang2rotm([k; theta]');

else

    %
    R = eye(3);

end

% if(Omega == zeros(3, 1))
% 
%     R = eye(3);
% 
% else
% 
%     %
%     theta = norm(Omega);
%     k = Omega / theta;
% 
%     %
%     R = cos(theta) * eye(3) + (1 - cos(theta)) * k * k' + sin(theta) * cpMap(k);
% 
% end