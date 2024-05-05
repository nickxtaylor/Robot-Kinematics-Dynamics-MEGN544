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
% yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda
% yadda yadda yadda yadda yadda yadda yadda yadda yaddayadda yadda yadda
% yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023
function [error] = rotationError(Rot_desired, Rot_current)

% 
R = Rot_desired * transpose(Rot_current);

%
error = rot2AngleAxis(R);

end