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
function [error] = transError(Td, Tc)

%
pos_d = Td(1:3, 4);
pos_c = Tc(1:3, 4);

%
Rd = Td(1:3, 1:3);
Rc = Tc(1:3, 1:3);

%
error = zeros(6, 1);
error(1:3) = pos_d - pos_c;
error(4:6) = rotationError(Rd, Rc);

end