% function [Omega] = rot2AngleAxis(R) maps R to angle axis
% 
% this function takes a 3x3 rotation matrix and maps it to angle axis
% vector containing magnitude and direction of rotation
%
% R = 3x3 rotation matrix
%
% Omega = 3x1 vecto containing magnitude and direction of rotation
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [Omega] = rot2AngleAxis(E)

%
% r = rotm2axang(R)';
% Omega = r(4) .* r(1:3);

%
p = [E(3, 2) - E(2, 3);
    E(1, 3) - E(3, 1);
    E(2, 1) - E(1, 2)];

%
theta = atan2(0.5*norm(p), 0.5*(trace(E) - 1));

%
if(theta >= pi-0.001 && theta <= pi+0.001)

    kx = sqrt(0.5 * (E(1, 1) + 1));
    ky = sqrt(0.5 * (E(2, 2) + 1));
    kz = sqrt(0.5 * (E(3, 3) + 1));

    %
    k = [kx ky kz]';
    Omega = theta .* k;

elseif(theta < 10^(-12) && theta > -10^(-12))

    Omega = [0 0 0]';

else

    k = (1 / (2*sin(theta))) .* p;
    Omega = theta .* k;

end

end