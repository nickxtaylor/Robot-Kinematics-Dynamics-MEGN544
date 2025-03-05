% function [twist] = transform2Twist(T) transformation matrix to screw
% theory
% 
% this function outputs twist given a 4x4 transformation matrix
%
% twist = twist vector
%
% T = 4x4 transformation matrix
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [twist] = transform2Twist(T)

%
d = T(1:3, 4);
R = T(1:3, 1:3);

%
r = rotm2axang(R);
theta = r(4);
k = r(1:3)';
Omega = theta .* k;

%
if(theta == 0)

    v = d;

else

    v = inv((eye(3) - R) * cpMap(k) + (Omega * k')) * d;

end

% 
twist = [v; Omega];

end