% function [T] = twist2Transform(twist) twist vector to transformation
% matrix
% 
% this function outputs a 4x4 transformation matrix given a twist vector
%
% T = 4x4 transformation matrix
%
% twist = twist vector
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [T] = twist2Transform(twist)

%
v = twist(1:3);
Omega = twist(4:6);

if(Omega == zeros(3, 1))

    %
    T = [eye(3), v;
        0 0 0 1];

else

    %
    theta = norm(Omega);
    k = Omega / theta;
    
    %
    R = cos(theta) * eye(3) + (1 - cos(theta)) * (k * k') + sin(theta) * cpMap(k);
    d = ((eye(3) - R) * cpMap(k) + (Omega * k')) * v;
    
    %
    T = [R, d;
        0 0 0 1];

end

end