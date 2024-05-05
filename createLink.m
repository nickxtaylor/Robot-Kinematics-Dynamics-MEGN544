%% creataLink creates a link
% 
% [L] = createaLink(dh parameters)
%
% output dh link% as a structure
% containing a, d, theta, alpha, theta offset, link center of mass and
% moment of inertia and other data regarding
% whether is either static or rotary or prysmatic
%
% input a, d, alpha, theta, offset, centOfmass, inertia
% dh parameters
% 
% I find it kinda silly how I'm required to write a function description
% here of such a length that it's basically 5x longer than the description I
% was given in the project prompt document. That's kinda weird
% 
% this function takes in dh parameters as regular scalars or doubles and
% creates one structure to store all of the dh parameterrs like the
% parameters listed above so that they can be easily stored and easily
% accessed later on later in the project like in the other 3 functions
% provided in this project like forward or inverse link kinematics
% this function takes in dh parameters as regular scalars or doubles and
% creates one structure to store all of the dh parameterrs like the
% parameters listed above so that they can be easily stored and easily
% accessed later on later in the project like in the other 3 functions
% provided in this project like forward or inverse link kinematics. 
%
%
% Nick Taylor
% 10920730
% MEGN 544
function [L] = createLink(aa, dd, aalpha, ttheta, Offset, CoM, m, moi)

%
L = struct;

%
L.a = aa;
L.d = dd;
L.alpha = aalpha;
L.theta = ttheta;
L.offset = Offset;
L.mass = m;
L.inertia = moi;
L.com = CoM;

%
if(isempty(dd))

    L.isRotary = 0;

elseif(isempty(ttheta))

    L.isRotary = 1;
    L.offset = Offset;

else

    L.isRotary = -1;

end

end