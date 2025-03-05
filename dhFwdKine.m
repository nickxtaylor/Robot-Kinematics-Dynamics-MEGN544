%% dhFwdKine computes forward kinematics
% 
% [H] = dhFwdKine(linkList, paramList)
%
% output
% - forward kinematics of manipulator
%
% input
% - array of dynamic/static links
% - array containing current state of joint variables, which determines if
% links are dynamic or static (prsymatic, rotary or static)
%
% [...]
%  
%
% Nick Taylor
% 10920730
% MEGN 544
function [T] = dhFwdKine(linkList, paramList)

%
n = length(linkList); % number of links
T = eye(4); 
count = 1; % variable to be updated ONLY WHEN LINKS ARE DYNAMIC

%
for i = 1:n

    %
    a = linkList(i).a;
    d = linkList(i).d;
    alpha = linkList(i).alpha;
    theta = linkList(i).theta;
    offset = linkList(i).offset;

    % check static / dynamic
    if(linkList(i).isRotary == 0)

        d = paramList(count) - offset;
        count = count + 1;

    elseif(linkList(i).isRotary == 1)

        theta = paramList(count) - offset;
        count = count + 1;
        
    end

    %
    Ti = dhTransform(a, d, alpha, theta);
    T = T * Ti;

end

end

%%

%%

%%

%%

%%

%%

%%