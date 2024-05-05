% function [X] = cpMap(w) cross product mapping
%
% function [X] = cpMap(w) maps a 3x1 vector to a 3x3 cross product matrix
% of itself
%
% X = crossproduct mapping of given vector w in 3x3 matrix form
%
% w = given vector of length 3
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [X] = cpMap(w)

%
X = [0 -w(3) w(2);
    w(3) 0 -w(1);
    -w(2) w(1) 0];

end