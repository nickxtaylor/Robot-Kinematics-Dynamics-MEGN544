% function [final] = orient(points, scale, offsetx, offsetz) reorient 3D
% points to be plotted
%
% function [final] = orient(points, scale, offsetx, offsetz) this function
% scales and linearly translates 3D points so that they match the start and
% end points desired as per the project description. There is a scaling
% factor as well as a linear shift in x and z directions.
%
% final = final set of points after being reoriented
%
% points = initial points given
% scale = scaling factor
% offsetx = offset in the x direction
% offsetz = offset in the z direction
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [final] = orient(points, scale, offsetx, offsetz)

%
[sz, ~] = size(points);

%
intermediate = scale .* points;

%
T = [eye(3), [offsetx, 0, offsetz]';
    0 0 0 1];

%
new = zeros(sz, 4);

%
for i = 1:sz

    %
    new(i, :) = T * [intermediate(i, 1), 0, intermediate(i, 2), 1]';

end

%
final = [new(:, 1), new(:, 3)];

end