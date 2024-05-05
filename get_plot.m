% function [] = get_plot(points)
%
% this function plots the desired 3d image of the CSM path on a plane along the y
% axis
%
% points = set of points to be plotted in 3d spelling out CSM
%
% no outputs beside 3d plot of points
%
% Nick Taylor
% 10920730
% MEGN 544
% SEPTEMBER 2023

function [] = get_plot(points)

%
x = points(:, 1);
y = points(:, 2);
z = points(:, 3);

%
figure(1)
plot3(x, y, z, 'black --', 'LineWidth', 1);
hold on

grid on
title('Lab Component')
xlabel('world x [m]')
ylabel('world y [m]')
zlabel('world z [m]')
axis equal
xticks([-0.05 -0.01 0.05 0.1 0.14])
zticks([0.4 0.44 0.48])
for i = 1:length(points)-1

    %
    scatter3(x(i), y(i), z(i), 'yellow', 'filled')
    delx = (x(i+1) - x(i));
    dely = (z(i+1) - z(i));
    theta = atan2(dely, delx);

    %
    mag = 0.0001;
    quiver3(x(i), y(i), z(i), sqrt(mag)*cos(theta), 0, sqrt(mag)*sin(theta), 'red', 'LineWidth', 2);
    quiver3(x(i), y(i), z(i), sqrt(mag)*sin(theta), 0, -sqrt(mag)*cos(theta), 'green', 'LineWidth', 2);
    quiver3(x(i), y(i), z(i), 0, 0.009, 0, 'blue', 'LineWidth', 2);

    %


end

scatter3(x(end), y(end), z(end), 'yellow', 'filled');
quiver3(x(i+1), y(i+1), z(i+1), sqrt(mag)*cos(theta), 0, sqrt(mag)*sin(theta), 'red', 'LineWidth', 2);
quiver3(x(i+1), y(i+1), z(i+1), sqrt(mag)*sin(theta), 0, -sqrt(mag)*cos(theta), 'green', 'LineWidth', 2);
quiver3(x(i+1), y(i+1), z(i+1), 0, 0.009, 0, 'blue', 'LineWidth', 2)

legend('path', 'points', 'local x' ,'local y' ,'local z')

hold off



end