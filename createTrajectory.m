function [trajectory] = createTrajectory(simTime)

if ~exist("simTime")
    simTime = 20;
end

% get trajectory
load('trajectory_transforms.mat');
c_time = 2;
n = size(pose_list, 3) + 6;
t_linear = linspace(c_time+1, simTime-2, n-4);
trajectory = zeros(n, 7);
trajectory(3:n-2, 1) = t_linear';

point_time = 0;
% two at zero_for_arm
trajectory(1, :) = [0, 0, pi/2, 0, 0, 0, 0];
% point_time = point_time + .5;
trajectory(2, :) = [.5, 0, pi/2, 0, 0, 0, 0];
% point_time = 2 - .5;

% top of C 
[th1, th2, th3, th4, th5, th6, ~] = abbInvKine(pose_list(:, :, 1), trajectory(2, 2:end));
trajectory(3, :) = [c_time, th1, th2, th3, th4, th5, th6];
% point_time = point_time+.5;

% every pose  in list
% trajectory(4:end, 1) = trajectory(4:end, 1) + point_time;
th_last = [th1, th2, th3, th4, th5, th6];
for i = 1:size(pose_list,3)

    %
    Td = pose_list(:, :, i);
    [th1, th2, th3, th4, th5, th6, ~] = abbInvKine(Td, th_last);
    trajectory(3+i, 2:7) = [th1, th2, th3, th4, th5, th6];
    th_last = [th1, th2, th3, th4, th5, th6];

end

% repeat bottom of M
[th1, th2, th3, th4, th5, th6] = abbInvKine(pose_list(:, :, end), th_last);
trajectory(n-2, 2:7) = [th1, th2, th3, th4, th5, th6];

% two at 
trajectory(n-1, :) = [simTime - 1, 0, 0, 0, 0, 0, 0];
trajectory(n, :) = [simTime, 0, 0, 0, 0, 0, 0];

%%

figure(5)
title('theta space in time')
xlabel('time [seconds]')
ylabel('thetas')
plot(trajectory(:, 1), trajectory(:, 2:end))
legend('\theta_1', '\theta_2', '\theta_3', '\theta_4', '\theta_5', '\theta_6')

run Geometry.m

figure(10)
scatter3(0, 0, 0)
hold on
for i = 1:n
    T = dhFwdKine(linkList,trajectory(i,2:end));
    %
    pos = T(1:3, 4);
    scatter3(pos(1), pos(2), pos(3), 'MarkerEdgeColor','k','MarkerFaceColor',[0 0 1])

end
hold off

end

% save('trajectory_transforms.mat','pose_list');