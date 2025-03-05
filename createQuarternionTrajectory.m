function [trajectory] = createQuarternionTrajectory(simTime)

if ~exist("simTime")
    simTime = 15;
end

% get trajectory
run Geometry.m
load('trajectory_transforms.mat');
nL = size(pose_list, 3);
n = size(pose_list, 3) + 6;
trajectory = zeros(n, 8);
c_time = 2;
t_linear = linspace(c_time + 1, simTime-2, n-4);
trajectory(3:n-2, 1) = t_linear';

% two at zero_for_arm;
paramList0 = [0, pi/2, 0, 0, 0, 0];
[Td0] = dhFwdKine(linkList, paramList0);
p0 = Td0(1:3, 4);
Q0 = rot2Quat(Td0(1:3, 1:3));
Q0 = Q0 / norm(Q0);
trajectory(1, :) = [0, p0', Q0'];
trajectory(2, :) = [0.5, p0', Q0'];
% check = Q0(1)^2 + Q0(2:4)'*Q0(2:4);

% top of C 
Td1 = pose_list(:, :, 1);
p1 = Td1(1:3, 4);
R1 = Td1(1:3, 1:3);
Q1 = rot2Quat(R1);
Q1 = Q1 / norm(Q1);
trajectory(3, :) = [c_time, p1', Q1'];
% check = Q1(1)^2 + Q1(2:4)'*Q1(2:4);

% every pose  in list
for i = 1:nL

    %
    Td = pose_list(:, :, i);
    pd = Td(1:3, 4);
    Qd = rot2Quat(Td(1:3, 1:3));
    Qd = Qd / norm(Qd);
    trajectory(3+i, 2:8) = [pd', Qd'];
    % check = Qd(1)^2 + Qd(2:4)'*Qd(2:4);

end

% repeat bottom of M
TdM = pose_list(:, :, end);
pM = TdM(1:3, 4);
RM = TdM(1:3, 1:3);
QM = rot2Quat(RM);
QM = QM / norm(QM);
trajectory(n-2, 2:8) = [pM', QM'];
% check = QM(1)^2 + QM(2:4)'*QM(2:4);

% two at zero_fop_arm
paramListf = [0, 0, 0, 0, 0, 0];
[Tdf] = dhFwdKine(linkList, paramListf);
pf = Tdf(1:3, 4);
Qf = rot2Quat(Tdf(1:3, 1:3));
Qf = Qf / norm(Qf);
trajectory(n-1, :) = [simTime - 1, pf', Qf'];
trajectory(n, :) = [simTime, pf', Qf'];
% check = Qf(1)^2 + Qf(2:4)'*Qf(2:4);

% check quarternion rotation
for i = 1:n-1

    % get Q
    Q_now = trajectory(i, 5:8)';
    Q_next = trajectory(i+1, 5:8)';

    % take dot product
    dot_prod = dot(Q_now, Q_next);

    % flip sign if necessary
    if(dot_prod < 0)

        trajectory(i+1, 5:8) = -1 * trajectory(i+1, 5:8);

    end

    %
    % check = Q_now(1)^2 + Q_now(2:4)'*Q_now(2:4)
    % if(check ~= 1)
    % 
    %     disp('i = ')
    %     i
    % 
    % end

end

end

% save('trajectory_transforms.mat','pose_list');