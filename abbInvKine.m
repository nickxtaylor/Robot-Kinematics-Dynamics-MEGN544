%% abbInvKine ABB inverse kinematics
%
% [theta's] = abbInvKine
%
% output 6 theta angles and boolean reachability information
% if reachable is true, the desired configuration is possible. Otherwise,
% the desired configuration is not possible. It could be beyond the robots
% most retracted state, or beyond the robots most extended state.
%
% input desired transformation matrix, previous theta
%
% ..
%  
%
% Nick Taylor
% 10920730
% MEGN 544
function [th1, th2, th3, th4, th5, th6, reach]  = abbInvKine(T_des, th_last)

%     a     d       alpha  th_offset  --- for reference
dh = [0     0.290   -pi/2  0;
      0.27  0       0      pi/2;
      0.07  0       -pi/2  0;
      0     0.302   pi/2   0;
      0     0       -pi/2  0;
      0     0.072   0      0];

% dh params
a2 = 0.27;
a3 = 0.07;
d1 = 0.29;
d4 = 0.302;
d6 = 0.072;
% reach is the boolean variable that determines if the final location is
% possible to reach or not
reach = 1;

% save is the matrix where all possible theta solutions will be stored
theta = zeros(6, 8);

% theta 1
d06 = T_des(1:3, 4);
d05 = d06 - d6 * T_des(1:3, 3);
t1 = [atan2(d05(2), d05(1)), atan2(d05(2), d05(1)) + pi];
theta(1, 1:4) = t1(1) * ones(1, 4); theta(1, 5:8) = t1(2) * ones(1, 4);

% theta 3
R01_1 = (rotZ(t1(1)) * rotX(dh(1, 3))); % two rotation matrices for two theta 1 values
R01_2 = (rotZ(t1(2)) * rotX(dh(1, 3)));
d15 = d05 - d1 * [0; 0; 1]; % d15 in the zero frame
d15_1 = transpose(R01_1)*d15; % d15 in the 1 frrame
d15_2 = transpose(R01_2)*d15;
nu = atan2(d4, a3);
gamma = (a2^2 + d4^2 + a3^2 - (norm(d15))^2) / (2 * a2 * sqrt(d4^2 + a3^2)); % law of cosines
phi = real([2 * atan(sqrt((1 - gamma) / (1 + gamma))), -2 * atan(sqrt((1 - gamma) / (1 + gamma)))]); % tangent half angle identity
theta(3, [1,2,5,6]) = (pi - phi(2) - nu) * ones(1, 4); % theta 3 solutions
theta(3, [3,4,7,8]) = (pi - phi(1) - nu) * ones(1, 4);

% theta 2 - solution provided from hw5
alpha = [a2 + a3*cos(theta(3, 1)) + d4*cos(theta(3, 1) + pi/2), a2 + a3*cos(theta(3, 3)) + d4*cos(theta(3, 3) + pi/2)];
beta = [a3*sin(theta(3, 1)) + d4*sin(theta(3, 1) + pi/2), a3*sin(theta(3, 3)) + d4*sin(theta(3, 3) + pi/2)];
% alpha(1), beta(1), d15_1
y12 = (alpha(1)*d15_1(2) - beta(1)*d15_1(1)) / (alpha(1)^2 + beta(1)^2);
x12 = (alpha(1)*d15_1(1) + beta(1)*d15_1(2)) / ((alpha(1))^2 + (beta(1))^2);
theta(2, 1:2) = (atan2(y12, x12)) * ones(1, 2);
% alpha(1), beta(1), d15_2
y34 = (alpha(1)*d15_2(2) - beta(1)*d15_2(1)) / ((alpha(1))^2 + (beta(1))^2);
x34 = (alpha(1)*d15_2(1) + beta(1)*d15_2(2)) / ((alpha(1))^2 + (beta(1))^2);
% theta(2, 3:4) = (atan2(y34, x34)) * ones(1, 2);
% alpha(2), beta(2), d15_1
y56 = (alpha(2)*d15_1(2) - beta(2)*d15_1(1)) / ((alpha(2))^2 + (beta(2))^2);
x56 = (alpha(2)*d15_1(1) + beta(2)*d15_1(2)) / ((alpha(2))^2 + (beta(2))^2);
% theta(2, 5:6) = (atan2(y56, x56)) * ones(1, 2);
% alpha(2), beta(2), d15_2
y78 = (alpha(2)*d15_2(2) - beta(2)*d15_2(1)) / ((alpha(2))^2 + (beta(2))^2);
x78 = (alpha(2)*d15_2(1) + beta(2)*d15_2(2)) / ((alpha(2))^2 + (beta(2))^2);
theta(2, 7:8) = (atan2(y78, x78)) * ones(1, 2);
%
theta(2, 3:4) = (atan2(y56, x56)) * ones(1, 2);
theta(2, 5:6) = (atan2(y34, x34)) * ones(1, 2);

% get theta 4, 5, 6
for i = [1 3 5 7]

    % get rotation matrix 3R6
    R03 = rotZ(theta(1, i)) * rotX(-pi/2) * rotZ(theta(2, i)) * rotZ(theta(3, i)) * rotX(-pi/2);
    R36 = (transpose(R03)) * T_des(1:3, 1:3);

    % theta5
    t5 = [atan2( (sqrt(R36(3, 1)^2 + R36(3, 2)^2)), R36(3, 3)), atan2((-sqrt(R36(3, 1)^2 + R36(3, 2)^2)), R36(3, 3))];
    theta(5, i:i+1) = t5;

    if(abs(sin(t5(1))) < 10^-10 || abs(sin(t5(2))) < 10^-10)

        % constraint optimization
        sumdiff = atan2(-R36(1, 2), R36(2, 2));
        pm = sign(R36(3, 3));
        sumdiff_last = th_last(4) + pm * th_last(6);
        sumdiff = sumdiff - round((sumdiff - sumdiff_last) / (2*pi)) * 2*pi;
        t4 = 0.5 * (sumdiff + th_last(4) - pm*th_last(6));
        theta(4, i:i+1) = [t4, t4];
        t6 = 0.5 * (th_last(6) + pm*sumdiff - pm*th_last(4));
        theta(6, i:i+1) = [t6, t6];

    else
   
        % theta6
        t6 = [atan2( (-R36(3, 2)/sin(t5(1))), (R36(3, 1)/sin(t5(1)))), atan2((-R36(3, 2)/sin(t5(2))), (R36(3, 1)/sin(t5(2))))];
        theta(6, i:i+1) = t6;
       
        % theta4
        t4 = [atan2((-R36(2, 3) / sin(t5(1))), (-R36(1, 3)/sin(t5(1)))), atan2((-R36(2, 3) / sin(t5(2))), (-R36(1, 3)/sin(t5(2))))];
        theta(4, i:i+1) = t4;

    end

end

%% here is where I would put my code to compare theta solutions to th_last

%
theta(2,:) = theta(2,:) + pi/2; % fix offset

th_last = reshape(th_last,6,1);

error = theta - repmat(th_last,[1,8]);
error = wrapToPi(error);
cost = sum(error.^2,1);
[~,bestInd] = min(cost);

final = theta(:,bestInd);

final = final - round( (final - th_last)/(2*pi))*2*pi;

% return final answer
th1 = final(1);
th2 = final(2);
th3 = final(3);
th4 = final(4);
th5 = final(5);
th6 = final(6);

%% just for checks

% T = eye(4);
% for j = 1:6
% 
%     % get dhtransform for all 26 points
%     a = dh(j, 1);
%     d = dh(j, 2);
%     alpha = dh(j, 3);
%     % theta_check = theta(j, i);
%     Ti = dhTransform(a, d, alpha, final(j));
%     T = T * Ti;
% 
% end
% 
% %
% pos = T(1:3, 4)

%% check reachability -- 

% check reachability - should always be true
if(1 - gamma < 0 || 1 + gamma < 0)
    reach = 0;
end
end