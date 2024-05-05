% myFunctionName Short description of what this function does.
%
% [output1, output2] = myFunctionName(input1, input2) Now a more description
% multiline description of the function would be appropriate.
%
% output1 = description of what the first output is/means include units if appropriate
% output2 = description of what the second output is/means include units if appropriate
%
% input1 = description of what the first input is/means include units if appropriate
% input2 = description of what the second input is/means include units if appropriate
%
% yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda 
% yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda 
% yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda
% yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda yadda 
%
% Your Name
% Your Student Number
% Course Number
% Date
function [motor_torq] = newtonEuler(linkList, paramList, paramListDot, paramlistDDot, boundary_cond)

%
q = paramList;
qdot = paramListDot;
qddot = paramlistDDot;

% start
w0 = boundary_cond.base_angular_velocity; 
alpha0 = boundary_cond.base_angular_acceleration;
a0 = boundary_cond.base_linear_acceleration;
fn = boundary_cond.distal_force;
tn = boundary_cond.distal_torque;
n = length(linkList);

% save --> n+1 or n rows
zi = zeros(n+1, 3); zi(1, :) = [0 0 1];
ri = zeros(n, 3);
ai = zeros(n+1, 3); ai(1, :) = a0';
acom = zeros(n+1, 3); acom(1, :) = a0';
wi = zeros(n+1, 3); wi(1, :) = w0';
alpha_i = zeros(n+1, 3); alpha_i(1, :) = alpha0';
Ti = zeros(4*(n+1), 4); Ti(1:4, :) = eye(4); T = eye(4);
fi = zeros(n+1, 3); fi(end, :) = fn;
ti = zeros(n+1, 3); ti(end, :) = tn;
motor_torq = zeros(n, 1);

% for each link --> proximal to distal
for i = 2:n+1

    %
    T0i = dhFwdKine(linkList(1:i-1), paramList(1:i-1));
    dn = T0i(1:3, 4);

    %
    Ti(4*(i-1):4*(i-1)+3, :) = dhFwdKine(linkList(i-1), paramList(i-1)); % T_(i-1, i)
    di = (T0i(1:3, 1:3) * Ti(1:3, 4))'; % rotated to zero frame!!!

    %
    R0i = T0i(1:3, 1:3);
    zi(i, :) = transpose(R0i * Ti(1:3, 3)); % rotated to be in zero frame

    %
    if(linkList(i-1).isRotary == 1) % rotary--> paramlistdot(i) = theta_dot_i

        %
        wi(i,:) = wi(i-1,:) + qdot(i) * zi(i-1, :);
        alpha_i(i, :) = alpha_i(i-1, :) + qddot * zi(i-1, :);
        ai(i, :) = ai(i-1, :) + cross(alpha_i(i, :), di) + cross(wi(i, :), cross(wi(i, :), di));

    elseif(linkList(i-1).isRotary == 0) % prysmatic --> paramlistdot(i) = d_dot_i

        %
        wi(i,:) = wi(i-1, :);
        alpha_i(i, :) = alpha_i(i-1, :);
        ai(i, :) = ai(i-1, :) + cross(alpha_i(i, :), di) + cross(wi(i, :), cross(wi(i, :), di)) ...
            + 2 * (cross(wi(i, :), qdot * zi(i-1, :))) + qddot * zi(i-1, :);

    else % static

        disp('static')

    end

    % get rii
    if(i == 2)

        ri(i-1, :) = (linkList(i-1).com)';

    else

        ri(i-1, :) = dn + (linkList(i-1).com)';

    end

    % get center of mass acceleration
    acom(i-1, :) = ai(i, :) + cross(alpha_i(i, :), ri(i-1, :)) + cross(wi(i, :), cross(wi(i, :), ri(i-1, :)));

    %
    fi(i-1, :) = linkList(i-1).mass * acom(i-1, :);
    ti(i-1, :) = transpose(linkList(i-1).inertia * alpha_i(i, :)' + cross(wi(i, :)', (linkList(i-1).inertia * wi(i, :)')));

    %
    motor_torq(i-1) = dot(zi(i-1,:), ti(i-1, :));

    %
    T = T * Ti(4*(i-1):4*(i-1)+3, :); % --> should be equal to T0i

end


end