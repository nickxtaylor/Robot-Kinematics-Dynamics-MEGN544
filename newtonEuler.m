% newtonEuler runs inverse kinematics to get the motor torques and forces 
%
% [motor_torq] = newtonEuler(linkList, paramList, paramListDot, paramListDDot, boundary_cond)
% inverse kinematics calculates the newtonian dynamics from FBD force and
% moment summations to get the necessary forces / torques for the desired
% trajectory / dynamics. Torques are for rotary joints, forces are for
% prysmatic joints
%
% input1 = linkList = list of links
% input2 = paramList = list of dynamic parameters at a given instant
% input3 = paramListDot = list of dynamic parameters for all joints at a
% given instant (theta_dot or d_dot)
% input4 = paramListDDot = list of double dot dynamic paramters for all
% joints
% boundary_conditions = newtonian characteristics of robot arm to start
%
% output = motor torques and forces 
%
% Nick Taylor
% 10920730
% MEGN544
% 11/19/2023
%
function [motor_torq] = newtonEuler(linkList, paramList, paramListDot, paramListDDot, boundary_cond)
%% start
N = length(linkList);

% get dh parameters for each joint
dyn_list = linkList.isRotary(:);

% get link parameters in its frame, m, r_com, I
m_list = linkList.mass(:);
r0i_list(:, :, :) = linkList.com(:);
I_list(:, :, :) = linkList.inertia(:, :);

% proximal frame velocities and accelerations
w0 = boundary_cond.base_angular_velocity; 
alpha0 = boundary_cond.base_angular_acceleration;
a0 = boundary_cond.base_linear_acceleration;

% Distal frame wrench in distal frame
FN = boundary_cond.distal_force;
TN = boundary_cond.distal_torque;

%% get desired rates for dynamic links: omega, alpha, d_dot, d_ddot

% link 1
z0 = [0, 0, 1]';
T01 = dhFwdKine(linkList(1), paramList(1));
d01 = T01(1:3, 4);
if(dyn_list(1) == 1) % rotary

    w01 = w0 + paramListDot(1) * z0;
    alpha01 = alpha0 + paramListDDot(1) * z0;
    d_ddot_01 = cross(alpha01, d01) + cross(w01, cross(w01, d01)) + a0;
    r01 = r0i_list(:, :, 1);
    r_ddot_01 = cross(alpha01, r01) + cross(w01, cross(w01, r01)) + a0;

elseif(dyn_list(1) == 0) % prysmatic

    w01 = w0;
    alpha01 = alpha0;
    d_ddot_01 = a0 + paramListDDot(1) * z0 + cross(alpha01, d01) + cross(w01, cross(w01, d01)) + 2 * cross(w01, paramListDot(1)*z0);
    r01 = T01(1:3, 1:3) * r0i_list(:, :, 1);
    r_ddot_01 = d_ddot_01;

end

% BUILD LIST STORAGE
% build list storage size [3, N]
zim1i = zeros(3, N); zim1i(:, 1) = z0; % Z LAST LISTS
dim1i = zeros(3, N); dim1i(:, 1) = d01; % d_i-1,i LIST 
rim1i = zeros(3, N); rim1i(:, 1) = r01;

% build list storage size [3, N+1]
w0i_list = zeros(3, N); w0i_list(:, 1) = w01; % --> ANGULAR VELOCITY LIST
alpha0i_list = zeros(3, N); alpha0i_list(:, 1) = alpha01; % ANGULAR ACCELERATION LIST
d_ddot_0i_list = zeros(3, N); d_ddot_0i_list(:, 1) = d_ddot_01; % LINEAR ACCELERATION LIST
r_ddot_0i_list = zeros(3, N); r_ddot_0i_list(:, 1) = r_ddot_01; % CoM ACCELERATION LIST

% link 2:N
if(N >= 2)

    % fill in lists
    for i = 2:N

        % build transformations matrices and d0i vectors and z_last
        Tim1i = dhFwdKine(linkList(i), paramList(i));
        zim1i(:, i) = Tim1i(1:3, 3);
        dim1i(:, i) = Tim1i(1:3, 4);
        T0i = dhFwdKine(linkList(1:i), paramList(1:i));
        rim1i(:, i) = dim1i(:, i) + T0i(1:3, 1:3) * r0i_list(i);
        
        % get desired rates for dynamic links: omega, alpha, d_ddot
        if(dyn_list(i) == 1) % ROTARY

            w0i_list(:, i) = w0i_list(:, i-1) + paramListDot(i) * zim1i(:, i);
            alpha0i_list(:, i) = alpha0i_list(:, i-1) + paramListDDot(i) * zim1i(:, i) + paramListDot(i) * cross(w0i_list(:, i-1), zim1i(:, 1));
            d_ddot_0i_list(:, i) = d_ddot_0i_list(:, i-1) + cross(alpha0i_list(:, i), dim1i(:, i)) ...
                + cross(w0i_list(:, i), cross(w0i_list(:, i), dim1i(:, i)));
            r_ddot_0i_list(:, i) = d_ddot_0i_list(:, i-1) + cross(w0i_list(:, i-1), rim1i(:, i)) ...
                + cross(w0i_list(:, i), cross(w0i_list(:, i), rim1i(:, i)));

        else % PRYSMATIC

            w0i_list(:, i) = w0i_list(:, i-1);
            alpha0i_list(:, i) = alpha0i_list(:, i-1);
            d_ddot_0i_list(:, i) = d_ddot_0i_list(:, i-1) + cross(alpha0i_list(:, i), dim1i(:, i)) ...
                + cross(w0i_list(:, i), cross(w0i_list(:, i), dim1i(:, i)))...
                + 2 * paramListDot(i) * cross(w0i_list(:, i-1), zim1i(:, i)) ...
                + paramListDDot(i) * zim1i(:, i);
            r_ddot_0i_list(:, i) = d_ddot_0i_list(:, i-1) + cross(w0i_list(:, i-1), rim1i(:, i)) ...
                + cross(w0i_list(:, i), cross(w0i_list(:, i), rim1i(:, i))) ...
                + 2 * paramListDot(i) * cross(w0i_list(:, i-1), zim1i(:, i)) ...
                + paramListDDot(i) * zim1i(:, i);

        end

    end

end

%%

% create fi, ni lists

%
fn = m_list(N) * r_ddot_0i_list(:, N) + FN;
nn = TN + I_list(:, :, N) * alpha0i_list(:, N) + cross(w0i_list(:, N), I_list(:, :, N) * w0i_list(:, N));

%
fnm1n = fn;
nnm1n = nn + cross(rim1i, fnm1n);

%
tau_n = dot(zim1i(:, N), nnm1n);
fn_motor = dot(zim1i(:, N), fnm1n);

%
motor_torq = zeros(N, 1);
if(dyn_list(N) == 1)

    motor_torq(N) = tau_n;

else

    motor_torq(N) = fn_motor;

end

end

%%

%%

%%

%%

%%

%%

