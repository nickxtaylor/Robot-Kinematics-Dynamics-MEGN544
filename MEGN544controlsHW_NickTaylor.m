%% MRGN 544 CONTROLS HW

% 
clear all; clc; close all;

%% a

% constants
m = 1; c = 0.5; k = 3;

%
wn = sqrt(k);
zeta = 1 / (4 * wn);
wd = wn * sqrt(1 - zeta^2);
sigma = - wn * zeta;
kp = m * wn^2 - k;
kd = 2 * zeta * wn - c;

%% b

%
OS = 5/100;
tr = 0.1;
ts = 0.5;
zeta_p = sqrt((log(OS))^2 / (pi^2 + log(OS)^2));
wn_p1 = (-log(0.01)) / (zeta_p * ts);
wn_p2 = (1.53 + 2.31 * zeta_p^2) / tr;
sigma_p1 = -wn_p1 * zeta_p;
sigma_p2 = -wn_p2 * zeta_p;
wd_p1 = wn_p1 * sqrt(1 - zeta_p^2);
wd_p2 = wn_p2 * sqrt(1 - zeta_p^2);
m1 = wd_p1 / sigma_p1;
m2 = wd_p2 / sigma_p2;
x = linspace(-20, 0, 10);
figure(1) 
hold on;
grid on;
xlim([-20 0])
scatter(sigma_p1, wd_p1, 'red x', 'LineWidth', 2);
scatter(sigma_p2, wd_p2, 'blue x', 'LineWidth', 2)
scatter(sigma_p1, -wd_p1, 'red x', 'LineWidth', 2);
scatter(sigma_p2, -wd_p2, 'blue x', 'LineWidth', 2)
plot(x, m1*x, 'black --')
plot(x, -m1*x, 'black --')
title('Root Locus')
xlabel('Im')
ylabel('Re')
hold off

%% c

%
A = [0, 1; -0.5, -3];
B = [0;1];
C = [1,0];
D = 0;
sys = ss(A,B,C,D);
zeta = sqrt((log(0.01))^2 / (pi^2 + (log(0.01))^2));
wn = (1.53 + 2.31 * zeta^2) / (0.5);
p1 = -wn * (zeta + 1i * sqrt(1 - zeta^2)); 
p2 = -wn * (zeta - 1i * sqrt(1 - zeta^2));
p = [p1, p2];
K = place(A,B,p);
Acl = A-B*K;
syscl = ss(Acl,B,C,D);
Pcl = pole(syscl);
step(syscl)

%% d 

simOut = sim("MEGN544controlsHW_NickTaylor_MODEL");

%% e

% 
Cc = [B, A*B];
V = eig(A);
prod = (V(1) * eye(2) - A) * (V(2) * eye(2) - A);
Kt = [0, 1] * inv(Cc) * prod;

%% yeah, I know. 