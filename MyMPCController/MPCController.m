function [sys, x0, str, ts] = MPCController(t, x, u, flag)

switch flag
    case 0
        [sys, x0, str, ts] = mdlInitializeSizes;
    case 2
        sys = mdlUpdates(t, x, u);
    case 3
        sys = mdlOutputs(t, x, u);
    case {1, 4, 9}
        sys = [];
    otherwise
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end

function [sys, x0, str, ts] = mdlInitializeSizes

sizes = simsizes;
sizes.NumContStates = 0;
sizes.NumDiscStates = 3;
sizes.NumOutputs = 2;
sizes.NumInputs = 3;
% Matrix D is non -empty
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 = [0;0;0;];
global U;
U = [0;0];
str = [];
% sample time: [period, offset]
ts = [0.05, 0];

function sys = mdlUpdates(t, x, u)
sys = x;

function sys = mdlOutputs(t, x, u)

global a b u_tilde;
global U; % U = u - u_r
global kesi;
Nx = 3; % the number of states
Nu = 2; % the number of control variables
Np = 60; % predict horizon
Nc = 30; % control horizon
pho = 10; % slack factor
fprintf('Update start, t = %6.3f\n', t)

% reference states
% A circle with a radius of 25 meters and a speed of 5 meters per second
% Center coordinates are (0, 35) and rotate counterclockwise
r(1) = 25 * sin(0.2 * t);
r(2) = 25 + 10 - 25 * cos(0.2 * t);
r(3) = 0.2 * t;
r_v = 5; % speed
r_delta = 0.104; % steering angle: arctan(L/R)

% parameters
kesi = zeros(Nx + Nu, 1);
kesi(1) = u(1) - r(1);
kesi(2) = u(2) - r(2);
kesi(3) = u(3) - r(3);
kesi(4) = U(1);
kesi(5) = U(2);
fprintf('Update start, u(1) = %4.2f\n', U(1));
fprintf('Update start, u(2) = %4.2f\n', U(2));

T = 0.05;
T_all = 40;

% vehicle parameters
L = 2.6;
u_tilde = zeros(Nx, Nu);
Q = eye(Nx * Np, Nx * Np);
R = 5 * eye(Nu * Nc); % 还不太理解
a = [1, 0, -r_v * sin(u(3)) * T;
     0, 1, r_v * cos(u(3)) * T;
     0, 0, 1;];
b = [cos(u(3)) * T, 0;
     sin(u(3)) * T, 0;
     tan(r_delta) * T / L, r_v * T / (cos(r_delta) ^ 2);];

A_cell = cell(2, 2);
B_cell = cell(2, 1);
A_cell{1, 1} = a;
A_cell{1, 2} = b;
A_cell{2, 1} = zeros(Nu, Nx);
A_cell{2, 2} = eye(Nu);
B_cell{1, 1} = b;
B_cell{2, 1} = eye(Nu);
A = cell2mat(A_cell); % size = [Nx + Nu, Nx + Nu]
B = cell2mat(B_cell); % size = [Nx + Nu, Nu]
C = cat(2, eye(Nx), zeros(Nx, Nu)); % size = [Nx, Nx + Nu]

Phi_cell = cell(Np, 1);
Theta_cell = cell(Np, Nc);
for j = 1 : Np
    Phi_cell{j, 1} = C * A ^ j;
    for k = 1 : Nc
        if k <= j
            Theta_cell{j, k} = C * A ^ (j - k) * B;
        else
            Theta_cell{j, k} = zeros(Nx, Nu);
        end
    end
end
Phi = cell2mat(Phi_cell); % size(Phi) = [Nx * Np, Nx + Nu]
Theta = cell2mat(Theta_cell); % size(Theta) = [Nx * Np, Nu * Nc]

% matrix for cost function
H_cell = cell(2, 2);
H_cell{1, 1} = Theta' * Q * Theta + R;
H_cell{1, 2} = zeros(Nu * Nc, 1);
H_cell{2, 1} = zeros(1, Nu * Nc);
H_cell{2, 2} = pho;
H = cell2mat(H_cell);
error = Phi * kesi;
f_cell = cell(1, 2);
f_cell{1, 1} = 2 * error' * Q * Theta;
f_cell{1, 2} = 0;
f = cell2mat(f_cell);

% Inequality constraint
A_t = zeros(Nc, Nc);
for p = 1 : Nc
    for q = 1 : Nc
        if q <= p
            A_t(p, q) = 1;
        else
            A_t(p, q) = 0;
        end
    end
end
A_I = kron(A_t, eye(Nu)); % size = [Nc * Nu, Nc * Nu]
Ut = kron(ones(Nc, 1), U);
umin = [-0.2;-0.54;]; % constraint for delta_u
umax = [0.2;0.332;];
delta_umin = [-0.05;-0.0082;];
delta_umax = [0.05;0.0082;];
Umin = kron(ones(Nc, 1), umin);
Umax = kron(ones(Nc, 1), umax);
A_cons_cell = {A_I, zeros(Nu * Nc, 1);
               -A_I, zeros(Nu * Nc, 1)};
b_cons_cell = {Umax - Ut;-Umin + Ut};
A_cons = cell2mat(A_cons_cell);
b_cons = cell2mat(b_cons_cell);
M = 10;
delta_Umin = kron(ones(Nc, 1), delta_umin);
delta_Umax = kron(ones(Nc, 1), delta_umax);
lb = [delta_Umin;0]; % lower bound: [delta_U, slack_factor]
ub = [delta_Umax;M]; % upper bound

% Solve the next state
% X = [delta_U', epsilon]
options = optimset;
[X, fval, exitflag] = quadprog(H, f, A_cons, b_cons, [], [], lb, ub, [], options);
u_tilde(1) = X(1);
u_tilde(2) = X(2);
U(1) = kesi(4) + u_tilde(1);
U(2) = kesi(5) + u_tilde(2);
u_real(1) = U(1) + r_v;
u_real(2) = U(2) + r_delta;
sys = u_real;
