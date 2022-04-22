function dxdt = kinematicbicycleCT0(x, u)
%% Continuous-time nonlinear dynamic model of a pendulum on a cart
%
% 4 states (x): 
%   cart position (z)
%   cart velocity (z_dot): when positive, cart moves to right
%   angle (theta): when 0, pendulum is at upright position
%   angular velocity (theta_dot): when positive, pendulum moves anti-clockwisely
% 
% 1 inputs: (u)
%   force (F): when positive, force pushes cart to right 
%
% Copyright 2018 The MathWorks, Inc.

%#codegen

%% parameters
% mCart = 1;  % cart mass
% mPend = 1;  % pendulum mass
% g = 9.81;   % gravity of earth
% L = 0.5;    % pendulum length
% Kd = 10;    % cart damping
l_er = 1;
%% Obtain x, u and y
% x
state=x;
x = state(1);
y = state(2);
psi = state(3);
v=state(4);
% u
f = [v*cos(psi); v*sin(psi);0;0;];
g = [0 -v*sin(psi); 0 v*cos(psi); 0 v/l_er; 1 0;];
dxdt = f+g*u;
