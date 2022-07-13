% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

%% Define the nonlinear state space model
clc
clear all
close all

load('NL_State_Space_3DOF_Sym.mat')
load('Parameters_3DOF.mat','param')
load('LQR_controller.mat')

xeq = [param.z_n0; 0; 0; 0; 0; 0]; % operating point around stable equilibrium
ref = [param.z_n0;0;0];

% Simulation time 
dt = 0.01; % sampling time
tend = 1; % duration of simulation in seconds
tspan = 0:dt:tend;

x0 = [-0.05; 0; 0; 0; 0; 0]; % current state

% x = zeros(6,length(t)); % Allocate state variable
% x(:,1) = x0;             % Initial conditions

dxdt_s = arrayfun(@char, dxdt_sym, 'UniformOutput', 0);
dxdt_s = string(dxdt_s);

%%

for k=1:length(t)-1
    k
    x0 = x(:,k)
%     u = Lc*ref - K*(x0 - xeq);
    u = - K_dt*x0;
    x(:,k+1) = x(:,k) + dt*NLdynamics3DOF(t(k), x0, u, param, wave_param, dxdt_s);
end

%%
%  - xeq) + [param.theta_s0_f; param.theta_s0_ap; param.theta_s0_as]
%Simulation without controller
f = @(t,x)NLdynamics3DOF(t, x, param, wave_param, dxdt_s, K, Lc); % ODE definition
[t,x] = ode45(f,tspan,x0);       % Solving ODE
