% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

%% Define the actual perturbed plant with parametric uncertainties
clc
clear all
close all

addpath('Data Files')

load('LTI_Symbolic_Matrices.mat')
run Define_Uncertain_Parameters.m

[A_u,B_u,Bd_u,C_u,D_u,Dd_u,Gsm_p,g_sm_p_i] = Parametric_Perturbed_Plant(param,A_s,B_s,Bd_s,C_s,D_s,Dd_s);

% Definitons of the state space
states = {'z_n', 'phi', 'theta', 'z_dot', 'phi_dot', 'theta_dot'};
act_inputs = {'delta_s_f';'delta_s_ap';'delta_s_as'}; % Actual angles of servo motors
com_inputs = {'delta_s_f_c';'delta_s_ap_c';'delta_s_as_c'}; % Commanded angles of servo motors
outputs = {'z_n'; 'phi'; 'theta'};
disturbances = {'u_w_f';'w_w_f';'u_w_ap';'w_w_ap';'u_w_as';'w_w_as'};

%% Define the tranfer function matrices with uncertainties

% Perturbed plant with uncertain parameters
Gp = ss(A_u,B_u,C_u,D_u,...
        'statename',states,'inputname',act_inputs,'outputname',outputs);
Gp = simplify(Gp,'full');
% Perturbed disturbances transfer matrix with uncertain parameters
Gd_p = ss(A_u,Bd_u,C_u,Dd_u,...
        'statename',states,'inputname',disturbances,'outputname',outputs);
Gd_p = simplify(Gd_p,'full');
% Perturbed actuators model
Gsm_p = ss(Gsm_p.A,Gsm_p.B,Gsm_p.C,Gsm_p.D,'statename',act_inputs,...
           'inputname',com_inputs,'outputname',act_inputs);

save('Data Files/LTI_Perturbed_Plant.mat')