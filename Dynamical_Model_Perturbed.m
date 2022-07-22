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

[A_u,B_u,Bd_u,C_u,D_u,Dd_u,Gsm,Gsm_p] = Parametric_Perturbed_Plant(param,A_s,B_s,Bd_s,C_s,D_s,Dd_s);

states = {'z', 'phi', 'theta', 'z_dot', 'phi_dot', 'theta_dot'};
act_inputs = {'delta_sf';'delta_sap';'delta_sas'}; % Actual angles of servo motors
com_inputs = {'delta_sf_c';'delta_sap_c';'delta_sas_c'};
outputs = {'z'; 'phi'; 'theta'};
disturbances = {'u_w_f';'w_w_f';'u_w_ap';'w_w_ap';'u_w_as';'w_w_as'};

%% Define the tranfer matrices for nominal and perturbed plant
% Nominal plant
G = ss(A_u.NominalValue,B_u.NominalValue,C_u,D_u,...
       'statename',states,'inputname',act_inputs,'outputname',outputs);
% Disturbances transfer matrix
Gd = ss(A_u.NominalValue,Bd_u.NominalValue,C_u,Dd_u,...
        'statename',states,'inputname',disturbances,'outputname',outputs);
% Perturbed plant with uncertain parameters
Gp = ss(A_u,B_u,C_u,D_u,...
        'statename',states,'inputname',act_inputs,'outputname',outputs);
Gp = simplify(Gp,'full');
% Perturbed disturbances transfer matrix with uncertain parameters
Gd_p = ss(A_u,Bd_u,C_u,Dd_u,...
        'statename',states,'inputname',disturbances,'outputname',outputs);
Gd_p = simplify(Gd_p,'full');
% Nominal actuators model
Gsm = ss(Gsm.A,Gsm.B,Gsm.C,Gsm.D,'statename',act_inputs,'inputname',...
         com_inputs,'outputname',act_inputs);
% Perturbed actuators model
Gsm_p = ss(Gsm_p.A,Gsm_p.B,Gsm_p.C,Gsm_p.D,'statename',act_inputs,...
           'inputname',com_inputs,'outputname',act_inputs);

save('Data Files/LTI_Perturbed_Plant.mat')