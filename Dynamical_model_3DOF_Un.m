% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

%% Define the perturbed plant
clc
clear all
close all

load('Linear_State_Space_3DOF_Sym.mat')

run Define_Parameters_3DOF_Un.m
[A_u,B_u,B_d_u,C_u,D_u,D_d_u] = Uncertain_Plant(param,A_s,B_s,B_d_s,C_s,D_s,D_d_s);

states = {'z', 'phi', 'theta', 'z_dot', 'phi_dot', 'theta_dot'};
inputs = {'theta_sf';'theta_sap';'theta_sas'};
outputs = {'z'; 'phi'; 'theta'};
disturbances = {'u_w_f';'w_w_f';'u_w_ap';'w_w_ap';'u_w_as';'w_w_as'};

%%
% Nominal plant
G = ss(A_u.NominalValue,B_u.NominalValue,C_u,D_u,...
       'statename',states,'inputname',inputs,'outputname',outputs);
% Disturbances transfer matrix
Gd = ss(A_u.NominalValue,B_d_u.NominalValue,C_u,D_d_u,...
        'statename',states,'inputname',disturbances,'outputname',outputs);
% Perturbed plant with uncertain parameters
Gp = ss(A_u,B_u,C_u,D_u,...
        'statename',states,'inputname',inputs,'outputname',outputs);
% Perturbed disturbances transfer matrix with uncertain parameters
Gd_p = ss(A_u,B_d_u,C_u,D_d_u,...
        'statename',states,'inputname',disturbances,'outputname',outputs);

save('Linear_State_Space_3DOF_Un.mat','G','Gd','Gp','Gd_p')
