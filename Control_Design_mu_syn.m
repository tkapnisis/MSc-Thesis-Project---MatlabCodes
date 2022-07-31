% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

clc
clear all
close all

addpath('Plotting Functions')
addpath('Data Files')

load('LTI_Perturbed_Plant.mat','Gp','Gd_p','Gsm_p')
load('Parameters_Nominal.mat','param')
load('LTI_Nominal_Plant.mat','G','Gd','Gsm','foil_loc')
load('Uncertainty_Weighting_Functions.mat','W_I_G','W_I_Gd','W_I_Gsm')

load('Controller_hinf.mat')

load('Controller_mu_syn.mat')

% Nominal plant model G(s)
% Nominal disturbance model Gd(s)
% Nominal actuator model Gsm(s)
% Perturbed plant model with uncertain parameters Gp(s)
% Perturbed disturbance model with uncertain parameters Gd_p(s)
% Perturbed actuator model with uncertain parameters Gsm_p(s)

nmeas = 3; % number of outputs 
ncont = 3; % number of inputs

run Bode_options.m
run Sigma_options.m

% Define the Weighting Functions for the mu-synthesis controller (same as
% hinf design for the nominal plant
[Wp,Wu,Wd,Wr] = Design_Weights();

%% Generalized Plant - Perturbed
% Upper bound of the absolute value for the complex perturbations
bound_G = 0.5;
bound_Gd = 0.5;
[P_Delta,P_aug,Gp_app,Gd_p_app] = Generalized_Plant_Perturbed...
                (G,Gd,Gsm,bound_G,bound_Gd,W_I_G,W_I_Gd,W_I_Gsm,Wp,Wu,Wd,Wr);
%% mu-synthesis of Hinf Controller - Perturbed Plant
disp('----------- mu-synthesis controller-Perturbed Plant --------------')
opts_mu = musynOptions('MixedMU','off');
tic;
[mu_syn_data.K_full,CL_mu,mu_syn_data.info_mu] = musyn(P_Delta,nmeas,ncont);%,opts_mu); 
mu_syn_data.timerun = toc;

%% Order reduction of the controller
figure
ncfmr(mu_syn_data.K_full)
fprintf('Give the required order of the controller for error less than 1e-3: \n')
pause
order = input("");
% mu_syn_data.K = ncfmr(mu_syn_data.K_full,order);
mu_syn_data.K = bstmr(mu_syn_data.K_full,order);

% Comparison of the reduced-order with the full-order controller with the
% singular value plot
mu_syn_data.K_size = size(mu_syn_data.K.A,1);
mu_syn_data.K_full_size = size(mu_syn_data.K_full.A,1);

[mu_syn_data.sv_K,mu_syn_data.wout_K] = sigma(mu_syn_data.K);
[mu_syn_data.sv_K_full,mu_syn_data.wout_K_full] = sigma(mu_syn_data.K_full);

[fig1,fig2,~] = loglog_ss(mu_syn_data.sv_K,mu_syn_data.wout_K,...
           mu_syn_data.sv_K_full,mu_syn_data.wout_K_full,[],[],2);

legend([fig1,fig2],strcat('Reduced-order:', num2str(mu_syn_data.K_size),' states'),...
    strcat('Full-order:', num2str(mu_syn_data.K_full_size),' states'))

%% Define all the loop transfer matrices for mu-synthesis controller and hinf controller
hinf_data.loops_p_app = loopsens(Gp_app,hinf_data.K);
hinf_data.Lp_app = hinf_data.loops_p_app.Lo;
hinf_data.Tp_app = hinf_data.loops_p_app.To;
hinf_data.Sp_app = hinf_data.loops_p_app.So;

mu_syn_data.loops = loopsens(G*Gsm,mu_syn_data.K);
mu_syn_data.L = mu_syn_data.loops.Lo;
mu_syn_data.T = mu_syn_data.loops.To;
mu_syn_data.S = mu_syn_data.loops.So;

mu_syn_data.loops_p_app = loopsens(Gp_app,mu_syn_data.K);
mu_syn_data.Lp_app = mu_syn_data.loops_p_app.Lo;
mu_syn_data.Tp_app = mu_syn_data.loops_p_app.To;
mu_syn_data.Sp_app = mu_syn_data.loops_p_app.So;

mu_syn_data.loops_p = loopsens(Gp*Gsm_p,mu_syn_data.K);
mu_syn_data.Lp = mu_syn_data.loops_p.Lo;
mu_syn_data.Tp = mu_syn_data.loops_p.To;
mu_syn_data.Sp = mu_syn_data.loops_p.So;

%% Save data
save('Data Files/Controller_mu_syn.mat','mu_syn_data','Gp_app','Gd_p_app','P_Delta')
save('Data Files/Controller_hinf.mat','hinf_data','-append')