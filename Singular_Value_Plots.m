% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

clc
clear all
close all

addpath('Plotting Functions')
addpath('Data Files')

load('LTI_Perturbed_Plant.mat','G','Gd','Gp','Gd_p')
load('Parameters_Nominal.mat','param')
load('LTI_Nominal_Plant.mat','foil_loc')
load('Controllers.mat')
% Nominal plant G(s)
% Disturbances transfer matrix Gd(s)
% Perturbed plant with uncertain parameters Gp(s)
% Perturbed disturbances transfer matrix with uncertain parameters Gd_p(s)

nmeas = 3; % number of outputs 
ncont = 3; % number of inputs

run Bode_options.m
run Sigma_options.m

% Define the Weighting Functions for the controller synthesis
[Wp,Wu,Wd,Wr,Gact,Gact_p,Wact] = Design_Weights();
%% Singular Values Plots for both hinf and mu-synthesis controllers

%% Sensitivity
% figure
% sigma(hinf_data.S,mu_syn_data.S,inv(Wp),opts_sigma);
% legend('\boldmath{$\sigma(S)$}','interpreter','latex','FontSize',15)

omega=logspace(-5,3,200);
[weights.sv_Wp,weights.wout_Wp] = sigma(inv(Wp),omega);
[mu_syn_data.sv_S,mu_syn_data.wout_S] = sigma(mu_syn_data.S,omega);
[hinf_data.sv_S,hinf_data.wout_S] = sigma(hinf_data.S,omega);

[fig1,fig2,fig3] = loglog_custom(mu_syn_data.sv_S,mu_syn_data.wout_S,...
           hinf_data.sv_S,hinf_data.wout_S,weights.sv_Wp,weights.wout_Wp,'weight');

legend([fig1,fig2,fig3],'\boldmath{$\sigma(W_p^{-1})$}',...
       '\boldmath{$\sigma(S):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(S):h_{\infty}$}','interpreter','latex','FontSize',12,...
       'Location','best')

%% Complementary Sensitivity (Closed-loop response for reference tracking)
% 
% figure
% sigma(hinf_data.T,mu_syn_data.T,opts_sigma);
% legend('\boldmath{$\sigma(T)$}','interpreter','latex','FontSize',15)

omega=logspace(-2,3,200);
[mu_syn_data.sv_T,mu_syn_data.wout_T] = sigma(mu_syn_data.T,omega);
[hinf_data.sv_T,hinf_data.wout_T] = sigma(hinf_data.T,omega);

[~,fig2,fig3] = loglog_custom(mu_syn_data.sv_T,mu_syn_data.wout_T,...
           hinf_data.sv_T,hinf_data.wout_T,[],[],'contr');

legend([fig2,fig3],'\boldmath{$\sigma(T):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(T):h_{\infty}$}','interpreter','latex','FontSize',12,...
       'Location','best')

%% Controller multiplied by Sensitivity (Contribution of reference to the control signal)
% figure
% sigma(hinf_data.K*hinf_data.S,mu_syn_data.K*mu_syn_data.S,inv(Wu),opts_sigma);
% legend('\boldmath{$\sigma(KS)$}','interpreter','latex','FontSize',15)

omega=logspace(-3,6,200);
[weights.sv_Wu,weights.wout_Wu] = sigma(inv(Wu),omega);
[mu_syn_data.sv_KS,mu_syn_data.wout_KS] = sigma(mu_syn_data.K*mu_syn_data.S,omega);
[hinf_data.sv_KS,hinf_data.wout_KS] = sigma(hinf_data.K*hinf_data.S,omega);

[fig1,fig2,fig3] = loglog_custom(mu_syn_data.sv_KS,mu_syn_data.wout_KS,...
           hinf_data.sv_KS,hinf_data.wout_KS,weights.sv_Wu,weights.wout_Wu,'weight');

legend([fig1,fig2,fig3],'\boldmath{$\sigma(W_u^{-1})$}',...
       '\boldmath{$\sigma(KS):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(KS):h_{\infty}$}','interpreter','latex','FontSize',12,...
       'Location','best')
%% Loop Tranfer Function Matrix
% figure
% sigma(hinf_data.L,mu_syn_data.L,opts_sigma);
% legend('\boldmath{$\sigma(GK)$}','interpreter','latex','FontSize',15)

omega=logspace(-5,4,200);
[mu_syn_data.sv_L,mu_syn_data.wout_L] = sigma(mu_syn_data.L,omega);
[hinf_data.sv_L,hinf_data.wout_L] = sigma(hinf_data.L,omega);

[~,fig2,fig3] = loglog_custom(mu_syn_data.sv_L,mu_syn_data.wout_L,...
           hinf_data.sv_L,hinf_data.wout_L,[],[],'contr');

legend([fig2,fig3],'\boldmath{$\sigma(L):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(L):h_{\infty}$}','interpreter','latex','FontSize',12,...
       'Location','best')

%% KSGd (Contribution of disturbance to the control signal)
% figure
% sigma(hinf_data.K*hinf_data.S*Gd,mu_syn_data.K*mu_syn_data.S*Gd,opts_sigma);
% legend('\boldmath{$\sigma(K S G_d)$}','interpreter','latex','FontSize',15)

omega=logspace(-5,4,200);
[mu_syn_data.sv_KSGd,mu_syn_data.wout_KSGd] = sigma(mu_syn_data.K*mu_syn_data.S*Gd,omega);
[hinf_data.sv_KSGd,hinf_data.wout_KSGd] = sigma(hinf_data.K*hinf_data.S*Gd,omega);

[~,fig2,fig3] = loglog_custom(mu_syn_data.sv_KSGd,mu_syn_data.wout_KSGd,...
           hinf_data.sv_KSGd,hinf_data.wout_KSGd,[],[],'contr');

legend([fig2,fig3],'\boldmath{$\sigma(KSGd):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(KSGd):h_{\infty}$}','interpreter','latex','FontSize',12,...
       'Location','best')
%% SGd (Contribution of disturbance to the error)
% figure
% sigma(hinf_data.S*Gd,mu_syn_data.S*Gd,opts_sigma);
% legend('\boldmath{$\sigma(SGd)$}','interpreter','latex','FontSize',15)

omega=logspace(-5,3,200);
[mu_syn_data.sv_SGd,mu_syn_data.wout_SGd] = sigma(mu_syn_data.S*Gd,omega);
[hinf_data.sv_SGd,hinf_data.wout_SGd] = sigma(hinf_data.S*Gd,omega);

[~,fig2,fig3] = loglog_custom(mu_syn_data.sv_SGd,mu_syn_data.wout_SGd,...
           hinf_data.sv_SGd,hinf_data.wout_SGd,[],[],'contr');

legend([fig2,fig3],'\boldmath{$\sigma(SGd):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(SGd):h_{\infty}$}','interpreter','latex','FontSize',12,...
       'Location','best')
