% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

%% Singular Values Plots for both hinf and mu-synthesis controllers
clc
clear all
close all

addpath('Plotting Functions')
addpath('Data Files')

load('LTI_Perturbed_Plant.mat','Gp','Gd_p','Gsm_p')
load('Parameters_Nominal.mat','param')
load('LTI_Nominal_Plant.mat','G','Gd','Gsm','foil_loc')
load('Controller_mu_syn.mat')
load('Controller_hinf.mat')
load('Uncertainty_Weighting_Functions.mat')

% Nominal plant G(s)
% Disturbances transfer matrix Gd(s)
% Perturbed plant with uncertain parameters Gp(s)
% Perturbed disturbances transfer matrix with uncertain parameters Gd_p(s)

run Bode_options.m
run Sigma_options.m

% Define the Weighting Functions for the controller synthesis
[Wp,Wu,Wd,Wr] = Design_Weights();

%% Define the Perturbed Plant and the Disturbance Transfer Matrix
% Multiplicative Input Uncertainty
% Define the Delta matrices with fully individual ultidyn elements

bound_G = 0.5;
Delta_I_G = ultidyn('Delta_I_G',[3,3],'Bound',bound_G);

Gp_app = G*(eye(3) + Delta_I_G*W_I_G);
Gp_app = minreal(Gp_app);

bound_Gd = 0.5;
Delta_I_Gd = ultidyn('Delta_I_Gd',[6,3],'Bound',bound_Gd);

Gd_p_app = Gd*(eye(6) + Delta_I_Gd*W_I_Gd);
Gd_p_app = minreal(Gd_p_app);

Delta_I_Gsm = blkdiag(ultidyn('delta_I_g_sm_f',[1,1],'Bound',1),...
                      ultidyn('delta_I_g_sm_f',[1,1],'Bound',1),...
                      ultidyn('delta_I_g_sm_f',[1,1],'Bound',1));

Gsm_p_app = Gsm*(eye(3) + Delta_I_Gsm*W_I_Gsm);
Gsm_p_app = minreal(Gsm_p_app);

%% Open-loop Plant Sigma plots
%
[sv_G,wout_G] = sigma(G);
[sv_Gd,wout_Gd] = sigma(Gd);

[fig1,fig2,~] = loglog_ss(sv_G,wout_G,sv_Gd,wout_Gd,[],[],2);

legend([fig1,fig2],'\boldmath{$\sigma(G)$}','\boldmath{$\sigma(G_d)$}',...
       'interpreter','latex','FontSize',12,'Location','best')
%}

%% Uncertainty Approximation Sigma Plots

% Plant model
omega=logspace(-1,3,100);
samples = 20;
unc_app.sv_G = sigma(G,omega);
unc_app.sv_Gp = sigma_uss(Gp,omega,samples);
unc_app.sv_Gp_app = sigma_uss(Gp_app,omega,samples);

[fig1,fig2,fig3] = loglog_uss(unc_app.sv_Gp,omega,unc_app.sv_Gp_app,omega,...
                              unc_app.sv_G,omega,samples,3);

legend([fig1,fig2,fig3],'\boldmath{$\sigma(G_{p})$}','\boldmath{$\sigma(G_{p}^a)$}',...
       '\boldmath{$\sigma(G)$}','interpreter','latex','FontSize',12,'Location','best')
%%
% Disturbance model
unc_app.sv_Gd = sigma(Gd,omega);
unc_app.sv_Gd_p = sigma_uss(Gd_p,omega,samples);
unc_app.sv_Gd_p_app = sigma_uss(Gd_p_app,omega,samples);

[fig1,fig2,fig3] = loglog_uss(unc_app.sv_Gd_p,omega,unc_app.sv_Gd_p_app,omega,...
                              unc_app.sv_Gd,omega,samples,3);

legend([fig1,fig2,fig3],'\boldmath{$\sigma(G_{d,p})$}','\boldmath{$\sigma(G_{d,p}^a)$}',...
       '\boldmath{$\sigma(G_d)$}','interpreter','latex','FontSize',12,'Location','best')

%%
% Actuator model
omega=logspace(0,3,100);
samples = 20;
unc_app.sv_Gsm = sigma(Gsm,omega);
unc_app.sv_Gsm_p = sigma_uss(Gsm_p,omega,samples);
unc_app.sv_Gsm_p_app = sigma_uss(Gsm_p_app,omega,samples);

[fig1,fig2,fig3] = loglog_uss(unc_app.sv_Gsm_p,omega,unc_app.sv_Gsm_p_app,omega,...
                              unc_app.sv_Gsm,omega,samples,3);

legend([fig1,fig2,fig3],'\boldmath{$\sigma(G_{sm,p})$}','\boldmath{$\sigma(G_{sm,p}^a)$}',...
       '\boldmath{$\sigma(G_{sm})$}','interpreter','latex','FontSize',12,'Location','best')


%% Sensitivity Nominal
omega = logspace(-5,3,100);
[weights_res.sv_Wp,~] = sigma(inv(Wp),omega);
[mu_syn_res.sv_S,~] = sigma(mu_syn_data.S,omega);
[hinf_res.sv_S,~] = sigma(hinf_data.S,omega);

[fig1,fig2,fig3] = loglog_ss(mu_syn_res.sv_S,omega,hinf_res.sv_S,omega,...
                             weights_res.sv_Wp,omega,3);

legend([fig1,fig2,fig3],'\boldmath{$\sigma(S):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(S):h_{\infty}$}','\boldmath{$\sigma(W_p^{-1})$}',...
       'interpreter','latex','FontSize',12,'Location','best')
title('Sensitivity - Nominal Plant')
%% Sensitivity - Perturbed Plant with Parametric Uncertainties
samples = 20;
omega = logspace(-5,3,100);
[weights_res.sv_Wp,~] = sigma(inv(Wp),omega);
mu_syn_res.sv_Sp = sigma_uss(mu_syn_data.Sp,omega,samples);
hinf_res.sv_Sp = sigma_uss(hinf_data.Sp,omega,samples);

[fig1,fig2,fig3] = loglog_uss(mu_syn_res.sv_Sp,omega,hinf_res.sv_Sp,omega,...
                             weights_res.sv_Wp,omega,samples,3);

legend([fig1,fig2,fig3],'\boldmath{$\sigma(S):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(S):h_{\infty}$}','\boldmath{$\sigma(W_p^{-1})$}',...
       'interpreter','latex','FontSize',12,'Location','best')
title('Sensitivity - Perturbed Plant with Parametric Uncertainties')
%% Sensitivity - Perturbed Plant with Multiplicative Input Uncertainties
samples = 20;
omega = logspace(-5,3,100);
[weights_res.sv_Wp,~] = sigma(inv(Wp),omega);
mu_syn_res.sv_Sp_app = sigma_uss(mu_syn_data.Sp_app,omega,samples);
hinf_res.sv_Sp_app = sigma_uss(hinf_data.Sp_app,omega,samples);

[fig1,fig2,fig3] = loglog_uss(mu_syn_res.sv_Sp_app,omega,hinf_res.sv_Sp_app,omega,...
                             weights_res.sv_Wp,omega,samples,3);

legend([fig1,fig2,fig3],'\boldmath{$\sigma(S):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(S):h_{\infty}$}','\boldmath{$\sigma(W_p^{-1})$}',...
       'interpreter','latex','FontSize',12,'Location','best')
title('Sensitivity - Perturbed Plant with Multiplicative Input Uncertainties')

%% Complementary Sensitivity (Closed-loop response for reference tracking)
omega=logspace(-2,3,100);
[mu_syn_res.sv_T,~] = sigma(mu_syn_data.T,omega);
[hinf_res.sv_T,~] = sigma(hinf_data.T,omega);

[fig1,fig2,~] = loglog_ss(mu_syn_res.sv_T,omega,hinf_res.sv_T,omega,[],[],2);

legend([fig1,fig2],'\boldmath{$\sigma(T):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(T):h_{\infty}$}','interpreter','latex','FontSize',12,...
       'Location','best')
title('Complementary Sensitivity - Nominal Plant')

%% Complementary Sensitivity - Perturbed Plant with Parametric Uncertainties
samples = 15;
omega=logspace(-2,3,100);
mu_syn_res.sv_Tp = sigma_uss(mu_syn_data.Tp,omega,samples);
hinf_res.sv_Tp = sigma_uss(hinf_data.Tp,omega,samples);

[fig1,fig2,~] = loglog_uss(mu_syn_res.sv_Tp,omega,hinf_res.sv_Tp,omega,...
                             [],[],samples,1);

legend([fig1,fig2],'\boldmath{$\sigma(T):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(T):h_{\infty}$}','interpreter','latex','FontSize',...
       12,'Location','best')
title('Sensitivity - Perturbed Plant with Parametric Uncertainties')
%% Complementary Sensitivity - Perturbed Plant with Multiplicative Input Uncertainties
samples = 15;
omega=logspace(-2,3,100);
mu_syn_res.sv_Tp_app = sigma_uss(mu_syn_data.Tp_app,omega,samples);
hinf_res.sv_Tp_app = sigma_uss(hinf_data.Tp_app,omega,samples);

[fig1,fig2,~] = loglog_uss(mu_syn_res.sv_Tp_app,omega,hinf_res.sv_Tp_app,omega,...
                             [],[],samples,1);

legend([fig1,fig2],'\boldmath{$\sigma(T):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(T):h_{\infty}$}','interpreter','latex','FontSize',...
       12,'Location','best')
title('Sensitivity - Perturbed Plant with Multiplicative Input Uncertainties')
%% KSWr (Contribution of reference signal to the control signal)
omega=logspace(-3,6,200);
[weights_res.sv_Wu,~] = sigma(inv(Wu),omega);
[mu_syn_res.sv_KS,~] = sigma(mu_syn_data.K*mu_syn_data.S*Wr,omega);
[hinf_res.sv_KS,~] = sigma(hinf_data.K*hinf_data.S*Wr,omega);

[fig1,fig2,fig3] = loglog_ss(mu_syn_res.sv_KS,omega,hinf_res.sv_KS,omega,...
                                 weights_res.sv_Wu,omega,3);

legend([fig1,fig2,fig3],'\boldmath{$\sigma(KSWr):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(KSWr):h_{\infty}$}','\boldmath{$\sigma(W_u^{-1})$}',...
       'interpreter','latex','FontSize',12,'Location','best')
%% Loop Tranfer Function Matrix
omega=logspace(-5,4,200);
[mu_syn_res.sv_L,~] = sigma(mu_syn_data.L,omega);
[hinf_res.sv_L,~] = sigma(hinf_data.L,omega);

[fig1,fig2,~] = loglog_ss(mu_syn_res.sv_L,omega,hinf_res.sv_L,omega,[],[],2);

legend([fig1,fig2],'\boldmath{$\sigma(L):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(L):h_{\infty}$}','interpreter','latex','FontSize',12,...
       'Location','best')

%% Loop Tranfer Function Matrix - Perturbed Plant with Parametric Uncertainties
samples = 15;
omega=logspace(-5,4,200);
mu_syn_res.sv_Lp = sigma_uss(mu_syn_data.Lp,omega,samples);
hinf_res.sv_Lp = sigma_uss(hinf_data.Lp,omega,samples);

[fig1,fig2,~] = loglog_uss(mu_syn_res.sv_Lp,omega,hinf_res.sv_Lp,omega,...
                             [],[],samples,1);

legend([fig1,fig2],'\boldmath{$\sigma(L):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(L):h_{\infty}$}','interpreter','latex','FontSize',...
       12,'Location','best')
title('Loop - Perturbed Plant with Parametric Uncertainties')
%% Loop Tranfer Function Matrix - Perturbed Plant with Multiplicative Input Uncertainties
samples = 15;
omega=logspace(-5,4,200);
mu_syn_res.sv_Lp_app = sigma_uss(mu_syn_data.Lp_app,omega,samples);
hinf_res.sv_Lp_app = sigma_uss(hinf_data.Lp_app,omega,samples);

[fig1,fig2,~] = loglog_uss(mu_syn_res.sv_Lp_app,omega,hinf_res.sv_Lp_app,omega,...
                             [],[],samples,1);

legend([fig1,fig2],'\boldmath{$\sigma(L):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(L):h_{\infty}$}','interpreter','latex','FontSize',...
       12,'Location','best')
title('Loop - Perturbed Plant with Multiplicative Input Uncertainties')
%% KSGd (Contribution of disturbance to the control signal)
omega = logspace(-5,4,200);
[weights_res.sv_Wu,~] = sigma(inv(Wu),omega);
[mu_syn_res.sv_KSGd,~] = sigma(mu_syn_data.K*mu_syn_data.S*Gd*Wd,omega);
[hinf_res.sv_KSGd,~] = sigma(hinf_data.K*hinf_data.S*Gd*Wd,omega);

[fig1,fig2,fig3] = loglog_ss(mu_syn_res.sv_KSGd,omega,hinf_res.sv_KSGd,...
                                omega,weights_res.sv_Wu,omega,3);

legend([fig1,fig2,fig3],'\boldmath{$\sigma(KSGdWd):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(KSGdWd):h_{\infty}$}','\boldmath{$\sigma(W_u^{-1})$}','interpreter','latex','FontSize',12,...
       'Location','best')
%% SGd (Contribution of disturbance to the error)
omega = logspace(-5,3,200);
[mu_syn_res.sv_SGd,~] = sigma(mu_syn_data.S*Gd,omega);
[hinf_res.sv_SGd,~] = sigma(hinf_data.S*Gd,omega);

[fig1,fig2,~] = loglog_ss(mu_syn_res.sv_SGd,omega,hinf_res.sv_SGd,omega,[],[],2);

legend([fig1,fig2],'\boldmath{$\sigma(SGd):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(SGd):h_{\infty}$}','interpreter','latex','FontSize',12,...
       'Location','best')
%% SGd - Perturbed Plant with Parametric Uncertainties
samples = 15;
omega=logspace(-5,3,200);
mu_syn_res.sv_SpGd_p = sigma_uss(mu_syn_data.Sp*Gd_p,omega,samples);
hinf_res.sv_SpGd_p = sigma_uss(hinf_data.Sp*Gd_p,omega,samples);

[fig1,fig2,~] = loglog_uss(mu_syn_res.sv_SpGd_p,omega,hinf_res.sv_SpGd_p,omega,...
                           [],[],samples,1);

legend([fig1,fig2],'\boldmath{$\sigma(SGd):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(SGd):h_{\infty}$}','interpreter','latex','FontSize',...
       12,'Location','best')
title('SGd - Perturbed Plant with Parametric Uncertainties')
%% SGd - Perturbed Plant with Multiplicative Input Uncertainties
samples = 15;
omega=logspace(-5,3,200);
mu_syn_res.sv_Sp_appGd_p_app = sigma_uss(mu_syn_data.Sp_app*Gd_p_app,omega,samples);
hinf_res.sv_Sp_appGd_p_app = sigma_uss(hinf_data.Sp_app*Gd_p_app,omega,samples);

[fig1,fig2,~] = loglog_uss(mu_syn_res.sv_Sp_appGd_p_app,omega,hinf_res.sv_Sp_appGd_p_app,...
                           omega,[],[],samples,1);

legend([fig1,fig2],'\boldmath{$\sigma(SGd):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(SGd):h_{\infty}$}','interpreter','latex','FontSize',...
       12,'Location','best')
title('SGd - Perturbed Plant with Multiplicative Input Uncertainties')
%% K (Controller)
omega = logspace(-6,6,200);
[mu_syn_res.sv_K,~] = sigma(mu_syn_data.K,omega);
[hinf_res.sv_K,~] = sigma(hinf_data.K,omega);

[fig1,fig2,~] = loglog_ss(mu_syn_res.sv_K,omega,hinf_res.sv_K,omega,[],[],2);

legend([fig1,fig2],'\boldmath{$\sigma(K):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(K):h_{\infty}$}','interpreter','latex','FontSize',12,...
       'Location','best')