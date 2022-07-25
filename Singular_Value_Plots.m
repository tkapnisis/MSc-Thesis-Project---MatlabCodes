% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

%% Singular Values Plots for both hinf and mu-synthesis controllers
clc
clear all
% close all

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

%%
%% Define the Perturbed Plant and the Disturbance Transfer Matrix
% Multiplicative Input Uncertainty
% Define the Delta matrices with fully individual ultidyn elements

bound_G = 1;
Delta_I_G = ultidyn('Delta_I_G',[3,3],'Bound',bound_G);

Gp_app = G*(eye(3) + Delta_I_G*W_I_G);
Gp_app = minreal(Gp_app);

bound_Gd = 1;
Delta_I_Gd = ultidyn('Delta_I_Gd',[6,3],'Bound',bound_Gd);

Gd_p_app = Gd*(eye(6) + Delta_I_Gd*W_I_Gd);
Gd_p_app = minreal(Gd_p_app);

Delta_I_Gsm = blkdiag(ultidyn('delta_I_g_sm_f',[1,1],'Bound',1),...
                      ultidyn('delta_I_g_sm_f',[1,1],'Bound',1),...
                      ultidyn('delta_I_g_sm_f',[1,1],'Bound',1));

Gsm_p_app = Gsm*(eye(3) + Delta_I_Gsm*W_I_Gsm);
Gsm_p_app = minreal(Gsm_p_app);

%% Open-loop Plant Sigma plots
%{
[sv_G,wout_G] = sigma(G);
[sv_Gd,wout_Gd] = sigma(Gd);

[fig1,fig2,~] = loglog_custom(sv_G,wout_G,...
           sv_Gd,wout_Gd,[],[],2);

legend([fig1,fig2],'\boldmath{$\sigma(G)$}','\boldmath{$\sigma(G_d)$}',...
       'interpreter','latex','FontSize',12,'Location','best')
%}

%% Uncertainty Approximation Sigma Plots

% Plant model
omega=logspace(-1,3,100);
samples = 25;
unc_app.sv_G = sigma(G,omega);
unc_app.sv_Gp = sigma_uss(Gp,omega,samples);
unc_app.sv_Gp_app = sigma_uss(Gp_app,omega,samples);

[fig1,fig2,fig3] = loglog_uss(unc_app.sv_Gp,omega,unc_app.sv_Gp_app,omega,...
                              unc_app.sv_G,omega,samples,3);

legend([fig1,fig2,fig3],'\boldmath{$\sigma(G_{p})$}','\boldmath{$\sigma(G_{p}^a)$}',...
       '\boldmath{$\sigma(G)$}','interpreter','latex','FontSize',12,'Location','best')
%%
% Disturbance model
omega=logspace(-1,3,100);
samples = 25;
unc_app.sv_Gd = sigma(Gd,omega);
unc_app.sv_Gd_p = sigma_uss(Gd_p,omega,samples);
unc_app.sv_Gd_p_app = sigma_uss(Gd_p_app,omega,samples);

[fig1,fig2,fig3] = loglog_uss(unc_app.sv_Gd_p,omega,unc_app.sv_Gd_p_app,omega,...
                              unc_app.sv_Gd,omega,samples,3);

legend([fig1,fig2,fig3],'\boldmath{$\sigma(G_{d,p})$}','\boldmath{$\sigma(G_{d,p}^a)$}',...
       '\boldmath{$\sigma(G_d)$}','interpreter','latex','FontSize',12,'Location','best')

%%
% Actuator model
omega=logspace(-1,3,100);
samples = 25;
unc_app.sv_Gsm = sigma(Gsm,omega);
unc_app.sv_Gsm_p = sigma_uss(Gsm_p,omega,samples);
unc_app.sv_Gsm_p_app = sigma_uss(Gsm_p_app,omega,samples);

[fig1,fig2,fig3] = loglog_uss(unc_app.sv_Gsm_p,omega,unc_app.sv_Gsm_p_app,omega,...
                              unc_app.sv_Gsm,omega,samples,3);

legend([fig1,fig2,fig3],'\boldmath{$\sigma(G_{sm,p})$}','\boldmath{$\sigma(G_{sm,p}^a)$}',...
       '\boldmath{$\sigma(G_{sm})$}','interpreter','latex','FontSize',12,'Location','best')


%% Sensitivity
% figure
% sigma(hinf_data.S,mu_syn_data.S,inv(Wp),opts_sigma);
% legend('\boldmath{$\sigma(S)$}','interpreter','latex','FontSize',15)

omega=logspace(-5,3,200);
[weights.sv_Wp,weights.wout_Wp] = sigma(inv(Wp),omega);
[mu_syn_data.sv_S,mu_syn_data.wout_S] = sigma(mu_syn_data.S,omega);
[hinf_data.sv_S,hinf_data.wout_S] = sigma(hinf_data.S,omega);

[fig1,fig2,fig3] = loglog_custom(mu_syn_data.sv_S,mu_syn_data.wout_S,...
           hinf_data.sv_S,hinf_data.wout_S,weights.sv_Wp,weights.wout_Wp,3);

legend([fig1,fig2,fig3],'\boldmath{$\sigma(S):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(S):h_{\infty}$}','\boldmath{$\sigma(W_p^{-1})$}',...
       'interpreter','latex','FontSize',12,'Location','best')

%% Complementary Sensitivity (Closed-loop response for reference tracking)
% 
% figure
% sigma(hinf_data.T,mu_syn_data.T,opts_sigma);
% legend('\boldmath{$\sigma(T)$}','interpreter','latex','FontSize',15)

omega=logspace(-2,3,200);
[mu_syn_data.sv_T,mu_syn_data.wout_T] = sigma(mu_syn_data.T,omega);
[hinf_data.sv_T,hinf_data.wout_T] = sigma(hinf_data.T,omega);

[fig1,fig2,~] = loglog_custom(mu_syn_data.sv_T,mu_syn_data.wout_T,...
                              hinf_data.sv_T,hinf_data.wout_T,[],[],2);

legend([fig1,fig2],'\boldmath{$\sigma(T):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(T):h_{\infty}$}','interpreter','latex','FontSize',12,...
       'Location','best')

%% Controller multiplied by Sensitivity (Contribution of reference to the control signal)
% figure
% sigma(hinf_data.K*hinf_data.S,mu_syn_data.K*mu_syn_data.S,inv(Wu),opts_sigma);
% legend('\boldmath{$\sigma(KS)$}','interpreter','latex','FontSize',15)

omega=logspace(-3,6,200);
[weights.sv_Wu,weights.wout_Wu] = sigma(inv(Wu),omega);
[mu_syn_data.sv_KS,mu_syn_data.wout_KS] = sigma(mu_syn_data.K*mu_syn_data.S*Wr,omega);
[hinf_data.sv_KS,hinf_data.wout_KS] = sigma(hinf_data.K*hinf_data.S*Wr,omega);

[fig1,fig2,fig3] = loglog_custom(mu_syn_data.sv_KS,mu_syn_data.wout_KS,...
           hinf_data.sv_KS,hinf_data.wout_KS,weights.sv_Wu,weights.wout_Wu,3);

legend([fig1,fig2,fig3],'\boldmath{$\sigma(KSWr):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(KSWr):h_{\infty}$}','\boldmath{$\sigma(W_u^{-1})$}',...
       'interpreter','latex','FontSize',12,'Location','best')
%% Loop Tranfer Function Matrix
% figure
% sigma(hinf_data.L,mu_syn_data.L,opts_sigma);
% legend('\boldmath{$\sigma(GK)$}','interpreter','latex','FontSize',15)

omega=logspace(-5,4,200);
[mu_syn_data.sv_L,mu_syn_data.wout_L] = sigma(mu_syn_data.L,omega);
[hinf_data.sv_L,hinf_data.wout_L] = sigma(hinf_data.L,omega);

[fig1,fig2,~] = loglog_custom(mu_syn_data.sv_L,mu_syn_data.wout_L,...
           hinf_data.sv_L,hinf_data.wout_L,[],[],2);

legend([fig1,fig2],'\boldmath{$\sigma(L):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(L):h_{\infty}$}','interpreter','latex','FontSize',12,...
       'Location','best')

%% KSGd (Contribution of disturbance to the control signal)
% figure
% sigma(hinf_data.K*hinf_data.S*Gd,mu_syn_data.K*mu_syn_data.S*Gd,opts_sigma);
% legend('\boldmath{$\sigma(K S G_d)$}','interpreter','latex','FontSize',15)

omega=logspace(-5,4,200);
[weights.sv_Wu,weights.wout_Wu] = sigma(inv(Wu),omega);
[mu_syn_data.sv_KSGd,mu_syn_data.wout_KSGd] = sigma(mu_syn_data.K*mu_syn_data.S*Gd*Wd,omega);
[hinf_data.sv_KSGd,hinf_data.wout_KSGd] = sigma(hinf_data.K*hinf_data.S*Gd*Wd,omega);

[fig1,fig2,fig3] = loglog_custom(mu_syn_data.sv_KSGd,mu_syn_data.wout_KSGd,...
           hinf_data.sv_KSGd,hinf_data.wout_KSGd,weights.sv_Wu,weights.wout_Wu,3);

legend([fig1,fig2,fig3],'\boldmath{$\sigma(KSGdWd):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(KSGdWd):h_{\infty}$}','\boldmath{$\sigma(W_u^{-1})$}','interpreter','latex','FontSize',12,...
       'Location','best')
%% SGd (Contribution of disturbance to the error)
% figure
% sigma(hinf_data.S*Gd,mu_syn_data.S*Gd,opts_sigma);
% legend('\boldmath{$\sigma(SGd)$}','interpreter','latex','FontSize',15)

omega=logspace(-5,3,200);
[mu_syn_data.sv_SGd,mu_syn_data.wout_SGd] = sigma(mu_syn_data.S*Gd,omega);
[hinf_data.sv_SGd,hinf_data.wout_SGd] = sigma(hinf_data.S*Gd,omega);

[fig1,fig2,~] = loglog_custom(mu_syn_data.sv_SGd,mu_syn_data.wout_SGd,...
           hinf_data.sv_SGd,hinf_data.wout_SGd,[],[],2);

legend([fig1,fig2],'\boldmath{$\sigma(SGd):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(SGd):h_{\infty}$}','interpreter','latex','FontSize',12,...
       'Location','best')

%% K (Controller)
% figure
% sigma(hinf_data.K,mu_syn_data.K,opts_sigma);
% legend('\boldmath{$\sigma(K)$}','interpreter','latex','FontSize',15)

omega=logspace(-6,6,200);
[mu_syn_data.sv_K,mu_syn_data.wout_K] = sigma(mu_syn_data.K,omega);
[hinf_data.sv_K,hinf_data.wout_K] = sigma(hinf_data.K,omega);

[fig1,fig2,~] = loglog_custom(mu_syn_data.sv_K,mu_syn_data.wout_K,...
           hinf_data.sv_K,hinf_data.wout_K,[],[],2);

legend([fig1,fig2],'\boldmath{$\sigma(K):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(K):h_{\infty}$}','interpreter','latex','FontSize',12,...
       'Location','best')


%% Controller multiplied by Sensitivity (Contribution of reference to the control signal)
% figure
% sigma(hinf_data.K*hinf_data.S,mu_syn_data.K*mu_syn_data.S,inv(Wu),opts_sigma);
% legend('\boldmath{$\sigma(KS)$}','interpreter','latex','FontSize',15)

omega=logspace(-4,6,200);
[weights.sv_Wu,weights.wout_Wu] = sigma(inv(Wu),omega);
[mu_syn_data.sv_KS,mu_syn_data.wout_KS] = sigma(mu_syn_data.K*mu_syn_data.S,omega);
[hinf_data.sv_KS,hinf_data.wout_KS] = sigma(hinf_data.K*hinf_data.S,omega);
[mu_syn_data.sv_KSGd,mu_syn_data.wout_KSGd] = sigma(mu_syn_data.K*mu_syn_data.S*Gd,omega);
[hinf_data.sv_KSGd,hinf_data.wout_KSGd] = sigma(hinf_data.K*hinf_data.S*Gd,omega);

[fig1,fig2,fig3] = loglog_custom(mu_syn_data.sv_KS+mu_syn_data.sv_KSGd,mu_syn_data.wout_KS,...
           hinf_data.sv_KS+hinf_data.sv_KSGd,hinf_data.wout_KS,weights.sv_Wu,weights.wout_Wu,3);

legend([fig1,fig2,fig3],'\boldmath{$\sigma(KS):\mu-$}\textbf{synthesis}',...
       '\boldmath{$\sigma(KS):h_{\infty}$}','\boldmath{$\sigma(W_u^{-1})$}',...
       'interpreter','latex','FontSize',12,'Location','best')
