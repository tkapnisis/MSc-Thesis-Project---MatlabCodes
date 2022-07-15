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

% Time duration of simulations
dt = 0.05; % sampling time
tend = 20; % duration of simulation in seconds
t = 0:dt:tend;

%% Simulations for reference tracking with square signal on heave
ref = [-0.05*square(2*pi/10*t);0*ones(size(t));0*ones(size(t))];

% Number of samples for simulating the uncertain systems
samples = 20;

% Calculation of the outputs
mu_syn_data.y_p_app_ref = lsim_uss(mu_syn_data.Tp_app,ref,t,samples);
mu_syn_data.y_p_ref = lsim_uss(mu_syn_data.Tp,ref,t,samples);
mu_syn_data.y_ref = lsim(mu_syn_data.T,ref,t);

hinf_data.y_p_app_ref = lsim_uss(hinf_data.Tp_app,ref,t,samples);
hinf_data.y_p_ref = lsim_uss(hinf_data.Tp,ref,t,samples);
hinf_data.y_ref = lsim(hinf_data.T,ref,t);

% Calculation of the control inputs
mu_syn_data.u_p_app_ref = lsim_uss(mu_syn_data.K*mu_syn_data.Sp_app,ref,t,samples);
mu_syn_data.u_p_ref = lsim_uss(mu_syn_data.K*mu_syn_data.Sp,ref,t,samples);
mu_syn_data.u_ref = lsim(mu_syn_data.K*mu_syn_data.S,ref,t);

hinf_data.u_p_app_ref = lsim_uss(hinf_data.K*hinf_data.Sp_app,ref,t,samples);
hinf_data.u_p_ref = lsim_uss(hinf_data.K*hinf_data.Sp,ref,t,samples);
hinf_data.u_ref = lsim(hinf_data.K*hinf_data.S,ref,t);

%%
figure
fig1 = plot_uss_states(t,mu_syn_data.y_p_app_ref,samples,param.z_n0,0.5,'-','#0072BD');
fig2 = plot_uss_states(t,mu_syn_data.y_p_ref,samples,param.z_n0,0.5,'--','#77AC30');
fig3 = plot_ss_states(t,mu_syn_data.y_ref,[],param.z_n0,1,'--','red','ref_no_leg');

legend([fig1,fig2,fig3],'\boldmath{$\mu$} \textbf{-synthesis-Multiplicative}',...
       '\boldmath{$\mu$} \textbf{-synthesis-Parametric}',...
       '\boldmath{$\mu$} \textbf{-synthesis-Nominal}','interpreter','latex',...
       'FontSize',10)

figure
fig1 = plot_uss_states(t,hinf_data.y_p_app_ref,samples,param.z_n0,0.5,'-','#0072BD');
fig2 = plot_uss_states(t,hinf_data.y_p_ref,samples,param.z_n0,0.5,'--','#77AC30');
fig3 = plot_ss_states(t,hinf_data.y_ref,[],param.z_n0, 1,'--','red','ref_no_leg');

legend([fig1,fig2,fig3],'\boldmath{$h_{\infty}$} \textbf{ controller-Multiplicative}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller-Parametric}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller-Nominal}','interpreter','latex',...
       'FontSize',10)

figure
fig1 = plot_uss_states(t,mu_syn_data.y_p_ref,samples,param.z_n0,0.5,'-','blue');
fig2 = plot_uss_states(t,hinf_data.y_p_ref,samples,param.z_n0,0.5,'--','red');

subplot(3,1,1)
grid minor
subplot(3,1,2)
grid minor
subplot(3,1,3)
grid minor
legend([fig1,fig2],'\boldmath{$\mu$} \textbf{-synthesis}',...
       '\boldmath{$h_{\infty}$}','interpreter','latex','FontSize',10)


figure
fig1 = plot_ss_states(t,mu_syn_data.y_ref,[],param.z_n0, 1,'-','#D95319','ref_no_leg');
fig2 = plot_ss_states(t,hinf_data.y_ref,[],param.z_n0, 1,'-','#0072BD','ref_no_leg');

subplot(3,1,1)
grid minor
subplot(3,1,2)
grid minor
subplot(3,1,3)
grid minor

legend([fig1,fig2],'\boldmath{$\mu$} \textbf{-synthesis controller}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller}','interpreter','latex')


u_eq = [param.theta_s_f0,param.theta_s_ap0,param.theta_s_as0];
figure
subplot(2,1,1)
plot_ss_inputs(t,mu_syn_data.u_ref,u_eq)
title('\textbf{Control Inputs - \boldmath{$\mu$}-synthesis}','interpreter','latex','FontSize',12)
subplot(2,1,2)
plot_ss_inputs(t,hinf_data.u_ref,u_eq)
title('\textbf{Control Inputs - \boldmath{$h_{\infty}$}}','interpreter','latex','FontSize',12)

figure
subplot(2,1,1)
plot_uss_inputs(t,mu_syn_data.u_p_ref,u_eq,samples)
title('\textbf{Control Inputs - \boldmath{$\mu$}-synthesis - Parametric Uncertainty}',...
      'interpreter','latex','FontSize',12)
subplot(2,1,2)
plot_uss_inputs(t,mu_syn_data.u_p_app_ref,u_eq,samples)
title('\textbf{Control Inputs - \boldmath{$\mu$}-synthesis - Multiplicative Uncertainty}',...
      'interpreter','latex','FontSize',12)

figure
subplot(2,1,1)
plot_uss_inputs(t,hinf_data.u_p_ref,u_eq,samples)
title('\textbf{Control Inputs - \boldmath{$h_{\infty}$} - Parametric Uncertainty}',...
      'interpreter','latex','FontSize',12)
subplot(2,1,2)
plot_uss_inputs(t,hinf_data.u_p_app_ref,u_eq,samples)
title('\textbf{Control Inputs - \boldmath{$h_{\infty}$} - Multiplicative Uncertainty}',...
      'interpreter','latex','FontSize',12)

figure
subplot(2,1,1)
plot_uss_inputs(t,mu_syn_data.u_p_ref,u_eq,samples)
title('\textbf{Control Inputs - \boldmath{$\mu$}-synthesis - Parametric Uncertainty}',...
      'interpreter','latex','FontSize',12)
subplot(2,1,2)
plot_uss_inputs(t,hinf_data.u_p_ref,u_eq,samples)
title('\textbf{Control Inputs - \boldmath{$h_{\infty}$} - Parametric Uncertainty}',...
      'interpreter','latex','FontSize',12)
%% Simulation of the closed loop system with regular waves
%
%  Calculation of waves velocity profile for each hydrofoil

% Parameters of long-crested regular wave
wave_param.omega_0 = 1.5;   % Wave frequency [rad/s]
wave_param.lambda = 0.5;    % Wave length [m]
wave_param.zeta_0 = 0.1;  % Wave amplitude [m]
wave_param.beta = pi;     % Encounter angle (beta=0 for following waves) [rad] 

[dw,wave_param] = Wave_Model(t,wave_param,foil_loc,param);

% Number of samples for simulating the uncertain systems
samples = 20;

% Calculation of the outputs
mu_syn_data.y_p_app_dist = lsim_uss(mu_syn_data.Sp_app*Gd_p_app,dw,t,samples);
mu_syn_data.y_p_dist = lsim_uss(mu_syn_data.Sp*Gd_p,dw,t,samples);
mu_syn_data.y_dist = lsim(mu_syn_data.S*Gd,dw,t);

hinf_data.y_p_app_dist = lsim_uss(hinf_data.Sp_app*Gd_p_app,dw,t,samples);
hinf_data.y_p_dist = lsim_uss(hinf_data.Sp*Gd_p,dw,t,samples);
hinf_data.y_dist = lsim(hinf_data.S*Gd,dw,t);

% Calculation of the control inputs
mu_syn_data.u_p_app_dist = lsim_uss(-mu_syn_data.K*mu_syn_data.Sp_app*Gd_p_app,dw,t,samples);
mu_syn_data.u_p_dist = lsim_uss(-mu_syn_data.K*mu_syn_data.Sp*Gd_p,dw,t,samples);
mu_syn_data.u_dist = lsim(-mu_syn_data.K*mu_syn_data.S*Gd,dw,t);

hinf_data.u_p_app_dist = lsim_uss(-hinf_data.K*hinf_data.Sp_app*Gd_p_app,dw,t,samples);
hinf_data.u_p_dist = lsim_uss(-hinf_data.K*hinf_data.Sp*Gd_p,dw,t,samples);
hinf_data.u_dist = lsim(-hinf_data.K*hinf_data.S*Gd,dw,t);

figure
fig1 = plot_uss_states(t,mu_syn_data.y_p_app_dist,samples,param.z_n0,0.5,'-','#0072BD');
fig2 = plot_uss_states(t,mu_syn_data.y_p_dist,samples,param.z_n0,0.5,'-','#77AC30');
fig3 = plot_ss_states(t,mu_syn_data.y_dist,[],param.z_n0,1,'--','red','ref_no_leg');

legend([fig1,fig2,fig3],'\boldmath{$\mu$} \textbf{-synthesis-Multiplicative}',...
       '\boldmath{$\mu$} \textbf{-synthesis-Parametric}',...
       '\boldmath{$\mu$} \textbf{-synthesis-Nominal}','interpreter','latex')

figure
fig1 = plot_uss_states(t,hinf_data.y_p_app_dist,samples,param.z_n0,0.5,'-','#0072BD');
fig2 = plot_uss_states(t,hinf_data.y_p_dist,samples,param.z_n0,0.5,'-','#77AC30');
fig3 = plot_ss_states(t,hinf_data.y_dist,[],param.z_n0, 1,'--','red','ref_no_leg');

legend([fig1,fig2,fig3],'\boldmath{$h_{\infty}$} \textbf{ controller-Multiplicative}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller-Parametric}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller-Nominal}','interpreter','latex')

figure
fig1 = plot_uss_states(t,mu_syn_data.y_p_dist,samples,param.z_n0,0.5,'-','blue');
fig2 = plot_uss_states(t,hinf_data.y_p_dist,samples,param.z_n0,0.5,'-','red');

subplot(3,1,1)
grid minor
subplot(3,1,2)
grid minor
subplot(3,1,3)
grid minor
legend([fig1,fig2],'\boldmath{$\mu$} \textbf{-synthesis}',...
       '\boldmath{$h_{\infty}$}','interpreter','latex')

figure
fig1 = plot_ss_states(t,mu_syn_data.y_dist,[],param.z_n0, 1,'-','#D95319','ref_no_leg');
fig2 = plot_ss_states(t,hinf_data.y_dist,[],param.z_n0, 1,'-','#0072BD','ref_no_leg');

subplot(3,1,1)
grid minor
subplot(3,1,2)
grid minor
subplot(3,1,3)
grid minor

legend([fig1,fig2],'\boldmath{$\mu$} \textbf{-synthesis controller}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller}','interpreter','latex')

u_eq = [param.theta_s_f0,param.theta_s_ap0,param.theta_s_as0];
figure
subplot(2,1,1)
plot_ss_inputs(t,mu_syn_data.u_dist,u_eq)
title('\textbf{Control Inputs - \boldmath{$\mu$}-synthesis}','interpreter','latex','FontSize',12)
subplot(2,1,2)
plot_ss_inputs(t,hinf_data.u_dist,u_eq)
title('\textbf{Control Inputs - \boldmath{$h_{\infty}$}}','interpreter','latex','FontSize',12)

figure
subplot(2,1,1)
plot_uss_inputs(t,mu_syn_data.u_p_dist,u_eq,samples)
title('\textbf{Control Inputs - \boldmath{$\mu$}-synthesis - Parametric Uncertainty}',...
      'interpreter','latex','FontSize',12)
subplot(2,1,2)
plot_uss_inputs(t,mu_syn_data.u_p_app_dist,u_eq,samples)
title('\textbf{Control Inputs - \boldmath{$\mu$}-synthesis - Multiplicative Uncertainty}',...
      'interpreter','latex','FontSize',12)

figure
subplot(2,1,1)
plot_uss_inputs(t,hinf_data.u_p_dist,u_eq,samples)
title('\textbf{Control Inputs - \boldmath{$h_{\infty}$} - Parametric Uncertainty}',...
      'interpreter','latex','FontSize',12)
subplot(2,1,2)
plot_uss_inputs(t,hinf_data.u_p_app_dist,u_eq,samples)
title('\textbf{Control Inputs - \boldmath{$h_{\infty}$} - Multiplicative Uncertainty}',...
      'interpreter','latex','FontSize',12)

figure
subplot(2,1,1)
plot_uss_inputs(t,mu_syn_data.u_p_dist,u_eq,samples)
title('\textbf{Control Inputs - \boldmath{$\mu$}-synthesis - Parametric Uncertainty}',...
      'interpreter','latex','FontSize',12)
subplot(2,1,2)
plot_uss_inputs(t,hinf_data.u_p_dist,u_eq,samples)
title('\textbf{Control Inputs - \boldmath{$h_{\infty}$} - Parametric Uncertainty}',...
      'interpreter','latex','FontSize',12)