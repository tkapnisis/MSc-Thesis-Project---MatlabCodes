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
load('Controller_mu_syn.mat')
load('Controller_hinf.mat')

% load('Simulations_Results.mat')
% Nominal plant G(s)
% Disturbances transfer matrix Gd(s)
% Perturbed plant with uncertain parameters Gp(s)
% Perturbed disturbances transfer matrix with uncertain parameters Gd_p(s)

% Equilibrium input
u_eq = [param.delta_s_f0,param.delta_s_ap0,param.delta_s_as0];
%% Simulations for reference tracking with square signal on heave
% Time duration of simulations
dt = 0.02; % sampling time
tend = 4; % duration of simulation in seconds
t = 0:dt:tend;

step_z = 0.1;
ref = [max(0,min(step_z*t,step_z));0*ones(size(t));0*ones(size(t))];
% ref = [-step_z*square(2*pi/10*t);0*ones(size(t));0*ones(size(t))];
%% Nominal system
% Calculation of the outputs for the nominal system
mu_syn_data.y_ref = lsim(mu_syn_data.T,ref,t);
hinf_data.y_ref = lsim(hinf_data.T,ref,t);

figure
[fig1,fig3] = plot_ss_states(t,mu_syn_data.y_ref,ref,param.z_n0, 1,'-','#D95319','ref');
[fig2,~] = plot_ss_states(t,hinf_data.y_ref,ref,param.z_n0, 1,'-','#0072BD','ref');

subplot(3,1,1)
grid minor
subplot(3,1,2)
grid minor
subplot(3,1,3)
grid minor

legend([fig1,fig2,fig3],'\boldmath{$\mu$} \textbf{-synthesis controller}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller}','\textbf{Reference Signal}',...
       'interpreter','latex','Location','best','FontSize',10)

% Calculation of the control inputs for the nominal system
mu_syn_data.u_ref = lsim(mu_syn_data.K*mu_syn_data.S,ref,t);
hinf_data.u_ref = lsim(hinf_data.K*hinf_data.S,ref,t);

figure
subplot(2,1,1)
plot_ss_inputs(t,mu_syn_data.u_ref,u_eq)
title('\textbf{Control Inputs - \boldmath{$\mu$}-synthesis}','interpreter','latex','FontSize',12)
subplot(2,1,2)
plot_ss_inputs(t,hinf_data.u_ref,u_eq)
title('\textbf{Control Inputs - \boldmath{$h_{\infty}$}}','interpreter','latex','FontSize',12)

%% Perturbed system
% Number of samples for simulating the uncertain systems
samples = 10;

% Calculation of the outputs for the perturbed system
mu_syn_data.y_p_app_ref = lsim_uss(mu_syn_data.Tp_app,ref,t,samples);
mu_syn_data.y_p_ref = lsim_uss(mu_syn_data.Tp,ref,t,samples);
1
hinf_data.y_p_app_ref = lsim_uss(hinf_data.Tp_app,ref,t,samples);
hinf_data.y_p_ref = lsim_uss(hinf_data.Tp,ref,t,samples);
2
% Calculation of the control inputs for the perturbed system
mu_syn_data.u_p_app_ref = lsim_uss(mu_syn_data.K*mu_syn_data.Sp_app,ref,t,samples);
mu_syn_data.u_p_ref = lsim_uss(mu_syn_data.K*mu_syn_data.Sp,ref,t,samples);
3
hinf_data.u_p_app_ref = lsim_uss(hinf_data.K*hinf_data.Sp_app,ref,t,samples);
hinf_data.u_p_ref = lsim_uss(hinf_data.K*hinf_data.Sp,ref,t,samples);
%%
figure
fig1 = plot_uss_states(t,mu_syn_data.y_p_app_ref,samples,param.z_n0,0.5,'-.','blue');
fig2 = plot_uss_states(t,mu_syn_data.y_p_ref,samples,param.z_n0,1,'--','#77AC30');
[fig3,fig4] = plot_ss_states(t,mu_syn_data.y_ref,ref,param.z_n0,1,'-','red','ref');

legend([fig1,fig2,fig3,fig4],'\boldmath{$\mu$} \textbf{-synthesis-Multiplicative}',...
       '\boldmath{$\mu$} \textbf{-synthesis-Parametric}',...
       '\boldmath{$\mu$} \textbf{-synthesis-Nominal}','\textbf{Reference Signal}',...
       'interpreter','latex','FontSize',10)

figure
fig1 = plot_uss_states(t,hinf_data.y_p_app_ref,samples,param.z_n0,0.5,'-.','blue');
fig2 = plot_uss_states(t,hinf_data.y_p_ref,samples,param.z_n0,1,'--','#77AC30');
[fig3,fig4] = plot_ss_states(t,hinf_data.y_ref,ref,param.z_n0, 1,'-','red','ref');

legend([fig1,fig2,fig3,fig4],'\boldmath{$h_{\infty}$} \textbf{ controller-Multiplicative}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller-Parametric}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller-Nominal}','\textbf{Reference Signal}',...
       'interpreter','latex','FontSize',10)

figure
fig1 = plot_uss_states(t,mu_syn_data.y_p_ref,samples,param.z_n0,0.5,'-.','blue');
fig2 = plot_uss_states(t,hinf_data.y_p_ref,samples,param.z_n0,0.5,'--','red');
[~,fig3] = plot_ss_states(t,hinf_data.y_ref,ref,param.z_n0, 1,'--','red','ref');

legend([fig1,fig2,fig3],'\boldmath{$\mu$} \textbf{-synthesis}','\boldmath{$h_{\infty}$}',...
      '\textbf{Reference Signal}','interpreter','latex','FontSize',10)

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

% Time duration of simulations
dt = 0.02; % sampling time
tend = 20; % duration of simulation in seconds
t = 0:dt:tend;

%  Calculation of waves velocity profile for each hydrofoil

% Parameters of long-crested regular wave
wave_param.omega_0 = 0.8;  % Wave frequency [rad/s]
wave_param.lambda = 0.5;   % Wave length [m]
wave_param.zeta_0 = 0.15;  % Wave amplitude [m]
wave_param.beta = pi;      % Encounter angle (beta=0 for following waves) [rad] 

[dw,wave_param] = Wave_Model(t,wave_param,foil_loc,param);

%% Nominal system
% Calculation of the outputs for the nominal system
mu_syn_data.y_dist = lsim(mu_syn_data.S*Gd,dw,t);
hinf_data.y_dist = lsim(hinf_data.S*Gd,dw,t);

% Calculation of the control inputs for the nominal system
mu_syn_data.u_dist = lsim(-mu_syn_data.K*mu_syn_data.S*Gd,dw,t);
hinf_data.u_dist = lsim(-hinf_data.K*hinf_data.S*Gd,dw,t);

figure
[fig1,~] = plot_ss_states(t,mu_syn_data.y_dist,[],param.z_n0, 1,'-','#D95319','dist');
[fig2,~] = plot_ss_states(t,hinf_data.y_dist,[],param.z_n0, 1,'-','#0072BD','dist');

subplot(3,1,1)
grid minor
subplot(3,1,2)
grid minor
subplot(3,1,3)
grid minor

legend([fig1,fig2],'\boldmath{$\mu$} \textbf{-synthesis}',...
       '\boldmath{$h_{\infty}$} \textbf{-synthesis}','interpreter','latex','FontSize',10)

figure
subplot(2,1,1)
plot_ss_inputs(t,mu_syn_data.u_dist,u_eq)
title('\textbf{Control Inputs - \boldmath{$\mu$}-synthesis}','interpreter','latex','FontSize',12)
subplot(2,1,2)
plot_ss_inputs(t,hinf_data.u_dist,u_eq)
title('\textbf{Control Inputs - \boldmath{$h_{\infty}$}-synthesis}','interpreter','latex','FontSize',12)

%%
% Number of samples for simulating the uncertain systems
samples = 15;

% Calculation of the outputs
mu_syn_data.y_p_app_dist = lsim_uss(mu_syn_data.Sp_app*Gd_p_app,dw,t,samples);
mu_syn_data.y_p_dist = lsim_uss(mu_syn_data.Sp*Gd_p,dw,t,samples);

hinf_data.y_p_app_dist = lsim_uss(hinf_data.Sp_app*Gd_p_app,dw,t,samples);
hinf_data.y_p_dist = lsim_uss(hinf_data.Sp*Gd_p,dw,t,samples);

% Calculation of the control inputs
mu_syn_data.u_p_app_dist = lsim_uss(-mu_syn_data.K*mu_syn_data.Sp_app*Gd_p_app,dw,t,samples);
mu_syn_data.u_p_dist = lsim_uss(-mu_syn_data.K*mu_syn_data.Sp*Gd_p,dw,t,samples);

hinf_data.u_p_app_dist = lsim_uss(-hinf_data.K*hinf_data.Sp_app*Gd_p_app,dw,t,samples);
hinf_data.u_p_dist = lsim_uss(-hinf_data.K*hinf_data.Sp*Gd_p,dw,t,samples);


figure
fig1 = plot_uss_states(t,mu_syn_data.y_p_app_dist,samples,param.z_n0,0.5,'-','#0072BD');
fig2 = plot_uss_states(t,mu_syn_data.y_p_dist,samples,param.z_n0,0.5,'-','#77AC30');
[fig3,~] = plot_ss_states(t,mu_syn_data.y_dist,[],param.z_n0,1,'--','red','dist');

legend([fig1,fig2,fig3],'\boldmath{$\mu$} \textbf{-synthesis-Multiplicative}',...
       '\boldmath{$\mu$} \textbf{-synthesis-Parametric}',...
       '\boldmath{$\mu$} \textbf{-synthesis-Nominal}','interpreter','latex')

figure
fig1 = plot_uss_states(t,hinf_data.y_p_app_dist,samples,param.z_n0,0.5,'-','#0072BD');
fig2 = plot_uss_states(t,hinf_data.y_p_dist,samples,param.z_n0,0.5,'-','#77AC30');
fig3 = plot_ss_states(t,hinf_data.y_dist,[],param.z_n0, 1,'--','red','dist');

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

%% Save data
save('Data Files/Simulations_Results','hinf_data','mu_syn_data')