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

mu_syn_data.y_ref = lsim(mu_syn_data.T,ref,t);
mu_syn_data.y_ref_full = lsim(mu_syn_data.T_full,ref,t);
%%
figure('Name','Response of the Closed-Loop System in Regular Waves - Nominal');
fig1 = plot_ss_states(t,mu_syn_data.y_ref,[],param.z_n0, 1,'-','#D95319','ref_no_leg');
fig2 = plot_ss_states(t,mu_syn_data.y_ref_full,[],param.z_n0, 1.5,'--','#0072BD','ref_no_leg');

subplot(3,1,1)
grid minor
subplot(3,1,2)
grid minor
subplot(3,1,3)
grid minor

mu_syn_data.K_full_size = size(mu_syn_data.K_full.A,1);
mu_syn_data.K_size = size(mu_syn_data.K.A,1);
legend([fig1,fig2],strcat('Reduced-order:', num2str(mu_syn_data.K_size),' states'),...
    strcat('Full-order:', num2str(mu_syn_data.K_full_size),' states'))

%%
hinf_data.loops_p_app = loopsens(Gp_app,hinf_data.K);
hinf_data.L_p_app = hinf_data.loops_p_app.Lo;
hinf_data.T_p_app = hinf_data.loops_p_app.To;
hinf_data.S_p_app = hinf_data.loops_p_app.So;

mu_syn_data.loops = loopsens(G*Gact,mu_syn_data.K);
mu_syn_data.L = mu_syn_data.loops.Lo;
mu_syn_data.T = mu_syn_data.loops.To;
mu_syn_data.S = mu_syn_data.loops.So;

mu_syn_data.loops_p_app = loopsens(Gp_app,mu_syn_data.K);
mu_syn_data.L_p_app = mu_syn_data.loops_p_app.Lo;
mu_syn_data.T_p_app = mu_syn_data.loops_p_app.To;
mu_syn_data.S_p_app = mu_syn_data.loops_p_app.So;

mu_syn_data.loops_p = loopsens(Gp*Gact_p,mu_syn_data.K);
mu_syn_data.L_p = mu_syn_data.loops_p.Lo;
mu_syn_data.T_p = mu_syn_data.loops_p.To;
mu_syn_data.S_p = mu_syn_data.loops_p.So;

% Select which case you want to use to plot the singular values
select = 3;

switch select
    case 1  % Nominal system with mu-synthesis controller
        T_ = mu_syn_data.T;
        S_ = mu_syn_data.S;
        G_ = G;
        Gd_ = Gd;
    case 2  % Perturbed system with mu-synthesis controller for parametric uncertainties
        T_ = mu_syn_data.T_p;
        S_ = mu_syn_data.S_p;
        G_ = Gp;
        Gd_ = Gd_p;
    case 3  % Perturbed system with mu-synthesis controller for multiplicative input uncertainties
        T_ = mu_syn_data.T_p_app;
        S_ = mu_syn_data.S_p_app;
        G_ = Gp_app;
        Gd_ = Gd_p_app;
end

%%
figure
sigma(S_,inv(Wp),opts_sigma);
legend('\boldmath{$\sigma(S)$}','interpreter','latex','FontSize',15)

figure
sigma(T_,opts_sigma);
legend('\boldmath{$\sigma(T)$}','interpreter','latex','FontSize',15)

figure
sigma(K_mu*S_,inv(Wu),opts_sigma);
legend('\boldmath{$\sigma(KS)$}','interpreter','latex','FontSize',15)

figure
sigma(G_*K_mu,opts_sigma);
legend('\boldmath{$\sigma(GK)$}','interpreter','latex','FontSize',15)

figure
sigma(S_*Gd_,opts_sigma);
legend('\boldmath{$\sigma(SG_d)$}','interpreter','latex','FontSize',15)

figure
sigma(K_mu*S_*Gd_,opts_sigma);
legend('\boldmath{$\sigma(K S G_d)$}','interpreter','latex','FontSize',15)

%% Simulations with square signal on heave
ref = [-0.05*square(0.5*t);0*ones(size(t));0*ones(size(t))];

figure
lsim(mu_syn_data.T_p_app,'b--',ref,t)
hold on
lsim(mu_syn_data.T_p,'r-.',ref,t)
lsim(mu_syn_data.T,'k-',ref,t)
title('Response of the Closed-Loop System')
legend('\boldmath{$\mu$} \textbf{-synthesis controller-Multiplicative}',...
       '\boldmath{$\mu$} \textbf{-synthesis controller-Parametric}',...
       '\boldmath{$\mu$} \textbf{-synthesis controller-Nominal}','interpreter','latex')
grid on

figure
lsim(hinf_data.T_p_app,'b--',ref,t)
hold on
lsim(hinf_data.T_p,'r-.',ref,t)
lsim(hinf_data.T,'k-',ref,t)
title('Response of the Closed-Loop System')
legend('\boldmath{$h_{\infty}$} \textbf{ controller-Multiplicative}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller-Parametric}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller-Nominal}','interpreter','latex')
grid on
%%
figure
lsim(mu_syn_data.T_p,'b--',ref,t)
hold on
lsim(hinf_data.T_p,'r-.',ref,t)
title('Response of the Closed-Loop System - Parametric Uncertainty')
legend('\boldmath{$\mu$} \textbf{-synthesis controller}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller}','interpreter','latex')
grid on
%%
figure
lsim(mu_syn_data.T,ref,t)
hold on
lsim(hinf_data.T,ref,t)
title('Response of the Closed-Loop System')
legend('mu-synthesis controller','Hinf Controller')
grid on

%%
[y,~,~] = lsim(T_,ref,t);
figure
subplot(3,1,1)
plot(t,y(:,1) + param.z_n0,'LineWidth',1.5)
title('Heave')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$z_n$} \textbf{[m]}','interpreter','latex')
grid minor
subplot(3,1,2)
plot(t,rad2deg(y(:,2)),'LineWidth',1.5)
title('Roll')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\phi$} \textbf{[deg]}','interpreter','latex')
grid minor
subplot(3,1,3)
plot(t,rad2deg(y(:,3)),'LineWidth',1.5)
title('Pitch')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\theta$} \textbf{[deg]}','interpreter','latex')
grid minor

inp_val = lsim(mu_syn_data.K*S_,ref,t);
% inp_val = lsim(K,ref'-y,t);

figure
plot(t,rad2deg(inp_val),'LineWidth', 1.5)
title('Servo motor angles - Control inputs')
grid minor
ylabel('\boldmath{$\theta_s$} \textbf{[deg]}','interpreter','latex')
xlabel('\textbf{time [s]}','interpreter','latex')
legend('Fore hydrofoil', 'Aft port hydrofoil', 'Aft starboard hydrofoil')

%% Simulation of the closed loop system with regular waves
%
%  Calculation of waves velocity profile for each hydrofoil

% Parameters of long-crested regular wave
wave_param.omega_0 = 1.5;   % Wave frequency [rad/s]
wave_param.lambda = 2;    % Wave length [m]
wave_param.zeta_0 = 0.1;  % Wave amplitude [m]
wave_param.beta = pi;     % Encounter angle (beta=0 for following waves) [rad] 

[dw,wave_param] = Wave_Model(t,wave_param,foil_loc,param);

% Number of samples for simulating the uncertain systems
samples = 10;

% Calculation of the outputs
mu_syn_data.y_p_app_dist = lsim_uss(mu_syn_data.S_p_app*Gd_p_app,dw,t,samples);
mu_syn_data.y_p_dist = lsim_uss(mu_syn_data.S_p*Gd_p,dw,t,samples);
mu_syn_data.y_dist = lsim(mu_syn_data.S*Gd,dw,t);

hinf_data.y_p_app_dist = lsim_uss(hinf_data.S_p_app*Gd_p_app,dw,t,samples);
hinf_data.y_p_dist = lsim_uss(hinf_data.S_p*Gd_p,dw,t,samples);
hinf_data.y_dist = lsim(hinf_data.S*Gd,dw,t);

% Calculation of the control inputs
mu_syn_data.u_p_app_dist = lsim_uss(-mu_syn_data.K*mu_syn_data.S_p_app*Gd_p_app,dw,t,samples);
mu_syn_data.u_p_dist = lsim_uss(-mu_syn_data.K*mu_syn_data.S_p*Gd_p,dw,t,samples);
mu_syn_data.u_dist = lsim(-mu_syn_data.K*mu_syn_data.S*Gd,dw,t);

hinf_data.u_p_app_dist = lsim_uss(-hinf_data.K*hinf_data.S_p_app*Gd_p_app,dw,t,samples);
hinf_data.u_p_dist = lsim_uss(-hinf_data.K*hinf_data.S_p*Gd_p,dw,t,samples);
hinf_data.u_dist = lsim(-hinf_data.K*hinf_data.S*Gd,dw,t);

figure('Name','Response of the Closed-Loop System in Regular Waves');
fig1 = plot_uss_states(t,mu_syn_data.y_p_app_dist,samples,param.z_n0,0.5,'-','#0072BD');
fig2 = plot_uss_states(t,mu_syn_data.y_p_dist,samples,param.z_n0,0.5,'-','#77AC30');
fig3 = plot_ss_states(t,mu_syn_data.y_dist,param.z_n0,1,'--','red');

legend([fig1,fig2,fig3],'\boldmath{$\mu$} \textbf{-synthesis-Multiplicative}',...
       '\boldmath{$\mu$} \textbf{-synthesis-Parametric}',...
       '\boldmath{$\mu$} \textbf{-synthesis-Nominal}','interpreter','latex')

figure('Name','Response of the Closed-Loop System in Regular Waves');
fig1 = plot_uss_states(t,hinf_data.y_p_app_dist,samples,param.z_n0,0.5,'-','#0072BD');
fig2 = plot_uss_states(t,hinf_data.y_p_dist,samples,param.z_n0,0.5,'-','#77AC30');
fig3 = plot_ss_states(t,hinf_data.y_dist,param.z_n0, 1,'--','red');

legend([fig1,fig2,fig3],'\boldmath{$h_{\infty}$} \textbf{ controller-Multiplicative}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller-Parametric}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller-Nominal}','interpreter','latex')


figure('Name','Response of the Closed-Loop System in Regular Waves - Parametric Uncertainty');
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


figure('Name','Response of the Closed-Loop System in Regular Waves - Nominal');
fig1 = plot_ss_states(t,mu_syn_data.y_dist,param.z_n0, 1,'-','#D95319');
fig2 = plot_ss_states(t,hinf_data.y_dist,param.z_n0, 1,'-','#0072BD');

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