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
%% Simulation of the closed loop system with regular waves

% Time duration of simulations
dt = 0.02; % sampling time
tend = 30; % duration of simulation in seconds
t = 0:dt:tend;

%  Calculation of waves velocity profile for each hydrofoil

% Parameters of long-crested regular wave
omega_0 = 0.4:0.2:2;  % Wave frequency [rad/s]
lambda = 0.2:0.2:2;   % Wave length [m]
zeta_0 = 0.01:0.01:0.1;  % Wave amplitude [m]
wave_param.beta = pi;      % Encounter angle (beta=0 for following waves) [rad] 

for i=1:length(omega_0)
    wave_param.omega_0 = omega_0(i);  % Wave frequency [rad/s]
    wave_param.lambda = 1;   % Wave length [m]
    wave_param.zeta_0 = 0.05;  % Wave amplitude [m]

    [dw,wave_param] = Wave_Model(t,wave_param,foil_loc,param);
    mu_syn_res.y_dist_omega(:,:,i) = lsim(mu_syn_data.S*Gd,dw,t);
    hinf_res.y_dist_omega(:,:,i) = lsim(hinf_data.S*Gd,dw,t);
    mu_syn_res.RMS_omega(:,i) = rms(mu_syn_res.y_dist_omega(:,:,i));
    hinf_res.RMS_omega(:,i) = rms(hinf_res.y_dist_omega(:,:,i));
end

for i=1:length(lambda)
    wave_param.omega_0 = 1;  % Wave frequency [rad/s]
    wave_param.lambda = lambda(i);   % Wave length [m]
    wave_param.zeta_0 = 0.05;  % Wave amplitude [m]

    [dw,wave_param] = Wave_Model(t,wave_param,foil_loc,param);
    mu_syn_res.y_dist_lambda(:,:,i) = lsim(mu_syn_data.S*Gd,dw,t);
    hinf_res.y_dist_lambda(:,:,i) = lsim(hinf_data.S*Gd,dw,t);
    mu_syn_res.RMS_lambda(:,i) = rms(mu_syn_res.y_dist_lambda(:,:,i));
    hinf_res.RMS_lambda(:,i) = rms(hinf_res.y_dist_lambda(:,:,i));
end

for i=1:length(zeta_0)
    wave_param.omega_0 = 1;  % Wave frequency [rad/s]
    wave_param.lambda = 1;   % Wave length [m]
    wave_param.zeta_0 = zeta_0(i);  % Wave amplitude [m]

    [dw,wave_param] = Wave_Model(t,wave_param,foil_loc,param);
    mu_syn_res.y_dist_zeta_0(:,:,i) = lsim(mu_syn_data.S*Gd,dw,t);
    hinf_res.y_dist_zeta_0(:,:,i) = lsim(hinf_data.S*Gd,dw,t);
    mu_syn_res.RMS_zeta_0(:,i) = rms(mu_syn_res.y_dist_zeta_0(:,:,i));
    hinf_res.RMS_zeta_0(:,i) = rms(hinf_res.y_dist_zeta_0(:,:,i));
end

%% omega_0
figure
subplot(3,1,1)
plot(omega_0,mu_syn_res.RMS_omega(1,:),'r-o')
hold on
plot(omega_0,hinf_res.RMS_omega(1,:),'b-o')
legend('\boldmath{$\mu$} \textbf{-synthesis controller}', '\boldmath{$h_{\infty}$} \textbf{ controller}',...
       'interpreter','latex','FontSize',10,'location','best')
grid minor
xlabel('Frequency (rad/s)')
ylabel('RMS Error (m)')
title('Heave','FontSize',12)

subplot(3,1,2)
plot(omega_0,rad2deg(mu_syn_res.RMS_omega(2,:)),'r-o')
hold on
plot(omega_0,rad2deg(hinf_res.RMS_omega(2,:)),'b-o')
% legend('\boldmath{$\mu$} \textbf{-synthesis controller}', '\boldmath{$h_{\infty}$} \textbf{ controller}',...
%        'interpreter','latex','FontSize',10,'location','best')
grid minor
xlabel('Frequency (rad/s)')
ylabel('RMS Error (deg)')
title('Roll','FontSize',12)

subplot(3,1,3)
plot(omega_0,rad2deg(mu_syn_res.RMS_omega(3,:)),'r-o')
hold on
plot(omega_0,rad2deg(hinf_res.RMS_omega(3,:)),'b-o')
% legend('\boldmath{$\mu$} \textbf{-synthesis controller}', '\boldmath{$h_{\infty}$} \textbf{ controller}',...
%        'interpreter','latex','FontSize',10,'location','best')
grid minor
xlabel('Frequency (rad/s)')
ylabel('RMS Error (deg)')
title('Pitch','FontSize',12)
% lambda
figure
subplot(3,1,1)
plot(lambda,mu_syn_res.RMS_lambda(1,:),'r-o')
hold on
plot(lambda,hinf_res.RMS_lambda(1,:),'b-o')
legend('\boldmath{$\mu$} \textbf{-synthesis controller}', '\boldmath{$h_{\infty}$} \textbf{ controller}',...
       'interpreter','latex','FontSize',10,'location','best')
grid minor
xlabel('Wave length (m)')
ylabel('RMS Error (m)')
title('Heave','FontSize',12)

subplot(3,1,2)
plot(lambda,rad2deg(mu_syn_res.RMS_lambda(2,:)),'r-o')
hold on
plot(lambda,rad2deg(hinf_res.RMS_lambda(2,:)),'b-o')
% legend('\boldmath{$\mu$} \textbf{-synthesis controller}', '\boldmath{$h_{\infty}$} \textbf{ controller}',...
%        'interpreter','latex','FontSize',10,'location','best')
grid minor
xlabel('Wave length (m)')
ylabel('RMS Error (deg)')
title('Roll','FontSize',12)

subplot(3,1,3)
plot(lambda,rad2deg(mu_syn_res.RMS_lambda(3,:)),'r-o')
hold on
plot(lambda,rad2deg(hinf_res.RMS_lambda(3,:)),'b-o')
% legend('\boldmath{$\mu$} \textbf{-synthesis controller}', '\boldmath{$h_{\infty}$} \textbf{ controller}',...
%        'interpreter','latex','FontSize',10,'location','best')
grid minor
xlabel('Wave length (m)')
ylabel('RMS Error (deg)')
title('Pitch','FontSize',12)

% zeta_0
figure
subplot(3,1,1)
plot(zeta_0,mu_syn_res.RMS_zeta_0(1,:),'r-o')
hold on
plot(zeta_0,hinf_res.RMS_zeta_0(1,:),'b-o')
legend('\boldmath{$\mu$} \textbf{-synthesis controller}', '\boldmath{$h_{\infty}$} \textbf{ controller}',...
       'interpreter','latex','FontSize',10,'location','best')
grid minor
xlabel('Wave amplitude (m)')
ylabel('RMS Error (m)')
title('Heave','FontSize',12)
xlim([min(zeta_0),max(zeta_0)])

subplot(3,1,2)
plot(zeta_0,rad2deg(mu_syn_res.RMS_zeta_0(2,:)),'r-o')
hold on
plot(zeta_0,rad2deg(hinf_res.RMS_zeta_0(2,:)),'b-o')
% legend('\boldmath{$\mu$} \textbf{-synthesis controller}', '\boldmath{$h_{\infty}$} \textbf{ controller}',...
%        'interpreter','latex','FontSize',10,'location','best')
grid minor
xlabel('Wave amplitude (m)')
ylabel('RMS Error (deg)')
title('Roll','FontSize',12)
xlim([min(zeta_0),max(zeta_0)])

subplot(3,1,3)
plot(zeta_0,rad2deg(mu_syn_res.RMS_zeta_0(3,:)),'r-o')
hold on
plot(zeta_0,rad2deg(hinf_res.RMS_zeta_0(3,:)),'b-o')
% legend('\boldmath{$\mu$} \textbf{-synthesis controller}', '\boldmath{$h_{\infty}$} \textbf{ controller}',...
%        'interpreter','latex','FontSize',10,'location','best')
grid minor
xlabel('Wave amplitude (m)')
ylabel('RMS Error (deg)')
title('Pitch','FontSize',12)
xlim([min(zeta_0),max(zeta_0)])
%% Nominal system
% Calculation of the outputs for the nominal system
mu_syn_res.y_dist = lsim(mu_syn_data.S*Gd,dw,t);
hinf_res.y_dist = lsim(hinf_data.S*Gd,dw,t);

figure
[fig1,~] = plot_ss_states(t,mu_syn_res.y_dist,[],param.z_n0, 1,'-','#D95319','dist');
[fig2,~] = plot_ss_states(t,hinf_res.y_dist,[],param.z_n0, 1,'-','#0072BD','dist');

subplot(3,1,1)
grid minor
subplot(3,1,2)
grid minor
subplot(3,1,3)
grid minor

legend([fig1,fig2],'\boldmath{$\mu$} \textbf{-synthesis}',...
       '\boldmath{$h_{\infty}$} \textbf{-synthesis}','interpreter','latex','FontSize',10)
