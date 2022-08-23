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

% load('Simulations_Results_RMS.mat')
% Equilibrium input
u_eq = [param.delta_s_f0,param.delta_s_ap0,param.delta_s_as0];

samples = 15;
%% Simulation of the closed loop system with regular waves

% Time duration of simulations
dt = 0.05; % sampling time
tend = 20; % duration of simulation in seconds
t = 0:dt:tend;

%  Calculation of waves velocity profile for each hydrofoil

% Parameters of long-crested regular wave
lambda = 1:1:5;
omega = sqrt(2*pi*param.g./lambda);  % Wave frequency [rad/s]
T = 2*pi./omega;
zeta_0 = 0.02:0.02:0.1;  % Wave amplitude [m]
beta = [0,pi];      % Encounter angle (beta=0 for following waves) [rad] 

% Parameters of long-crested regular wave
wave_param.omega = omega(3);  % Wave frequency [rad/s]
wave_param.lambda = lambda(3);   % Wave length [m]
wave_param.zeta_0 = zeta_0(3);  % Wave amplitude [m]
wave_param.beta = 0*pi;      % Encounter angle (beta=0 for following waves) [rad] 
[dw,wave_param] = Wave_Model(t,wave_param,foil_loc,param);

%%
% Nominal system
% Calculation of the outputs for the nominal system
mu_syn_res.y_dist = lsim(mu_syn_data.S*Gd,dw,t);
hinf_res.y_dist = lsim(hinf_data.S*Gd,dw,t);

% Calculation of the control inputs for the nominal system
mu_syn_res.u_dist = lsim(-mu_syn_data.K*mu_syn_data.S*Gd,dw,t);
hinf_res.u_dist = lsim(-hinf_data.K*hinf_data.S*Gd,dw,t);

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

figure
fig1 = plot_ss_inputs(t,mu_syn_res.u_dist,u_eq,1,'-','blue');
fig2 = plot_ss_inputs(t,hinf_res.u_dist,u_eq,1,'-','red');
title('Control Inputs - Servo Motor Angles','FontSize',12)
legend([fig1,fig2],'\boldmath{$\mu$} \textbf{-synthesis controller: Nominal}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller: Nominal}',...
       'interpreter','latex','Location','best','FontSize',10)
subplot(3,1,1)
grid minor
subplot(3,1,2)
grid minor
subplot(3,1,3)
grid minor

%% Simulatioms for different wave conditions
%
tic;
%%%%%%%%%%%%%%%%%%%%%%%%%%% Following sea %%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('----------- Following sea ----------')
for i=1:length(omega)
        wave_param.omega = omega(i); % Wave frequency [rad/s]
        wave_param.lambda = lambda(i);   % Wave length [m]
        wave_param.zeta_0 = zeta_0(3);   % Wave amplitude [m]
        wave_param.beta = beta(1);       % Encounter angle [rad]
        [dw,wave_param] = Wave_Model(t,wave_param,foil_loc,param);
        mu_syn_res.y_dist_omega_0(:,:,i,:) = lsim_uss(mu_syn_data.Sp*Gd_p,dw,t,samples);
        hinf_res.y_dist_omega_0(:,:,i,:) = lsim_uss(hinf_data.Sp*Gd_p,dw,t,samples);
        i
    for j=1:samples         
        mu_syn_res.RMS_omega_0(:,i,j) = rms(mu_syn_res.y_dist_omega_0(:,:,i,j));
        hinf_res.RMS_omega_0(:,i,j) = rms(hinf_res.y_dist_omega_0(:,:,i,j));
    end
end
disp('omega')

for i=1:length(zeta_0)
        wave_param.omega = omega(3); % Wave frequency [rad/s]
        wave_param.lambda = lambda(3);   % Wave length [m]
        wave_param.zeta_0 = zeta_0(i);   % Wave amplitude [m]
        wave_param.beta = beta(1);       % Encounter angle [rad]
        [dw,wave_param] = Wave_Model(t,wave_param,foil_loc,param);
        mu_syn_res.y_dist_zeta_0_0(:,:,i,:) = lsim_uss(mu_syn_data.Sp*Gd_p,dw,t,samples);
        hinf_res.y_dist_zeta_0_0(:,:,i,:) = lsim_uss(hinf_data.Sp*Gd_p,dw,t,samples);
        i
    for j=1:samples
        mu_syn_res.RMS_zeta_0_0(:,i,j) = rms(mu_syn_res.y_dist_zeta_0_0(:,:,i,j));
        hinf_res.RMS_zeta_0_0(:,i,j) = rms(hinf_res.y_dist_zeta_0_0(:,:,i,j));
    end
end
disp('zeta_0')
%%%%%%%%%%%%%%%%%%%%%%%%%%% Head sea %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('---------- Head sea -----------')
for i=1:length(omega)
        wave_param.omega = omega(i); % Wave frequency [rad/s]
        wave_param.lambda = lambda(i);   % Wave length [m]
        wave_param.zeta_0 = zeta_0(3);   % Wave amplitude [m]
        wave_param.beta = beta(2);       % Encounter angle [rad]
        [dw,wave_param] = Wave_Model(t,wave_param,foil_loc,param);
        mu_syn_res.y_dist_omega_pi(:,:,i,:) = lsim_uss(mu_syn_data.Sp*Gd_p,dw,t,samples);
        hinf_res.y_dist_omega_pi(:,:,i,:) = lsim_uss(hinf_data.Sp*Gd_p,dw,t,samples);
        i
    for j=1:samples         
        mu_syn_res.RMS_omega_pi(:,i,j) = rms(mu_syn_res.y_dist_omega_pi(:,:,i,j));
        hinf_res.RMS_omega_pi(:,i,j) = rms(hinf_res.y_dist_omega_pi(:,:,i,j));
    end
end
disp('omega')

for i=1:length(zeta_0)
        wave_param.omega = omega(3); % Wave frequency [rad/s]
        wave_param.lambda = lambda(3);   % Wave length [m]
        wave_param.zeta_0 = zeta_0(i);   % Wave amplitude [m]
        wave_param.beta = beta(2);       % Encounter angle [rad]
        [dw,wave_param] = Wave_Model(t,wave_param,foil_loc,param);
        mu_syn_res.y_dist_zeta_0_pi(:,:,i,:) = lsim_uss(mu_syn_data.Sp*Gd_p,dw,t,samples);
        hinf_res.y_dist_zeta_0_pi(:,:,i,:) = lsim_uss(hinf_data.Sp*Gd_p,dw,t,samples);
        i
    for j=1:samples
        mu_syn_res.RMS_zeta_0_pi(:,i,j) = rms(mu_syn_res.y_dist_zeta_0_pi(:,:,i,j));
        hinf_res.RMS_zeta_0_pi(:,i,j) = rms(hinf_res.y_dist_zeta_0_pi(:,:,i,j));
    end
end
disp('zeta_0')
time_elaps = toc;
%}
%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% omega %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
subplot(3,1,1)
hold on
for i=1:samples
    plot(omega,mu_syn_res.RMS_omega_0(1,:,i),'b--o')
    plot(omega,hinf_res.RMS_omega_0(1,:,i),'-.v','Color','#77AC30')
    plot(omega,mu_syn_res.RMS_omega_pi(1,:,i),'r--o')
    plot(omega,hinf_res.RMS_omega_pi(1,:,i),'-.v','Color','#EDB120')    
end    

legend('\boldmath{$\mu$} \textbf{-synthesis: Following sea}',...
       '\boldmath{$h_{\infty}$} \textbf{: Following sea}',...
       '\boldmath{$\mu$} \textbf{-synthesis: Head sea}',...
       '\boldmath{$h_{\infty}$} \textbf{: Head sea}',...
       'interpreter','latex','FontSize',10,'location','best')
grid minor
ax = gca;
ax.FontSize = 10; 
xlabel('Wave frequency (rad/s)','FontSize',12)
ylabel('\textbf{RMS } \boldmath{$z_n$}\textbf{ (m)}','interpreter','latex','FontSize',12)
title('Heave','FontSize',12)

subplot(3,1,2)
hold on
for i=1:samples
    plot(omega,rad2deg(mu_syn_res.RMS_omega_0(2,:,i)),'b--o')
    plot(omega,rad2deg(hinf_res.RMS_omega_0(2,:,i)),'-.v','Color','#77AC30')
    plot(omega,rad2deg(mu_syn_res.RMS_omega_pi(2,:,i)),'r--o')
    plot(omega,rad2deg(hinf_res.RMS_omega_pi(2,:,i)),'-.v','Color','#EDB120')
end    
grid minor
xlabel('Wave frequency (rad/s)','FontSize',12)
ylabel('\textbf{RMS } \boldmath{$\phi$}\textbf{ (m)}','interpreter','latex','FontSize',10')
title('Roll','FontSize',12)

subplot(3,1,3)
hold on
for i=1:samples
    plot(omega,rad2deg(mu_syn_res.RMS_omega_0(3,:,i)),'b--o')
    plot(omega,rad2deg(hinf_res.RMS_omega_0(3,:,i)),'-.v','Color','#77AC30')
    plot(omega,rad2deg(mu_syn_res.RMS_omega_pi(3,:,i)),'r--o')
    plot(omega,rad2deg(hinf_res.RMS_omega_pi(3,:,i)),'-.v','Color','#EDB120')    
end    
grid minor
ax = gca;
ax.FontSize = 10;
xlabel('Wave frequency (rad/s)','FontSize',12)
ylabel('\textbf{RMS } \boldmath{$\theta$}\textbf{ (m)}','interpreter','latex','FontSize',10')
title('Pitch','FontSize',12)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% zeta_0 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
subplot(3,1,1)
hold on
for i=1:samples
    plot(zeta_0,mu_syn_res.RMS_zeta_0_0(1,:,i),'b--o')
    plot(zeta_0,hinf_res.RMS_zeta_0_0(1,:,i),'-.v','Color','#77AC30')
    plot(zeta_0,mu_syn_res.RMS_zeta_0_pi(1,:,i),'r--o')
    plot(zeta_0,hinf_res.RMS_zeta_0_pi(1,:,i),'-.v','Color','#EDB120')    
end    

legend('\boldmath{$\mu$} \textbf{-synthesis: Following sea}',...
       '\boldmath{$h_{\infty}$} \textbf{: Following sea}',...
       '\boldmath{$\mu$} \textbf{-synthesis: Head sea}',...
       '\boldmath{$h_{\infty}$} \textbf{: Head sea}',...
       'interpreter','latex','FontSize',10,'location','best')
grid minor
ax = gca;
ax.FontSize = 10;
xlabel('Wave amplitude (m)','FontSize',12)
ylabel('\textbf{RMS } \boldmath{$z_n$}\textbf{ (m)}','interpreter','latex','FontSize',10')
title('Heave','FontSize',12)

subplot(3,1,2)
hold on
for i=1:samples
    plot(zeta_0,rad2deg(mu_syn_res.RMS_zeta_0_0(2,:,i)),'b--o')
    plot(zeta_0,rad2deg(hinf_res.RMS_zeta_0_0(2,:,i)),'-.v','Color','#77AC30')
    plot(zeta_0,rad2deg(mu_syn_res.RMS_zeta_0_pi(2,:,i)),'r--o')
    plot(zeta_0,rad2deg(hinf_res.RMS_zeta_0_pi(2,:,i)),'-.v','Color','#EDB120')    
end    
grid minor
xlabel('Wave amplitude (m)','FontSize',12)
ylabel('\textbf{RMS } \boldmath{$\phi$}\textbf{ (m)}','interpreter','latex','FontSize',10')
title('Roll','FontSize',12)

subplot(3,1,3)
hold on
for i=1:samples
    plot(zeta_0,rad2deg(mu_syn_res.RMS_zeta_0_0(3,:,i)),'b--o')
    plot(zeta_0,rad2deg(hinf_res.RMS_zeta_0_0(3,:,i)),'-.v','Color','#77AC30')
    plot(zeta_0,rad2deg(mu_syn_res.RMS_zeta_0_pi(3,:,i)),'r--o')
    plot(zeta_0,rad2deg(hinf_res.RMS_zeta_0_pi(3,:,i)),'-.v','Color','#EDB120')    
end    
grid minor
ax = gca;
ax.FontSize = 10;
xlabel('Wave amplitude (m)','FontSize',12)
ylabel('\textbf{RMS } \boldmath{$\theta$}\textbf{ (m)}','interpreter','latex','FontSize',10')
title('Pitch','FontSize',12)
%% Save data
save('Data Files/Simulations_Results_RMS','hinf_res','mu_syn_res','omega',...
     'lambda', 'zeta_0', 'beta')