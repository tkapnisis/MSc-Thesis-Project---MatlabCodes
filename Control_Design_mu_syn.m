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
load('Multiplicative_Input_Uncertainty.mat','W_I_G_ss','W_I_Gd_ss')

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
tend = 30; % duration of simulation in seconds
t = 0:dt:tend;

%% Mixed-sensitivity Hinf controller Design
% Define the Weighting Functions for the Hinf controller
[Wp,Wu,Wd,Wr,Gact,Gact_p,Wact] = Design_Weights();

% Generalized Plant - Nominal
P = Generalized_Plant_Nominal(G,Gd,Wp,Wu,Wd,Wr,Gact);
disp('----------- Hinf Controller Synthesis-Nominal Plant --------------')
% Hinf Controller synthesis - Nominal Plant
[hinf.K,~,gamma,~] = hinfsyn(P,nmeas,ncont);
gamma

hinf.loops = loopsens(G*Gact,hinf.K);
hinf.L = hinf.loops.Lo;
hinf.T = hinf.loops.To;
hinf.S = hinf.loops.So;

hinf.loops_p = loopsens(Gp*Gact_p,hinf.K);
hinf.L_p = hinf.loops_p.Lo;
hinf.T_p = hinf.loops_p.To;
hinf.S_p = hinf.loops_p.So;

%% Singular Values of S, T, KS, GK, S*Gd, K*S*Gd
%{
opts_sigma = sigmaoptions;
opts_sigma.MagScale = 'log';
opts_sigma.MagUnits = 'abs';
opts_sigma.InputLabels.FontSize = 10;
opts_sigma.OutputLabels.FontSize = 10;
opts_sigma.XLabel.FontSize = 11;
opts_sigma.YLabel.FontSize = 11;
opts_sigma.TickLabel.FontSize = 10;
opts_sigma.Title.FontSize = 12;
opts_sigma.Grid = 'on';

figure
sigma(S,inv(Wp),opts_sigma);
legend('\boldmath{$\sigma(S)$}','interpreter','latex','FontSize',15)

figure
sigma(T,opts_sigma);
legend('\boldmath{$\sigma(T)$}','interpreter','latex','FontSize',15)

figure
sigma(K*S,inv(Wu),opts_sigma);
legend('\boldmath{$\sigma(KS)$}','interpreter','latex','FontSize',15)

figure
sigma(G*K,opts_sigma);
legend('\boldmath{$\sigma(GK)$}','interpreter','latex','FontSize',15)

figure
sigma(S*Gd,opts_sigma);
legend('\boldmath{$\sigma(SGd)$}','interpreter','latex','FontSize',15)

figure
sigma(K*S*Gd,opts_sigma);
legend('\boldmath{$\sigma(KSGd)$}','interpreter','latex','FontSize',15)
%}
%% Generalized Plant - Perturbed
% Upper bound of the absolute value for the complex perturbations
bound_G = 0.2;
bound_Gd = 0.2;
[P_Delta,P_aug,Gp_app,Gd_p_app] = Generalized_Plant_Perturbed...
                (G,Gd,bound_G,bound_Gd,W_I_G_ss,W_I_Gd_ss,Wp,Wu,Wd,Wr,Wact);
%% mu-synthesis of Hinf Controller - Perturbed Plant
disp('----------- mu-synthesis controller-Perturbed Plant --------------')
mu_opts = musynOptions('FitOrder',[5 5],'MaxIter',20);
tic;
% mu_opts = musynOptions('Display','full','TargetPerf',1,'FullDG',false);%,'FrequencyGrid',[1e-1,1e1]);
[mu_syn.K_full,CL_mu,info_mu] = musyn(P_Delta,nmeas,ncont);%,mu_opts); 
timerun = toc;

%% Order reduction of the controller
%
figure
ncfmr(mu_syn.K_full)
fprintf('Give the required order of the controller for error less than 1e-3: \n')
pause
order = input("");
mu_syn.K = ncfmr(mu_syn.K_full,order);

mu_syn.loops_full = loopsens(G*Gact,mu_syn.K_full);
mu_syn.L_full = mu_syn.loops_full.Lo;
mu_syn.T_full = mu_syn.loops_full.To;
mu_syn.S_full = mu_syn.loops_full.So;

mu_syn.loops = loopsens(G*Gact,mu_syn.K);
mu_syn.L = mu_syn.loops.Lo;
mu_syn.T = mu_syn.loops.To;
mu_syn.S = mu_syn.loops.So;

% Simulations with square signal on heave for comparison of the reduced
% order controller

ref = [-0.05*square(0.5*t);0*ones(size(t));0*ones(size(t))];

mu_syn.y_ref = lsim(mu_syn.T,ref,t);
mu_syn.y_ref_full = lsim(mu_syn.T_full,ref,t);

figure('Name','Response of the Closed-Loop System in Regular Waves - Nominal');
fig1 = plot_ss_states(t,mu_syn.y_ref,param.z_n0, 1,'-','#D95319');
fig2 = plot_ss_states(t,mu_syn.y_ref_full,param.z_n0, 1.5,'--','#0072BD');

subplot(3,1,1)
grid minor
subplot(3,1,2)
grid minor
subplot(3,1,3)
grid minor

legend([fig1,fig2],'Reduced order - 40 states','Full order - 214 states')

%%
hinf.loops_p_app = loopsens(Gp_app,hinf.K);
hinf.L_p_app = hinf.loops_p_app.Lo;
hinf.T_p_app = hinf.loops_p_app.To;
hinf.S_p_app = hinf.loops_p_app.So;

mu_syn.loops = loopsens(G*Gact,mu_syn.K);
mu_syn.L = mu_syn.loops.Lo;
mu_syn.T = mu_syn.loops.To;
mu_syn.S = mu_syn.loops.So;

mu_syn.loops_p_app = loopsens(Gp_app,mu_syn.K);
mu_syn.L_p_app = mu_syn.loops_p_app.Lo;
mu_syn.T_p_app = mu_syn.loops_p_app.To;
mu_syn.S_p_app = mu_syn.loops_p_app.So;

mu_syn.loops_p = loopsens(Gp*Gact_p,mu_syn.K);
mu_syn.L_p = mu_syn.loops_p.Lo;
mu_syn.T_p = mu_syn.loops_p.To;
mu_syn.S_p = mu_syn.loops_p.So;

% Select which case you want to use to plot the singular values
select = 3;

switch select
    case 1  % Nominal system with mu-synthesis controller
        T_ = mu_syn.T;
        S_ = mu_syn.S;
        G_ = G;
        Gd_ = Gd;
    case 2  % Perturbed system with mu-synthesis controller for parametric uncertainties
        T_ = mu_syn.T_p;
        S_ = mu_syn.S_p;
        G_ = Gp;
        Gd_ = Gd_p;
    case 3  % Perturbed system with mu-synthesis controller for multiplicative input uncertainties
        T_ = mu_syn.T_p_app;
        S_ = mu_syn.S_p_app;
        G_ = Gp_app;
        Gd_ = Gd_p_app;
end
%% Check the Robustness of the mu-synthesis controller

% Robust stability of uncertain system
[stabmarg,wcu,info] = robstab(T_)

% [stabmarg,destabunc,report,info] = robuststab(Tp)
% Generalized feedback interconnection of P block K block in order to
% obtain the N tranfer matrix
[M,~,Blkstruct] = lftdata(P_Delta);
Blkstruct(6).Name = 'Delta_P';
Blkstruct(6).Size = [9,6];
Blkstruct(6).Type = 'ultidyn';
Blkstruct(6).Occurrences = 1;
Blkstruct(6).Simplify = 2;
Ndk=minreal(lft(M,mu_syn.K));
omega=logspace(-4,4,400);

Nfdk=frd(Ndk,omega);

% Nominal stability
maxeigNdk=max(real(eig(Ndk))); 

% Nominal performance
[mubnds,~]=mussv(Nfdk(10:15,13:21),Blkstruct(6));
muNPdk=mubnds(:,1);
[muNPinfDK, muNPwDK]=norm(muNPdk,inf); 

% Robust stability
[mubnds,~]=mussv(Nfdk(1:9,1:12),Blkstruct(1:5));
muRSdk=mubnds(:,1);
[muRSinfDK, muRSwDK]=norm(muRSdk,inf); 

% Robust performance
[mubnds,~]=mussv(Nfdk(:,:),Blkstruct);
muRPdk=mubnds(:,1);
[muRPinfDK, muRPwDK]=norm(muRPdk,inf); 
%% Frequency response of the structured singular values for NP, RS and RP

figure
hold on
bodeplot(muNPdk,opts_bode)
bodeplot(muRSdk,opts_bode)
bodeplot(muRPdk,opts_bode)
legend('μ_ΔP(N22(jω))-NP','μ_Δ(N11(jω))-RS','μ_Δhat(N(jω))-RP','Location','best')
title('Structured singular values - μ for NP, RS and RP (mu-synthesis)')

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
lsim(mu_syn.T_p_app,'b--',ref,t)
hold on
lsim(mu_syn.T_p,'r-.',ref,t)
lsim(mu_syn.T,'k-',ref,t)
title('Response of the Closed-Loop System')
legend('\boldmath{$\mu$} \textbf{-synthesis controller-Multiplicative}',...
       '\boldmath{$\mu$} \textbf{-synthesis controller-Parametric}',...
       '\boldmath{$\mu$} \textbf{-synthesis controller-Nominal}','interpreter','latex')
grid on

figure
lsim(hinf.T_p_app,'b--',ref,t)
hold on
lsim(hinf.T_p,'r-.',ref,t)
lsim(hinf.T,'k-',ref,t)
title('Response of the Closed-Loop System')
legend('\boldmath{$h_{\infty}$} \textbf{ controller-Multiplicative}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller-Parametric}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller-Nominal}','interpreter','latex')
grid on
%%
figure
lsim(mu_syn.T_p,'b--',ref,t)
hold on
lsim(hinf.T_p,'r-.',ref,t)
title('Response of the Closed-Loop System - Parametric Uncertainty')
legend('\boldmath{$\mu$} \textbf{-synthesis controller}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller}','interpreter','latex')
grid on
%%
figure
lsim(mu_syn.T,ref,t)
hold on
lsim(hinf.T,ref,t)
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

inp_val = lsim(mu_syn.K*S_,ref,t);
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
samples = 5;

% Calculation of the outputs
mu_syn.y_p_app_dist = lsim_uss(mu_syn.S_p_app*Gd_p_app,dw,t,samples);
mu_syn.y_p_dist = lsim_uss(mu_syn.S_p*Gd_p,dw,t,samples);
mu_syn.y_dist = lsim(mu_syn.S*Gd,dw,t);

hinf.y_p_app_dist = lsim_uss(hinf.S_p_app*Gd_p_app,dw,t,samples);
hinf.y_p_dist = lsim_uss(hinf.S_p*Gd_p,dw,t,samples);
hinf.y_dist = lsim(hinf.S*Gd,dw,t);

% Calculation of the control inputs
mu_syn.u_p_app_dist = lsim_uss(-mu_syn.K*mu_syn.S_p_app*Gd_p_app,dw,t,samples);
mu_syn.u_p_dist = lsim_uss(-mu_syn.K*mu_syn.S_p*Gd_p,dw,t,samples);
mu_syn.u_dist = lsim(-mu_syn.K*mu_syn.S*Gd,dw,t);

hinf.u_p_app_dist = lsim_uss(-hinf.K*hinf.S_p_app*Gd_p_app,dw,t,samples);
hinf.u_p_dist = lsim_uss(-hinf.K*hinf.S_p*Gd_p,dw,t,samples);
hinf.u_dist = lsim(-hinf.K*hinf.S*Gd,dw,t);

%%
figure('Name','Response of the Closed-Loop System in Regular Waves');
fig1 = plot_uss_states(t,mu_syn.y_p_app_dist,samples,param.z_n0,0.5,'-','#0072BD');
fig2 = plot_uss_states(t,mu_syn.y_p_dist,samples,param.z_n0,0.5,'-','#77AC30');
fig3 = plot_ss_states(t,mu_syn.y_dist,param.z_n0,1,'--','red');

legend([fig1,fig2,fig3],'\boldmath{$\mu$} \textbf{-synthesis-Multiplicative}',...
       '\boldmath{$\mu$} \textbf{-synthesis-Parametric}',...
       '\boldmath{$\mu$} \textbf{-synthesis-Nominal}','interpreter','latex')

figure('Name','Response of the Closed-Loop System in Regular Waves');
fig1 = plot_uss_states(t,hinf.y_p_app_dist,samples,param.z_n0,0.5,'-','#0072BD');
fig2 = plot_uss_states(t,hinf.y_p_dist,samples,param.z_n0,0.5,'-','#77AC30');
fig3 = plot_ss_states(t,hinf.y_dist,param.z_n0, 1,'--','red');

legend([fig1,fig2,fig3],'\boldmath{$h_{\infty}$} \textbf{ controller-Multiplicative}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller-Parametric}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller-Nominal}','interpreter','latex')


figure('Name','Response of the Closed-Loop System in Regular Waves - Parametric Uncertainty');
fig1 = plot_uss_states(t,mu_syn.y_p_dist,samples,param.z_n0,0.5,'-','blue');
fig2 = plot_uss_states(t,hinf.y_p_dist,samples,param.z_n0,0.5,'-','red');

subplot(3,1,1)
grid minor
subplot(3,1,2)
grid minor
subplot(3,1,3)
grid minor
legend([fig1,fig2],'\boldmath{$\mu$} \textbf{-synthesis}',...
       '\boldmath{$h_{\infty}$}','interpreter','latex')


figure('Name','Response of the Closed-Loop System in Regular Waves - Nominal');
fig1 = plot_ss_states(t,mu_syn.y_dist,param.z_n0, 1,'-','#D95319');
fig2 = plot_ss_states(t,hinf.y_dist,param.z_n0, 1,'-','#0072BD');

subplot(3,1,1)
grid minor
subplot(3,1,2)
grid minor
subplot(3,1,3)
grid minor

legend([fig1,fig2],'\boldmath{$\mu$} \textbf{-synthesis controller}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller}','interpreter','latex')

%%
u_eq = [param.theta_s_f0,param.theta_s_ap0,param.theta_s_as0];
figure
subplot(2,1,1)
plot_ss_inputs(t,mu_syn.u_dist,u_eq)
title('\textbf{Control Inputs - \boldmath{$\mu$}-synthesis}','interpreter','latex','FontSize',12)
subplot(2,1,2)
plot_ss_inputs(t,hinf.u_dist,u_eq)
title('\textbf{Control Inputs - \boldmath{$h_{\infty}$}}','interpreter','latex','FontSize',12)

figure
subplot(2,1,1)
plot_uss_inputs(t,mu_syn.u_p_dist,u_eq,samples)
title('\textbf{Control Inputs - \boldmath{$\mu$}-synthesis - Parametric Uncertainty}',...
      'interpreter','latex','FontSize',12)
subplot(2,1,2)
plot_uss_inputs(t,mu_syn.u_p_app_dist,u_eq,samples)
title('\textbf{Control Inputs - \boldmath{$\mu$}-synthesis - Multiplicative Uncertainty}',...
      'interpreter','latex','FontSize',12)

figure
subplot(2,1,1)
plot_uss_inputs(t,hinf.u_p_dist,u_eq,samples)
title('\textbf{Control Inputs - \boldmath{$h_{\infty}$} - Parametric Uncertainty}',...
      'interpreter','latex','FontSize',12)
subplot(2,1,2)
plot_uss_inputs(t,hinf.u_p_app_dist,u_eq,samples)
title('\textbf{Control Inputs - \boldmath{$h_{\infty}$} - Multiplicative Uncertainty}',...
      'interpreter','latex','FontSize',12)

figure
subplot(2,1,1)
plot_uss_inputs(t,mu_syn.u_p_dist,u_eq,samples)
title('\textbf{Control Inputs - \boldmath{$\mu$}-synthesis - Parametric Uncertainty}',...
      'interpreter','latex','FontSize',12)
subplot(2,1,2)
plot_uss_inputs(t,hinf.u_p_dist,u_eq,samples)
title('\textbf{Control Inputs - \boldmath{$h_{\infty}$} - Parametric Uncertainty}',...
      'interpreter','latex','FontSize',12)


%% Discrete time step response
%{
h = 0.05;
Gp_DT=c2d(Gp,h,'tustin');
K_DT = c2d(K,h,'tustin');

Sum = sumblk('e = r - y',3);
sys_CL_DT = connect(Gp_DT,K_DT,Sum,'r','y');

figure
step(sys_CL_DT)
hold on
step(sys_CL)
grid on
title('Step response - Reference tracking with PD controller')
%}
%% Save data
% save('Data Files/Controller_mu_synthesis.mat')
% load('Controller_mu_synthesis.mat')