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

% Nominal plant G(s)
% Disturbances transfer matrix Gd(s)
% Perturbed plant with uncertain parameters Gp(s)
% Perturbed disturbances transfer matrix with uncertain parameters Gd_p(s)

nmeas = 3; % number of outputs 
ncont = 3; % number of inputs

run Bode_options.m
run Sigma_options.m

% Time duration of simulations
dt = 0.02; % sampling time
tend = 20; % duration of simulation in seconds
t = 0:dt:tend;

% Equilibrium input
u_eq = [param.theta_s_f0,param.theta_s_ap0,param.theta_s_as0];
%% Pole-zero map of the open-loops of nominal and perturbed plant
%{
% Pole-zero map of the open-loop G(s)
figure
pzplot(G,'b');
hold on
grid on
set(findall(gcf,'Type','line'),'MarkerSize',15)
pzplot(Gp,'r')
legend('Nominal plant G(s)','Perturbed plant Gp(s)','Location','best','FontSize',11)
% MIMO poles and zeros of G
ps = pole(Gp);
zs = tzero(Gp);

%}
%% Bodeplot of the open-loops of nominal and perturbed plant
%{
figure
bodeplot(Gp,G,opts_bode)
set(findall(gcf,'Type','line'),'LineWidth',1.2)
legend('Perturbed plant Gp(s)','Nominal plant G(s)','Location','best','FontSize',11)
%}
%% Singular values and gamma analysis
%{
figure
sigmaplot(G,opts_sigma)
title('Plant Singular Values');
%}
%% Define the Weighting Functions for the Hinf controller
[Wp,Wu,Wd,Wi,Wref,Gact,Gact_p] = Design_Weights();

% Generalized Plant - Nominal
P = Generalized_Plant_Nominal(G,Gd,Wp,Wu,Wd,Wi,Wref,Gact);

% Hinf Controller synthesis - Nominal Plant
[hinf_data.K,~,gamma,~] = hinfsyn(P,nmeas,ncont);
gamma

hinf_data.loops = loopsens(G*Gact,hinf_data.K);
hinf_data.L = hinf_data.loops.Lo;
hinf_data.T = hinf_data.loops.To;
hinf_data.S = hinf_data.loops.So;

hinf_data.loops_p = loopsens(Gp*Gact_p,hinf_data.K);
hinf_data.Lp = hinf_data.loops_p.Lo;
hinf_data.Tp = hinf_data.loops_p.To;
hinf_data.Sp = hinf_data.loops_p.So;

select = 1;
switch select
    case 1
        T_ = hinf_data.T;
        S_ = hinf_data.S;
        G_ = G;
        Gd_ = Gd;
    case 2    
        T_ = hinf_data.Tp;
        S_ = hinf_data.Sp;
        G_ = Gp;
        Gd_ = Gd_p;   
end

%% Singular Values of S, T, KS, GK, S*Gd, K*S*Gd
%
figure
sigma(S_,inv(Wp),opts_sigma);
legend('\boldmath{$\sigma(S)$}','\boldmath{$W_p^{-1}$}','interpreter','latex','FontSize',15)

figure
sigma(T_,opts_sigma);
legend('\boldmath{$\sigma(T)$}','interpreter','latex','FontSize',15)

figure
sigma(hinf_data.K*S_,inv(Wu),opts_sigma);
legend('\boldmath{$\sigma(KS)$}','\boldmath{$W_u^{-1}$}','interpreter','latex','FontSize',15)

figure
sigma(G_*hinf_data.K,opts_sigma);
legend('\boldmath{$\sigma(GK)$}','interpreter','latex','FontSize',15)

figure
sigma(S_*Gd_,opts_sigma);
legend('\boldmath{$\sigma(SGd)$}','interpreter','latex','FontSize',15)

figure
sigma(hinf_data.K*S_*Gd_,opts_sigma);
legend('\boldmath{$\sigma(KSGd)$}','interpreter','latex','FontSize',15)

figure
sigma(hinf_data.K,opts_sigma);
legend('\boldmath{$\sigma(K)$}','interpreter','latex','FontSize',15)
%}

%% Simulation of the closed loop system with the Hinf controller

figure
step(hinf_data.T)
title('Step Response with Hinf Controller')
grid minor
%%
ref = [-0.05*square(2*pi/10*t);0*ones(size(t));0*ones(size(t))];
[y,~,~] = lsim(hinf_data.T,ref,t);

figure
plot_ss_states(t,y,ref,param.z_n0,1,'-','blue','ref');

u_in = lsim(hinf_data.K*hinf_data.S,ref,t);

figure
plot_ss_inputs(t,u_in,u_eq)

%% Simulation of the closed loop system with the Hinf controller and regular waves

%  Calculation of waves velocity profile for each hydrofoil
% Parameters of long-crested regular wave
wave_param.omega_0 = 1.5;   % Wave frequency [rad/s]
wave_param.lambda = 2;    % Wave length [m]
wave_param.zeta_0 = 0.1;  % Wave amplitude [m]
wave_param.beta = pi;     % Encounter angle (beta=0 for following waves) [rad] 

[dw,wave_param] = Wave_Model(t,wave_param,foil_loc,param);

[y,~,x] = lsim(hinf_data.S*Gd,dw,t);

figure
plot_ss_states(t,y,[],param.z_n0,1.5,'-','#0072BD','dist');

u_in = lsim(-hinf_data.K*hinf_data.S*Gd,dw,t);

figure
plot_ss_inputs(t,u_in,u_eq)

%% Save data
save('Data Files/Controller_hinf','hinf_data')
%% Discretization of the controller time step response
%{
dt = 1/100;
K_hinf_DT = c2d(K_hinf,dt,'tustin');

%{
Gp_DT=c2d(Gp,h,'tustin');
Sum = sumblk('e = r - y',3);
sys_CL_DT = connect(Gp_DT,K_DT,Sum,'r','y');

figure
step(sys_CL_DT)
hold on
step(sys_CL)
grid on
title('Step response - Reference tracking with PD controller')
%}
%}