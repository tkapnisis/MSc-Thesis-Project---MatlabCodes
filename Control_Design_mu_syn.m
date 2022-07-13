% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft


clc
clear all
close all

addpath('Plotting Functions')
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

figure
pzplot(G,'b');
hold on
grid on
set(findall(gcf,'Type','line'),'MarkerSize',15)
pzplot(Gp_out_mult,'r')
legend('Nominal plant G(s)','Simplified Perturbed plant Gp_app(s)','Location','best','FontSize',11)
%}
%% Nyquist
%{
res = usample(Gp_app,1);
figure
w = logspace(-1,1,20);
nyquist(res);
%}
%% Bodeplot of the open-loops of nominal and perturbed plant
%{
opts_bode = bodeoptions;
opts_bode.MagScale = 'log';
opts_bode.MagUnits = 'abs';
opts_bode.InputLabels.Interpreter = 'none';
opts_bode.InputLabels.FontSize = 10;
opts_bode.OutputLabels.FontSize = 10;
opts_bode.XLabel.FontSize = 11;
opts_bode.YLabel.FontSize = 11;
opts_bode.TickLabel.FontSize = 10;
opts_bode.Title.FontSize = 12;
% opts_bode.XLimMode = 'manual';
% opts_bode.Xlim = [1e-3 1e2];
opts_bode.PhaseVisible = 'off';
opts_bode.Grid = 'on';
%%
figure
bodeplot(Gp_app,G,opts_bode)
set(findall(gcf,'Type','line'),'LineWidth',1.2)
legend('Perturbed plant Gp(s)','Nominal plant G(s)','Location','best','FontSize',11)
title('Gp_app')

figure
bodeplot(Gp_out_mult,G,opts_bode)
set(findall(gcf,'Type','line'),'LineWidth',1.2)
legend('Approximated Perturbed plant Gp_app(s)','Nominal plant G(s)','Location','best','FontSize',11)
title('Gp_cor')
%%
figure
bodeplot(Gp_app,G,opts_bode)
set(findall(gcf,'Type','line'),'LineWidth',1.2)
legend('Approximated Perturbed plant Gp_app(s)','Nominal plant G(s)','Location','best','FontSize',11)
title('Gp')
%}
%% Singular values and gamma analysis
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
sigma(Gp_app,G,opts_sigma);
legend('\boldmath{$\sigma(Gp_app)$}','\boldmath{$\sigma(G)$}','interpreter','latex','FontSize',15)
%%
figure
sigma(Gp,G,opts_sigma);
legend('\boldmath{$\sigma(Gp)$}','\boldmath{$\sigma(G)$}','interpreter','latex','FontSize',15)
%}
%{
% RGA
omega1 = 1e-2;
Gf1 = freqresp(G,omega1);
RGAw_1(:,:) = Gf1.*inv(Gf1)';

omega2 = 1*2*pi;
Gf2 = freqresp(G,omega2);
RGAw_2(:,:) = Gf2.*inv(Gf2)';

[U1,S1,V1]=svd(Gf1);
sv1=diag(S1);
gamma1=sv1(1)/sv1(2);

[U2,S2,V2]=svd(Gf2);
sv2=diag(S2);
gamma2=sv2(1)/sv2(2);

[sv,wout] = sigma(Gp,{1e-3 1e2});
figure
loglog(wout(:,1),sv(1,:))
hold on
loglog(wout(:,1),sv(3,:))
YScale = 'log';
legend('$\bar{\sigma}(G)$','$\underline{\sigma}(G)$',...
        'interpreter','latex','FontSize',15)
grid on
xlabel('Frequency [rad/s]')
ylabel('Singular Values (abs)')
title('Singular Values')

% for i=1:size(sv,2)
%     gamma(i) = sv(1,i)/sv(3,i);
% end    
% figure 
% plot(gamma)
%}
%% Scaling matrices
%{
De = diag([0.1,0.1,0.1]);
Du = diag([0.5,0.5,0.5]);
Dr = diag([0.1,0.1,0.1]);
Dd = diag([0.05,0.1,0.05,0.1,0.05,0.1]);

G_sc = inv(De)*G*Du;
Gd_sc = inv(De)*Gd*Dd;
R = inv(De)*Dr;
%}
%% Mixed-sensitivity Hinf controller Design
%% Define the Weighting Functions for the Hinf controller
[Wp,Wu,Wd,Wr,Gact,Gact_p,Wact] = Weights_Design();

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
[mu_syn.K,CL_mu,info_mu] = musyn(P_Delta,nmeas,ncont);%,mu_opts); 
timerun = toc;
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
select = 1;

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
dt = 0.05; % sampling time
tend = 30; % duration of simulation in seconds
t = 0:dt:tend;

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

figure
lsim(mu_syn.T_p,'b--',ref,t)
hold on
lsim(hinf.T_p,'r-.',ref,t)
title('Response of the Closed-Loop System - Parametric Uncertainty')
legend('\boldmath{$\mu$} \textbf{-synthesis controller}',...
       '\boldmath{$h_{\infty}$} \textbf{ controller}','interpreter','latex')
grid on

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
samples = 10;


mu_syn.y_p_app_dist = lsim_uss(mu_syn.S_p_app*Gd_p_app,dw,t,samples);
mu_syn.y_p_dist = lsim_uss(mu_syn.S_p*Gd_p,dw,t,samples);
mu_syn.y_dist = lsim(mu_syn.S*Gd,dw,t);

hinf.y_p_app_dist = lsim_uss(hinf.S_p_app*Gd_p_app,dw,t,samples);
hinf.y_p_dist = lsim_uss(hinf.S_p*Gd_p,dw,t,samples);
hinf.y_dist = lsim(hinf.S*Gd,dw,t);

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

[y,~,x] = lsim(S_*Gd_,dw,t);

plot_ss_states(t,y,param.z_n0)

u_eq = [param.theta_s_f0,param.theta_s_ap0,param.theta_s_as0];
u_in = lsim(-mu_syn.K*mu_syn.S*Gd_,dw,t);
plot_ss_inputs(t,u_in,u_eq)

u_in = lsim_uss(-mu_syn.K*mu_syn.S_p_app*Gd_p_app,dw,t,samples);
plot_uss_inputs(t,u_in,u_eq,samples)


%}
%% Order reduction of the controller
%{
figure
ncfmr(K_mu)
[~,info] = ncfmr(K_mu);
[marg,freq] = ncfmargin(G,K_mu);
K_mu_red = ncfmr(K_mu,20,info);
%}
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
% save('Controller_mu_synthesis.mat')
load('Controller_mu_synthesis.mat')