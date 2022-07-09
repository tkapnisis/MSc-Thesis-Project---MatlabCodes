% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft


clc
clear all
close all

load('LTI_Perturbed_Plant.mat','G','Gd','Gp','Gd_p')
load('Parameters_Nominal.mat','param')
load('LTI_Nominal_Plant.mat','foil_loc')

load('Multiplicative_Uncertainty.mat','W_I_G_ss','W_I_Gd_ss')
% Nominal plant G(s)
% Disturbances transfer matrix Gd(s)
% Perturbed plant with uncertain parameters Gp(s)
% Perturbed disturbances transfer matrix with uncertain parameters Gd_p(s)

nmeas = 3; % number of outputs 
ncont = 3; % number of inputs
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
bode_opts = bodeoptions;
bode_opts.MagScale = 'log';
bode_opts.MagUnits = 'abs';
bode_opts.InputLabels.Interpreter = 'none';
bode_opts.InputLabels.FontSize = 10;
bode_opts.OutputLabels.FontSize = 10;
bode_opts.XLabel.FontSize = 11;
bode_opts.YLabel.FontSize = 11;
bode_opts.TickLabel.FontSize = 10;
bode_opts.Title.FontSize = 12;
% bode_opts.XLimMode = 'manual';
% bode_opts.Xlim = [1e-3 1e2];
bode_opts.PhaseVisible = 'off';
bode_opts.Grid = 'on';
%%
figure
bodeplot(Gp_app,G,bode_opts)
set(findall(gcf,'Type','line'),'LineWidth',1.2)
legend('Perturbed plant Gp(s)','Nominal plant G(s)','Location','best','FontSize',11)
title('Gp_app')

figure
bodeplot(Gp_out_mult,G,bode_opts)
set(findall(gcf,'Type','line'),'LineWidth',1.2)
legend('Approximated Perturbed plant Gp_app(s)','Nominal plant G(s)','Location','best','FontSize',11)
title('Gp_cor')
%%
figure
bodeplot(Gp_app,G,bode_opts)
set(findall(gcf,'Type','line'),'LineWidth',1.2)
legend('Approximated Perturbed plant Gp_app(s)','Nominal plant G(s)','Location','best','FontSize',11)
title('Gp')
%}
%% Singular values and gamma analysis
%{
sigma_opts = sigmaoptions;
sigma_opts.MagScale = 'log';
sigma_opts.MagUnits = 'abs';
sigma_opts.InputLabels.FontSize = 10;
sigma_opts.OutputLabels.FontSize = 10;
sigma_opts.XLabel.FontSize = 11;
sigma_opts.YLabel.FontSize = 11;
sigma_opts.TickLabel.FontSize = 10;
sigma_opts.Title.FontSize = 12;
sigma_opts.Grid = 'on';

figure
sigma(Gp_app,G,sigma_opts);
legend('\boldmath{$\sigma(Gp_app)$}','\boldmath{$\sigma(G)$}','interpreter','latex','FontSize',15)
%%
figure
sigma(Gp,G,sigma_opts);
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
[Wp,Wu,Wd,Wr,Wact] = Hinf_Weights_Design();

% Generalized Plant - Nominal
P = Generalized_Plant_Nominal(G,Gd,Wp,Wu,Wd,Wr,Wact);
disp('----------- Hinf Controller Synthesis-Nominal Plant --------------')
% Hinf Controller synthesis - Nominal Plant
[K_hinf,~,gamma,~] = hinfsyn(P,nmeas,ncont);
gamma

loops = loopsens(G,K_hinf);

L = loops.Lo;
T = loops.To;
S = loops.So;

loops_hinf_p = loopsens(Gp,K_hinf);
L_hinf_p = loops_hinf_p.Lo;
T_hinf_p = loops_hinf_p.To;
S_hinf_p = loops_hinf_p.So;

%% Singular Values of S, T, KS, GK, S*Gd, K*S*Gd
%{
sigma_opts = sigmaoptions;
sigma_opts.MagScale = 'log';
sigma_opts.MagUnits = 'abs';
sigma_opts.InputLabels.FontSize = 10;
sigma_opts.OutputLabels.FontSize = 10;
sigma_opts.XLabel.FontSize = 11;
sigma_opts.YLabel.FontSize = 11;
sigma_opts.TickLabel.FontSize = 10;
sigma_opts.Title.FontSize = 12;
sigma_opts.Grid = 'on';

figure
sigma(S,inv(Wp),sigma_opts);
legend('\boldmath{$\sigma(S)$}','interpreter','latex','FontSize',15)

figure
sigma(T,sigma_opts);
legend('\boldmath{$\sigma(T)$}','interpreter','latex','FontSize',15)

figure
sigma(K*S,inv(Wu),sigma_opts);
legend('\boldmath{$\sigma(KS)$}','interpreter','latex','FontSize',15)

figure
sigma(G*K,sigma_opts);
legend('\boldmath{$\sigma(GK)$}','interpreter','latex','FontSize',15)

figure
sigma(S*Gd,sigma_opts);
legend('\boldmath{$\sigma(SGd)$}','interpreter','latex','FontSize',15)

figure
sigma(K*S*Gd,sigma_opts);
legend('\boldmath{$\sigma(KSGd)$}','interpreter','latex','FontSize',15)
%}
%% Generalized Plant - Perturbed
% Upper bound of the absolute value for the complex perturbations
bound_G = 0.3;
bound_Gd = 0.3;
[P_Delta,Paug,Gp_app,Gd_p_app] = Generalized_Plant_Perturbed...
                (G,Gd,bound_G,bound_Gd,W_I_G_ss,W_I_Gd_ss,Wp,Wu,Wd,Wr,Wact);
%% mu-synthesis of Hinf Controller - Perturbed Plant
disp('----------- mu-synthesis controller-Perturbed Plant --------------')
mu_opts = musynOptions('Display','full','FitOrder',[5 5]);
tic;
% mu_opts = musynOptions('Display','full','TargetPerf',1,'FullDG',false);%,'FrequencyGrid',[1e-1,1e1]);
[K_mu,CL_mu,info_mu] = musyn(P_Delta,nmeas,ncont);%,mu_opts); 
timerun = toc;

%%

loops_hinf_p_app = loopsens(Gp_app,K_hinf);
L_hinf_p_app = loops_hinf_p_app.Lo;
T_hinf_p_app = loops_hinf_p_app.To;
S_hinf_p_app = loops_hinf_p_app.So;

loops_mu_n = loopsens(G,K_mu);
L_mu_n = loops_mu_n.Lo;
T_mu_n = loops_mu_n.To;
S_mu_n = loops_mu_n.So;

loops_mu_app = loopsens(Gp_app,K_mu);
L_mu_p_app = loops_mu_app.Lo;
T_mu_p_app = loops_mu_app.To;
S_mu_p_app = loops_mu_app.So;

loops_mu_p = loopsens(Gp,K_mu);
L_mu_p = loops_mu_p.Lo;
T_mu_p = loops_mu_p.To;
S_mu_p = loops_mu_p.So;

num = 3;

switch num
    case 1
        T_ = T_mu_n;
        S_ = S_mu_n;
        G_ = G;
        Gd_ = Gd;
    case 2    
        T_ = T_mu_p;
        S_ = S_mu_p;
        G_ = Gp;
        Gd_ = Gd_p;
    case 3
        T_ = T_mu_p_app;
        S_ = S_mu_p_app;
        G_ = Gp_app;
        Gd_ = Gd_p_app;
end
%% Check the Robustness of the mu-synthesis controller

% Robust stability of uncertain system
[stabmarg,wcu,info] = robstab(T_)

% [stabmarg,destabunc,report,info] = robuststab(Tp)
% Generalized feedback interconnection of P block K block in order to
% obtain the N tranfer matrix
[M,~,BlkStruct] = lftdata(P_Delta);

Ndk=minreal(lft(M,K_mu));
omega=logspace(-4,3,400);

Nfdk=frd(Ndk,omega);

% Nominal stability
maxeigNdk=max(real(eig(Ndk))); 
%%
% Nominal performance
% blk=[9 6]; 
% [mubnds,~]=mussv(Nfdk(7:12,10:18),blk,'c');
blk=[9 6]; 
[mubnds,~]=mussv(Nfdk(4:9,4:12),blk,'c');
muNPdk=mubnds(:,1);
[muNPinfDK, muNPwDK]=norm(muNPdk,inf); 

% Robust stability
% blk = [3 3;6 3]; 
% [mubnds,~]=mussv(Nfdk(1:6,1:9),blk,'c');
blk = [3 3]; 
[mubnds,~]=mussv(Nfdk(1:3,1:3),blk,'c');
muRSdk=mubnds(:,1);
[muRSinfDK, muRSwDK]=norm(muRSdk,inf); 

% Robust performance
% blk=[3 3; 6 3; 9 6]; 
% [mubnds,~]=mussv(Nfdk(:,:),blk,'c');
blk=[3 3; 9 6]; 
[mubnds,~]=mussv(Nfdk(:,:),blk,'c');
muRPdk=mubnds(:,1);
[muRPinfDK, muRPwDK]=norm(muRPdk,inf); 
%% Frequency response of the structured singular values for NP, RS and RP
bode_opts = bodeoptions;
bode_opts.MagScale = 'log';
bode_opts.MagUnits = 'abs';
bode_opts.InputLabels.Interpreter = 'none';
bode_opts.InputLabels.FontSize = 10;
bode_opts.OutputLabels.FontSize = 10;
bode_opts.XLabel.FontSize = 11;
bode_opts.YLabel.FontSize = 11;
bode_opts.TickLabel.FontSize = 10;
bode_opts.Title.FontSize = 12;
bode_opts.PhaseVisible = 'off';
bode_opts.Grid = 'on';

figure
hold on
bodeplot(muNPdk,bode_opts)
bodeplot(muRSdk,bode_opts)
bodeplot(muRPdk,bode_opts)
legend('μ_ΔP(N22(jω))-NP','μ_Δ(N11(jω))-RS','μ_Δhat(N(jω))-RP','Location','best')
title('Structured singular values - μ for NP, RS and RP (mu-synthesis)')

%%
sigma_opts = sigmaoptions;
sigma_opts.MagScale = 'log';
sigma_opts.MagUnits = 'abs';
sigma_opts.InputLabels.FontSize = 10;
sigma_opts.OutputLabels.FontSize = 10;
sigma_opts.XLabel.FontSize = 11;
sigma_opts.YLabel.FontSize = 11;
sigma_opts.TickLabel.FontSize = 10;
sigma_opts.Title.FontSize = 12;
sigma_opts.Grid = 'on';

figure
sigma(S_,inv(Wp),sigma_opts);
legend('\boldmath{$\sigma(S)$}','interpreter','latex','FontSize',15)

figure
sigma(T_,sigma_opts);
legend('\boldmath{$\sigma(T)$}','interpreter','latex','FontSize',15)

figure
sigma(K_mu*S_,inv(Wu),sigma_opts);
legend('\boldmath{$\sigma(KS)$}','interpreter','latex','FontSize',15)

figure
sigma(G_*K_mu,sigma_opts);
legend('\boldmath{$\sigma(GK)$}','interpreter','latex','FontSize',15)

figure
sigma(S_*Gd_,sigma_opts);
legend('\boldmath{$\sigma(SG_d)$}','interpreter','latex','FontSize',15)

figure
sigma(K_mu*S_*Gd_,sigma_opts);
legend('\boldmath{$\sigma(K S G_d)$}','interpreter','latex','FontSize',15)

%% Simulation of the closed loop system with the Hinf controller

figure
step(T_mu_p,5)
hold on
step(T)
title('Step Response of the Closed-Loop System - Parametric Uncertainty')
legend('mu-synthesis controller-Perturbed','Hinf Controller-Nominal')
grid on

figure
step(T_mu_p_app,5)
hold on
step(T)
title('Step Response of the Closed-Loop System - Multiplicative Input Uncertainty')
legend('mu-synthesis controller-Perturbed','Hinf Controller-Nominal')
grid on

figure
step(T_hinf_p,5)
hold on
step(T)
title('Step Response of the Closed-Loop System - Parametric Uncertainty')
legend('Hinf Controller-Perturbed','Hinf Controller-Nominal')
grid on

figure
step(T_hinf_p_app,5)
hold on
step(T)
title('Step Response of the Closed-Loop System - Multiplicative Input Uncertainty')
legend('Hinf Controller-Perturbed','Hinf Controller-Nominal')
grid on

%%
figure
step(T_mu_n)
hold on
step(T)
title('Step Response of the Closed-Loop System')
legend('mu-synthesis controller','Hinf Controller')
grid on
%% Simulation with step signal on heave
dt = 0.01; % sampling time
tend = 30; % duration of simulation in seconds
t = 0:dt:tend;

% ref = [0.5*sin(t);0*ones(size(t));0*ones(size(t))];
ref = [-0.05*square(0.5*t);0*ones(size(t));0*ones(size(t))];

x0 = [0, 0, 0, 0, 0, 0];
[y,~,~] = lsim(T_,ref,t);

figure
lsim(T_,ref,t);
%%
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

inp_val = lsim(Kunc*S_,ref,t);
% inp_val = lsim(K,ref'-y,t);

figure
plot(t,rad2deg(inp_val),'LineWidth', 1.5)
title('Servo motor angles - Control inputs')
grid minor
ylabel('\boldmath{$\theta_s$} \textbf{[deg]}','interpreter','latex')
xlabel('\textbf{time [s]}','interpreter','latex')
legend('Fore hydrofoil', 'Aft port hydrofoil', 'Aft starboard hydrofoil')

%% Simulation of the closed loop system with the Hinf controller
%{
%  Calculation of waves velocity profile for each hydrofoil

% Parameters of long-crested regular wave
wave_param.omega_0 = 1.5;   % Wave frequency [rad/s]
wave_param.lambda = 2;    % Wave length [m]
wave_param.zeta_0 = 0.1;  % Wave amplitude [m]
wave_param.beta = pi;     % Encounter angle (beta=0 for following waves) [rad] 

[dw,wave_param] = Wave_Model(t,wave_param,foil_loc,param);

[y,~,x] = lsim(Sp*Gd,dw,t);

% figure
% lsim(Sp*Gd,dw,t);

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

% inp_val = lsim(K,-y,t);
inp_val = lsim(-K_mu*Sp*Gd,dw,t);

figure 
plot(t,rad2deg(inp_val),'LineWidth', 1.5)
title('Servo motor angles - Control inputs')
grid minor
ylabel('\boldmath{$\theta_s$} \textbf{[deg]}','interpreter','latex')
xlabel('\textbf{time [s]}','interpreter','latex')
legend('Fore hydrofoil', 'Aft port hydrofoil', 'Aft starboard hydrofoil')
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
% save('Hinf_Controller_Design2')
% load('Hinf_Controller_Design.mat')