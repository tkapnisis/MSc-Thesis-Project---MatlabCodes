% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft


clc
clear all
close all

load('LTI_Perturbed_Plant.mat','G','Gd','Gp','Gd_p')
load('Parameters_Nominal.mat','param')
load('LTI_Nominal_Plant.mat','foil_loc')

% load('Multiplicative_Uncertainty.mat','Gp_app','Gd_p_app','W_O_G_ss','W_O_Gd_ss')
% Nominal plant G(s)
% Disturbances transfer matrix Gd(s)
% Perturbed plant with uncertain parameters Gp(s)
% Perturbed disturbances transfer matrix with uncertain parameters Gd_p(s)

nmeas = 3; % number of outputs 
ncont = 3; % number of inputs

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
%% Nyquist
%{
res = usample(Gp,1);
figure
w = logspace(-1,1,20);
nyquist(G);
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
%% Define the Weighting Functions for the Hinf controller
[Wp,Wu,Wd,Wr,Gact,Gact_p,Wact] = Weights_Design();

% Generalized Plant - Nominal
P = Generalized_Plant_Nominal(G,Gd,Wp,Wu,Wd,Wr,Gact);

% Hinf Controller synthesis - Nominal Plant
[K_hinf,~,gamma,~] = hinfsyn(P,nmeas,ncont);
gamma

loops = loopsens(G*Gact,K_hinf);
% loops = loopsens(G,K_hinf);
L = loops.Lo;
T = loops.To;
S = loops.So;

loops_p = loopsens(Gp,K_hinf);
Lp = loops_p.Lo;
Tp = loops_p.To;
Sp = loops_p.So;

select = 1;
switch select
    case 1
        T_ = T;
        S_ = S;
        G_ = G;
        Gd_ = Gd;
    case 2    
        T_ = Tp;
        S_ = Sp;
        G_ = Gp;
        Gd_ = Gd_p;
%     case 3    
%         T_ = Tp_app;
%         S_ = Sp_app;
%         G_ = Gp_app;
%         Gd_ = Gd_p_app;        
end

%{
figure
bodeplot(Wd(1,1),opts_bode)
%}
%% Singular Values of S, T, KS, GK, S*Gd, K*S*Gd
%
figure
sigma(S_,inv(Wp),opts_sigma);
legend('\boldmath{$\sigma(S)$}','interpreter','latex','FontSize',15)

figure
sigma(T_,opts_sigma);
legend('\boldmath{$\sigma(T)$}','interpreter','latex','FontSize',15)

figure
sigma(K_hinf*S_,inv(Wu),opts_sigma);
legend('\boldmath{$\sigma(KS)$}','interpreter','latex','FontSize',15)

figure
sigma(G_*K_hinf,opts_sigma);
legend('\boldmath{$\sigma(GK)$}','interpreter','latex','FontSize',15)

figure
sigma(S_*Gd_,opts_sigma);
legend('\boldmath{$\sigma(SGd)$}','interpreter','latex','FontSize',15)

figure
sigma(K_hinf*S_*Gd_,opts_sigma);
legend('\boldmath{$\sigma(KSGd)$}','interpreter','latex','FontSize',15)

figure
sigma(K_hinf,opts_sigma);
legend('\boldmath{$\sigma(K)$}','interpreter','latex','FontSize',15)
%}

%% Simulation of the closed loop system with the Hinf controller

figure
% step(usample(Tp,20),3);
% hold on
step(T,3)
% legend('Perturbed Plant', 'Nominal Plant')
title('Step Response with Hinf Controller')
grid minor
%%
figure
step(usample(Tp_app,20),3);
hold on
step(T,'r-')
legend('Approximated Perturbed Plant', 'Nominal Plant')
title('Step Response with Hinf Controller')
grid minor
%%
dt = 0.05; % sampling time
tend = 30; % duration of simulation in seconds
t = 0:dt:tend;

% ref = [0.5*sin(t);0*ones(size(t));0*ones(size(t))];
ref = [-0.05*square(0.5*t);0*ones(size(t));0*ones(size(t))];

x0 = [0, 0, 0, 0, 0, 0];
[y,~,~] = lsim(T,ref,t);

% figure
% lsim(Tp_app,ref,t);
% legend('Approximated Perturbed Plant')
%

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

% figure
% lsim(K*Sp,ref,t);

eq_input = [param.theta_s_f0,param.theta_s_ap0,param.theta_s_as0];
inp_val = lsim(K_hinf*S,ref,t) + eq_input;
% inp_val = lsim(K,ref'-y,t);

figure
plot(t,rad2deg(inp_val),'LineWidth', 1.5)
title('Servo motor angles - Control inputs')
grid minor
ylabel('\boldmath{$\theta_s$} \textbf{[deg]}','interpreter','latex')
xlabel('\textbf{time [s]}','interpreter','latex')
legend('Fore hydrofoil', 'Aft port hydrofoil', 'Aft starboard hydrofoil')

%% Simulation of the closed loop system with the Hinf controller and regular waves

%  Calculation of waves velocity profile for each hydrofoil

% Parameters of long-crested regular wave
wave_param.omega_0 = 1.5;   % Wave frequency [rad/s]
wave_param.lambda = 2;    % Wave length [m]
wave_param.zeta_0 = 0.1;  % Wave amplitude [m]
wave_param.beta = pi;     % Encounter angle (beta=0 for following waves) [rad] 

[dw,wave_param] = Wave_Model(t,wave_param,foil_loc,param);

% [y,t,x] = lsim(sys_CL,ref,t);
% figure
[y,~,x] = lsim(S*Gd,dw,t);
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

% inp_val = lsim(-K_hinf,y,t);
inp_val = lsim(-K_hinf*S*Gd,dw,t) + eq_input;

figure 
plot(t,rad2deg(inp_val),'LineWidth', 1.5)
title('Servo motor angles - Control inputs')
grid minor
ylabel('\boldmath{$\theta_s$} \textbf{[deg]}','interpreter','latex')
xlabel('\textbf{time [s]}','interpreter','latex')
legend('Fore hydrofoil', 'Aft port hydrofoil', 'Aft starboard hydrofoil')

%% Order-reduction of the controller
figure
ncfmr(K_hinf)
[~,info] = ncfmr(K_hinf);
% ncfmargin(P,C)
K_hinf_red = ncfmr(K_hinf,15,info);
% [marg,freq] = ncfmargin(G,Kunc)

%%
figure
subplot(3,1,1)
step(T(1,1))
% hold on
% step(Tp(1,1))
title('Heave Step Response')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$z_n$} \textbf{[m]}','interpreter','latex')
grid minor
legend('Original K','Reduced-order K')

subplot(3,1,2)
step(T(2,2))
% hold on
% step(Tp(2,2))
title('Roll')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\phi$} \textbf{[rad]}','interpreter','latex')
grid minor
subplot(3,1,3)
step(T(3,3))
% hold on
% step(Tp(3,3))
title('Pitch')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\theta$} \textbf{[rad]}','interpreter','latex')
grid minor

% figure
% step(sys_CL)
% Step_info = stepinfo(sys_CL);
%% Save data
% save('Hinf_Controller')
%% Discretization of the controller time step response
%
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