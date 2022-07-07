% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft


clc
clear all
close all

load('LTI_Perturbed_Plant.mat','G','Gd','Gp','Gd_p')
load('Parameters_Nominal.mat','param')
load('LTI_Nominal_Plant.mat','foil_loc')

% Nominal plant G(s)
% Disturbances transfer matrix Gd(s)
% Perturbed plant with uncertain parameters Gp(s)
% Perturbed disturbances transfer matrix with uncertain parameters Gd_p(s)

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
bode_opts.XLimMode = 'manual';
bode_opts.Xlim = [1e-3 1e2];
bode_opts.PhaseVisible = 'off';
bode_opts.Grid = 'on';

figure
bodeplot(Gp,G,bode_opts)
set(findall(gcf,'Type','line'),'LineWidth',1.2)
legend('Perturbed plant Gp(s)','Nominal plant G(s)','Location','best','FontSize',11)
%}
%% Singular values and gamma analysis
%{
[sv,wout] = sigma(G,{1e-5 1e2});
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
%% Define the Weighting Functions for the Hinf controller
[Wp,Wu,Wd,Wr,Wact] = Hinf_Weights_Design();

% Generalized Plant - Nominal

Wp.u = 'v';
Wp.y = 'z1';
Wu.u = 'u';
Wu.y = 'z2';
Wd.u = 'd';
Wd.y = 'dw';
Wr.u = 'r';
Wr.y = 'rw';
% Wact.u = 'u';
% Wact.y = 'u_act';
% G.u = 'u_act';
% G.y = 'yG';
Gd.u = 'dw';
Gd.y = 'yGd';
G.u = 'u';
G.y = 'yG';

Sum_err = sumblk('v = rw - yG - yGd',3);
% Sum_err = sumblk('v = r - y',3);
% Sum_err = sumblk('v = r - yG',3);
inputs = {'r','d','u'};
% inputs = {'r','u'};
outputs = {'z1','z2','v'};
% P = connect(G,Gd,Wp,Wu,Wd,Wr,Wact,Sum_err,inputs,outputs);
P = connect(G,Gd,Wp,Wu,Wd,Wr,Sum_err,inputs,outputs);
% P = connect(G,Wp,Wu,Sum_err,inputs,outputs);
P = minreal(P);

% Hinf Controller synthesis - Nominal Plant
nmeas = 3; % number of outputs 
ncont = 3; % number of inputs

[K,CL,gamma,info] = hinfsyn(P,nmeas,ncont);
gamma

% loops = loopsens(G*Wact,K);
loops = loopsens(G,K);

L = loops.Lo;
T = loops.To;
S = loops.So;

loops_p = loopsens(Gp,K);
Lp = loops_p.Lo;
Tp = loops_p.To;
Sp = loops_p.So;

%% Singular Values of S, T, KS, GK, S*Gd, K*S*Gd
%
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

%% Simulation of the closed loop system with the Hinf controller

figure
step(usample(Tp,20));
hold on
step(T,'r-')
legend('Perturbed Plant', 'Nominal Plant')
title('Step Response with Hinf Controller')
grid minor

dt = 0.05; % sampling time
tend = 30; % duration of simulation in seconds
t = 0:dt:tend;

% ref = [0.5*sin(t);0*ones(size(t));0*ones(size(t))];
ref = [-0.05*square(0.5*t);0*ones(size(t));0*ones(size(t))];

x0 = [0, 0, 0, 0, 0, 0];
[y,~,~] = lsim(T,ref,t);

% figure
% lsim(T,ref,t);

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

inp_val = lsim(K*S,ref,t);
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

inp_val = lsim(-K,y,t);
% inp_val = lsim(-K*S*Gd,dw,t);

figure 
plot(t,rad2deg(inp_val),'LineWidth', 1.5)
title('Servo motor angles - Control inputs')
grid minor
ylabel('\boldmath{$\theta_s$} \textbf{[deg]}','interpreter','latex')
xlabel('\textbf{time [s]}','interpreter','latex')
legend('Fore hydrofoil', 'Aft port hydrofoil', 'Aft starboard hydrofoil')

%%
opts = bodeoptions;
opts.MagScale = 'log';
opts.MagUnits = 'abs';
% opts.MagLowerLimMode = 'manual';
% opts.MagLowerLim = 1e-5;
opts.PhaseVisible = 'off';
% opts.YLimMode = 'manual';
% opts.YLim = {[1e-5,1e1]};

figure
subplot(3,1,1)
bodeplot(S(1,1),1/Wp(1,1),opts);
legend('Sensitivity S11','1/Wp11','Location','best')
grid on
subplot(3,1,2)
bodeplot(S(2,2),1/Wp(2,2),opts)
legend('Sensitivity S22','1/Wp22','Location','best')
grid on
subplot(3,1,3)
bodeplot(S(3,3),1/Wp(3,3),opts)
legend('Sensitivity S33','1/W33','Location','best')
grid on
%%
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

figure 
bodeplot(K*S,inv(Wu),bode_opts)
% bodeplot(K*S,opts)
grid on
title('Bode plot of controller K*S versus its weight 1/Wu')
legend('K*S','1/Wu')
%%
figure 
bodeplot(K,opts)
grid on
title('Bode plot of controller K(2,1)*S versus its weight 1/Wu12')
legend('K*S','1/Wu')
%%
% Bandwidth

[sv,wout] = sigma(S);
[Max_sigma,Max_sigma_l] = max(sv(:));
[I_row, I_col] = ind2sub(size(sv),Max_sigma_l);
w_B = wout(I_col)
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
% save('Hinf_Controller_Design2')

% load('Hinf_Controller_Design.mat')
%%
figure
ncfmr(Kunc)
[~,info] = ncfmr(Kunc);
% ncfmargin(P,C)
%%
Kunc_red = ncfmr(Kunc,20,info);
%%
% [marg,freq] = ncfmargin(G,Kunc)
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