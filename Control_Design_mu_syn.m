% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft


clc
clear all
close all

load('LTI_Perturbed_Plant.mat','G','Gd','Gp','Gd_p')
load('Parameters_Nominal.mat','param')
load('LTI_Nominal_Plant.mat','foil_loc')
% load('Additive_Uncertainty.mat','W_A','W_A_ss')
load('Multiplicative_Uncertainty.mat','W_I','W_I_ss','Gp_inp_mult')
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
bodeplot(Gp_cor,G,bode_opts)
set(findall(gcf,'Type','line'),'LineWidth',1.2)
legend('Approximated Perturbed plant Gp_app(s)','Nominal plant G(s)','Location','best','FontSize',11)
title('Gp')
%}
%% Singular values and gamma analysis
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

%%
%{
sigma_opts = sigmaoptions;
sigma_opts.MagScale = 'log';
sigma_opts.MagUnits = 'abs';
% sigma_opts.InputLabels.Interpreter = 'none';
sigma_opts.InputLabels.FontSize = 10;
sigma_opts.OutputLabels.FontSize = 10;
sigma_opts.XLabel.FontSize = 11;
sigma_opts.YLabel.FontSize = 11;
sigma_opts.TickLabel.FontSize = 10;
sigma_opts.Title.FontSize = 12;
% sigma_opts.XLimMode = 'manual';
% sigma_opts.Xlim = [1e-3 1e2];
% sigma_opts.PhaseVisible = 'off';
sigma_opts.Grid = 'on';

figure
sigma(S,sigma_opts);

figure
sigma(T,sigma_opts);


figure
sigma(K*S,sigma_opts);
%}
%% Generalized Plant - Perturbed

% Defining the complex scalar uncertainties for each channel of perturbed plant
bound = 0.1;
Delta_A = blkdiag(  ultidyn('d11',[1,1],'Bound',bound),...
                    ultidyn('d21',[1,1],'Bound',bound),...
                    ultidyn('d31',[1,1],'Bound',bound),...
                    ultidyn('d12',[1,1],'Bound',bound),...
                    ultidyn('d22',[1,1],'Bound',bound),...
                    ultidyn('d32',[1,1],'Bound',bound),...
                    ultidyn('d13',[1,1],'Bound',bound),...
                    ultidyn('d23',[1,1],'Bound',bound),...
                    ultidyn('d33',[1,1],'Bound',bound));

W_A_ref = [W_A.w11,        0,        0, W_A.w12,        0,        0, W_A.w13,        0,        0;...
                  0, W_A.w21,        0,        0, W_A.w22,        0,        0, W_A.w23,       0;...
                  0,        0, W_A.w31,        0,        0, W_A.w32,        0,        0, W_A.w33];

% [M,Delta_A_diag,BlkStruct] = lftdata(Gp_add);

aux_mat = ss([ ones(3,1), zeros(3,1), zeros(3,1);...
              zeros(3,1),  ones(3,1), zeros(3,1);...
              zeros(3,1), zeros(3,1), ones(3,1)]);
%%
[Wp,Wu,Wd,Wr,Wact] = Hinf_Weights_Design();

Wp.u = 'v';
Wp.y = 'z1';
Wu.u = 'u';
Wu.y = 'z2';
Wd.u = 'd';
Wd.y = 'dw';
Wr.u = 'r';
Wr.y = 'rw';
% Wact.u = 'u';
% Wact.y = 'u_Wact';
Gp_inp_mult.u = 'u';
Gp_inp_mult.y = 'yG';
Gd.u = 'dw';
Gd.y = 'yGd';
% Gp_add.u = 'u';
% Gp_add.y = 'yG';
% aux_mat.u = 'u';
% aux_mat.y = 'y_Delta';
% W_A_ref.u = 'u_Delta';
% W_A_ref.y = 'y_W_A';

% % W_I_Delta.u = 'u';
% % W_I_Delta.y = 'u_mult';
% Gp_cor.u = 'u';
% Gp_cor.y = 'y';

% Sum_mult = sumblk('u_un = u + u_mult',3);
% Sum_mult = sumblk('u_un = u + y_W_I',3);
% Sum_out = sumblk('y_un = yG + y_W_A',3);
% Sum_err = sumblk('v = r - y_un',3);
% Sum_err = sumblk('v = r - y',3);
% inputs = {'u_Delta','r','d','u'};
% outputs = {'y_Delta','z1','z2','v'};

% Sum_err = sumblk('v = rw - yG - yGd - y_W_A',3);
Sum_err = sumblk('v = rw - yG - yGd',3);
inputs = {'r','d','u'};
outputs = {'z1','z2','v'};
% Paug = connect(G,Gd,Wp,Wu,Wd,Wr,Sum_err,inputs,outputs);
% Paug = connect(Gp_app,Gd,Wp,Wu,Wd,Wr,Sum_err,inputs,outputs);
% Paug = connect(G,Wp,Wu,W_O,Sum_out,Sum_err,inputs,outputs);
% Paug = connect(G,Wp,Wu,W_A_ref,sup_mat,Sum_out,Sum_err,inputs,outputs);
% Paug = connect(Gp_cor,Wp,Wu,Sum_err,inputs,outputs);
% Paug = connect(G,Gd,W_A_ref,aux_mat,Wp,Wu,Wr,Wd,Sum_err,inputs,outputs);
Paug = connect(Gp_inp_mult,Gd,Wp,Wu,Wr,Wd,Sum_err,inputs,outputs);

Paug = minreal(Paug);

% Generalized feedback interconnection of Delta block P block
% Punc = lft(Delta_A,Paug);
% Punc = minreal(Punc);
%% Approximated Perturbed Plant by the Additive Uncertainty
aux_mat.u = 'u';
aux_mat.y = 'y_Delta';
W_A_ref.u = 'u_Delta';
W_A_ref.y = 'y_W_A';
G.u = 'u';
G.y = 'y';

Sum_add = sumblk('y_un = y + y_W_A',3);
inputs = {'u_Delta','u'};
outputs = {'y_Delta','y_un'};
Gp_aug = connect(G,W_A_ref,aux_mat,Sum_add,inputs,outputs);
Gp_aug = minreal(Gp_aug);

Gp_app = lft(Delta_A,Gp_aug);
Gp_app = minreal(Gp_app);

% Alternative method ***
% sys1 = series(aux_mat,Delta_A);
% sys2 = series(sys1,W_A_ref);
% 
% Gp_app2 = parallel(G,sys2);
% Gp_app2 = minreal(Gp_app2);


%% mu-synthesis of Hinf Controller - Perturbed Plant

mu_opts = musynOptions('Display','full','FullDG',false);%,'FitOrder',[5 2]);
tic;
% mu_opts = musynOptions('Display','full','TargetPerf',1,'FullDG',false);%,'FrequencyGrid',[1e-1,1e1]);
[Kunc,CLunc,info_unc] = musyn(Paug,nmeas,ncont);%,mu_opts); 
timerun = toc;

loops_p = loopsens(G,Kunc);
Lp = loops_p.Lo;
Tp = loops_p.To;
Sp = loops_p.So;

% Robust stability of uncertain system
% [stabmarg,wcu] = robstab(Tp);
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
sigma(Sp,inv(Wp),sigma_opts);
legend('\boldmath{$\sigma(S)$}','interpreter','latex','FontSize',15)

figure
sigma(Tp,sigma_opts);
legend('\boldmath{$\sigma(T)$}','interpreter','latex','FontSize',15)

figure
sigma(Kunc*Sp,inv(Wu),sigma_opts);
legend('\boldmath{$\sigma(KS)$}','interpreter','latex','FontSize',15)

figure
sigma(Gp_app*Kunc,sigma_opts);
legend('\boldmath{$\sigma(GK)$}','interpreter','latex','FontSize',15)

figure
sigma(Sp*Gd,sigma_opts);
legend('\boldmath{$\sigma(SG_d)$}','interpreter','latex','FontSize',15)

figure
sigma(Kunc*Sp*Gd,sigma_opts);
legend('\boldmath{$\sigma(K S G_d)$}','interpreter','latex','FontSize',15)

%% Simulation of the closed loop system with the Hinf controller

figure
step(T)
title('Hinf')
grid on

figure
step(Tp)
title('mu-synthesis')
grid on
%% Simulation with step signal on heave
dt = 0.01; % sampling time
tend = 30; % duration of simulation in seconds
t = 0:dt:tend;

% ref = [0.5*sin(t);0*ones(size(t));0*ones(size(t))];
ref = [-0.05*square(0.5*t);0*ones(size(t));0*ones(size(t))];

x0 = [0, 0, 0, 0, 0, 0];
[y,~,~] = lsim(Tp,ref,t);

% figure
% lsim(Tp,ref,t);

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

inp_val = lsim(Kunc*Sp,ref,t);
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

[y,~,x] = lsim(S*Gd,dw,t);
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
inp_val = lsim(-K*S*Gd,dw,t);

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
ncfmr(Kunc)
[~,info] = ncfmr(Kunc);
[marg,freq] = ncfmargin(G,Kunc);
Kunc_red = ncfmr(Kunc,20,info);
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