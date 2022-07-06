% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft


clc
clear all
close all

load('LTI_Perturbed_Plant.mat','G','Gd','Gp','Gd_p')
load('Parameters_Nominal.mat','param')
load('LTI_Nominal_Plant.mat','foil_loc')
% load('Uncertainty_Approximation.mat','Delta_O','W_O','Delta_O_d','W_O_d',...
%                                      'Gp_app','Gd_p_app')
% load('Uncertainty_Approximation2.mat','Gp_app','W_O','Delta_O')
% load('Uncertainty_Approximation5.mat','W_A')
% load('Uncertainty_Approximation6.mat','Gp_ap','Gp_app','W_I')
% load('Uncertainty_ApproximationLAST.mat','W_I')

load('Gp_out_mult.mat')


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
bode_opts.XLimMode = 'manual';
bode_opts.Xlim = [1e-3 1e2];
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

figure
bodeplot(Gp,G,bode_opts)
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
% Wd.u = 'd';
% Wd.y = 'dw';
% Wr.u = 'r';
% Wr.y = 'rw';
% Wact.u = 'u';
% Wact.y = 'u_act';
% G.u = 'u_act';
% G.y = 'yG';
% Gd.u = 'dw';
% Gd.y = 'yGd';
G.u = 'u';
G.y = 'y';


% Sum_err = sumblk('v = rw - yG - yGd',3);
Sum_err = sumblk('v = r - y',3);
% Sum_err = sumblk('v = r - yG',3);
% inputs = {'r','d','u'};
inputs = {'r','u'};
outputs = {'z1','z2','v'};
% P = connect(G,Gd,Wp,Wu,Wd,Wr,Sum_err,inputs,outputs);
% P = connect(G,Gd,Wp,Wu,Wd,Sum_err,inputs,outputs);
% P = connect(G_sc,Gd_sc,Wp,Wu,Wd,Sum_err,inputs,outputs);
% P = connect(G,Gd,Wp,Wu,Wd,Wr,Wact,Sum_err,inputs,outputs);
P = connect(G,Wp,Wu,Sum_err,inputs,outputs);

P = minreal(P);

% Hinf Controller synthesis - Nominal Plant
nmeas = 3; % number of outputs 
ncont = 3; % number of inputs

[K,CL,gamma,info] = hinfsyn(P,nmeas,ncont);
gamma

% loops = loopsens(G*Wact,K);
loops = loopsens(Gp,K);
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
bound = 1;
Delta_I = [ultidyn('d11',[1,1],'Bound',bound),...
           ultidyn('d12',[1,1],'Bound',bound),...
           ultidyn('d13',[1,1],'Bound',bound);...
           ultidyn('d21',[1,1],'Bound',bound),...
           ultidyn('d22',[1,1],'Bound',bound),...
           ultidyn('d23',[1,1],'Bound',bound);...
           ultidyn('d31',[1,1],'Bound',bound),...
           ultidyn('d32',[1,1],'Bound',bound),...
           ultidyn('d33',[1,1],'Bound',bound)];

% Defining the multiplicative weiting tranfer matrix
% W_I_Delta = uss([]);

for i=1:size(G,1)
    for j=1:size(G,2)
        temp = strcat('w',num2str(i),num2str(j));
        W_I_Delta.(temp) = W_I.(temp)*Delta_I(i,j);
    end
end
% W_I_Delta = minreal(W_I_Delta);

%%
figure
pzmap(Gp_app)
grid on
%%
figure
bodeplot(Gp_ap,bode_opts)

%%

% [M,Delta,BlkStruct] = lftdata(Gp_app);
% W_I_ref = [W_I(1,1),        0,        0, W_I(1,2),        0,        0, W_I(1,3),        0,        0;...
%                   0, W_I(2,1),        0,        0, W_I(2,2),        0,        0, W_I(2,3),       0;...
%                   0,        0, W_I(3,1),        0,        0, W_I(3,2),        0,        0, W_I(2,3)];
W_I_ref = [W_I.w11,        0,        0, W_I.w21,        0,        0, W_I.w31,        0,        0;...
                  0, W_I.w12,        0,        0, W_I.w22,        0,        0, W_I.w32,       0;...
                  0,        0, W_I.w13,        0,        0, W_I.w23,        0,        0, W_I.w33];

%%
[M,Delta_I_diag,BlkStruct] = lftdata(Gp_ap);


%%
sup_mat = ss([ ones(3,1), zeros(3,1), zeros(3,1);...
           zeros(3,1),  ones(3,1), zeros(3,1);...
           zeros(3,1), zeros(3,1),  ones(3,1)]);



%%
res = W_I_ref*Delta_I_diag*sup_map;
% [M,Delta,BlkStruct] = lftdata(Gp_app_ref);
%%
[Wp,Wu,Wd,Wr,Wact] = Hinf_Weights_Design();

Wp.u = 'v';
Wp.y = 'z1';
Wu.u = 'u';
Wu.y = 'z2';
% Wd.u = 'd';
% Wd.y = 'dw';
% Wr.u = 'r';
% Wr.y = 'rw';
% Wact.u = 'u';
% Wact.y = 'u_Wact';
% W_O.u = 'yG';
% W_O.y = 'yW_O';
% W_I.u = 'u';
% W_I.y = 'yDelta_I';
% G.u = 'u';
% G.y = 'yG';
% Gd.u = 'dw';
% Gd.y = 'yGd';
% G.u = 'u';
% G.y = 'yG';
% sup_mat.u = 'u';
% sup_mat.y = 'y_Delta';
% W_I_ref.u = 'u_Delta';
% W_I_ref.y = 'y_W_I';
% % W_I_Delta.u = 'u';
% % W_I_Delta.y = 'u_mult';
Gp_out_mult.u = 'u';
Gp_out_mult.y = 'y';

% Sum_mult = sumblk('u_un = u + u_mult',3);
% Sum_mult = sumblk('u_un = u + y_W_I',3);
% Sum_err = sumblk('v = rw - yG - yGd',3);
% Sum_out = sumblk('y_un = yG + uDelta_O',3);
% Sum_err = sumblk('v = r - y_un',3);
Sum_err = sumblk('v = r - y',3);
% inputs = {'u_Delta','r','u'};
inputs = {'r','u'};
% outputs = {'y_Delta','z1','z2','v'};
outputs = {'z1','z2','v'};
% Paug = connect(Gp_app,Gd,Wp,Wu,Wd,Wr,Sum_err,inputs,outputs);
% Paug = connect(G,Wp,Wu,W_O,Sum_out,Sum_err,inputs,outputs);
% Paug = connect(G,Wp,Wu,W_I_ref,sup_mat,Sum_mult,Sum_err,inputs,outputs);
Paug = connect(Gp_out_mult,Wp,Wu,Sum_err,inputs,outputs);
% Paug = connect(G,W_I_ref,sup_mat,Wp,Wu,Sum_mult,Sum_err,inputs,outputs);

% Paug = minreal(Paug);

%% Gp reformulated

%%
W_I.w11.u = 'u';
W_I.w11.y = 'yDelta';

res = ultidyn('d11',[1,1],'Bound',bound);
res.u = 'yDelta';
res.y = 'uDelta';

multi = connect(W_I.w11,res,'u','uDelta');
%%
W_I.w12.u = 'u(2)';
W_I.w13.u = 'u(3)';
W_I.w21.u = 'u(1)';
W_I.w22.u = 'u(2)';
W_I.w23.u = 'u(3)';
W_I.w31.u = 'u(1)';
W_I.w32.u = 'u(2)';
W_I.w33.u = 'u(3)';

W_I_Delta.w11.y = 'y(1)';
W_I_Delta.w12.y = 'y(1)';
W_I_Delta.w13.y = 'y(1)';
W_I_Delta.w21.y = 'y(2)';
W_I_Delta.w22.y = 'y(2)';
W_I_Delta.w23.y = 'y(2)';
W_I_Delta.w31.y = 'y(3)';
W_I_Delta.w32.y = 'y(3)';
W_I_Delta.w33.y = 'y(3)';

W_I_Delta_mat = [W_I_Delta.w11, W_I_Delta.w12, W_I_Delta.w13;...
                 W_I_Delta.w21, W_I_Delta.w22, W_I_Delta.w23;...
                 W_I_Delta.w31, W_I_Delta.w32, W_I_Delta.w33];

%%
sup_mat.u = 'u';
sup_mat.y = 'y_Delta';
W_I_ref.u = 'u_Delta';
W_I_ref.y = 'y_W_I';

% W_I_Delta_mat.u = 'u';
% W_I_Delta_mat.y = 'u_mult';
% G.u = 'u_un';
% G.y = 'y';

Sum_mult = sumblk('u_un = u + y_W_I',3);
inputs = {'u_Delta','u'};
outputs = {'y_Delta','y'};
Gp_aug = connect(G,W_I_ref,sup_mat,Sum_mult,inputs,outputs);
Gp_aug = minreal(Gp_aug);

%%
Gp_cor = lft(Delta_I_diag,Gp_aug);
Gp_cor = minreal(Gp_cor);


%%
% Generalized feedback interconnection of Delta block P block
Punc = lft(Delta_I_diag,Paug);
Punc = minreal(Punc);
%% mu-synthesis of Hinf Controller - Perturbed Plant
%
% opts = musynOptions('MixedMU','on','FullDG',false,'FitOrder',[5 2]);
tic;
opts = musynOptions('Display','full','TargetPerf',1,'FullDG',false,'FrequencyGrid',[1e-1,1e1]);
[Kunc,CLunc,info_unc] = musyn(Paug,nmeas,ncont,opts); 
timerun = toc;
%}

%%
%
% loops_p = loopsens(G,(eye(3)-Wi)*Kunc);
loops_p = loopsens(Gp,Kunc);
Lp = loops_p.Lo;
Tp = loops_p.To;
Sp = loops_p.So;

% figure
% lsim(T,ref,t);
%{ 
loops_i = loopsens(G,(eye(3)-Wi)*K);
Lp_i = loops_i.Lo;
Tp_i = loops_i.To;
Sp_i = loops_i.So;

%
figure
opts = bodeoptions;
opts.MagScale = 'log';
opts.MagUnits = 'abs';
opts.PhaseVisible = 'off';
% opts.XLimMode = 'manual';
% opts.XLim = {[1e-2,1e2]};
bodeplot(loops.So,loops.To,loops.Lo,opts)
legend('Sensitivity S(s)','Complementary Sensitivity T(s)','Loop L(s)')
grid on

% K.InputName = {'e_zn','e_roll','e_pitch'};  
% K.OutputName = {'Theta_s_f','Theta_s_f','Theta_s_as'};
K.InputName = {'e'};  
% K.OutputName = {'u'};
K.OutputName = {'theta_sf';'theta_sap';'theta_sas'};
K = minreal(K);

formula = 'e = r - %y';
signames = ["z", "phi", "theta"];
Sum = sumblk(formula,signames);
% Sum = sumblk('e = r - y',3);
% sys_CL = connect(Gp,K,Sum,'r',{'z', 'phi', 'theta'});
sys_CL = connect(G,K,Sum,'r',{'z', 'phi', 'theta'});

% https://nl.mathworks.com/help/robust/ref/uss.musyn.html
%}

%% Simulation of the closed loop system with the Hinf controller

figure
step(T)
title('Hinf')
grid on

figure
step(Tp)
title('mu-synthesis')
grid on
%%



dt = 0.01; % sampling time
tend = 30; % duration of simulation in seconds
t = 0:dt:tend;

% ref = [0.5*sin(t);0*ones(size(t));0*ones(size(t))];
ref = [-0.05*square(0.5*t);0*ones(size(t));0*ones(size(t))];

x0 = [0, 0, 0, 0, 0, 0];
[y,~,~] = lsim(T,ref,t);

figure
lsim(Tp,ref,t);

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

inp_val = lsim(K*S,ref,t);
% inp_val = lsim(K,ref'-y,t);

figure
plot(t,rad2deg(inp_val),'LineWidth', 1.5)
title('Servo motor angles - Control inputs')
grid minor
ylabel('\boldmath{$\theta_s$} \textbf{[deg]}','interpreter','latex')
xlabel('\textbf{time [s]}','interpreter','latex')
legend('Fore hydrofoil', 'Aft port hydrofoil', 'Aft starboard hydrofoil')

figure
step(T);
grid minor
%% Simulation of the closed loop system with the Hinf controller for uncertain plant
figure
step(usample(Tp,2),2);
hold on
step(T,'r-.') 
legend('Perturbed Plant', 'Nominal Plant')

%% Simulation of the closed loop system with the Hinf controller

%  Calculation of waves velocity profile for each hydrofoil

% Parameters of long-crested regular wave
wave_param.omega_0 = 1.5;   % Wave frequency [rad/s]
wave_param.lambda = 2;    % Wave length [m]
wave_param.zeta_0 = 0.1;  % Wave amplitude [m]
wave_param.beta = pi;     % Encounter angle (beta=0 for following waves) [rad] 

[dw,wave_param] = Wave_Model(t,wave_param,foil_loc,param);
%%
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

% inp_val = lsim(K,-y,t);
inp_val = lsim(-K*S*Gd,dw,t);

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