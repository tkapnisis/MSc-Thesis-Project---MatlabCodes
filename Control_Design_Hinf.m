% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft


clc
clear all
close all

load('LTI_Perturbed_Plant.mat','G','Gd','Gp','Gd_p','Gp_s','Gd_p_s')
load('Parameters_Nominal.mat','param')
load('LTI_Nominal_Plant.mat','foil_loc')

% Nominal plant G(s)
% Disturbances transfer matrix Gd(s)
% Perturbed plant with uncertain parameters Gp(s)
% Perturbed disturbances transfer matrix with uncertain parameters Gd_p(s)
%%
%
% [M,Delta,BlkStruct] = lftdata(GpUN);

%%
opts = bodeoptions;
opts.MagScale = 'log';
opts.MagUnits = 'abs';
opts.InputLabels.Interpreter = 'none';
opts.InputLabels.FontSize = 10;
opts.OutputLabels.FontSize = 10;
opts.XLabel.FontSize = 11;
opts.YLabel.FontSize = 11;
opts.TickLabel.FontSize = 10;
opts.Title.FontSize = 12;
opts.PhaseVisible = 'off';

% Approximation of the uncertainty
omega = logspace(-2,2,100);
order = 4;
Wa = ss([]);
k=0;
figure
title('Relative Gaps (blue) vs. Shaping Filter Magnitude (red)')
% reldiff = frd([]);
for i=1:3
    for j=1:3
        Gnom_frd = frd(G(i,j),omega);
        Gp_samples = usample(Gp(i,j),50);
        Gp_frd = frd(Gp_samples,omega);
%         [usys,Info] = ucover(Gp_frd,Gnom_frd,order,order,'Additive');
        [~,Info] = ucover(Gp_frd,Gnom_frd,order,'InputMult');
        Wa(i,j) = Info.W1;
        Wa_frd = frd(Wa(i,j),omega);
        k=k+1;
        subplot(3,3,k)
        bodeplot((Gp_frd - Gnom_frd)/Gnom_frd,'b--',Wa_frd,'r',opts)
        j
    end
    i
end   

%%
order = [4,4,4];
Gnom_frd = frd(G,omega);
Gp_samples = usample(Gp,20);
Gp_frd = frd(Gp_samples,omega);
[usys,Info] = ucover(Gp_frd,Gnom_frd,order,'OutputMult');
Wa = Info.W1;
Wa_frd = frd(Wa,omega);
%%
Gnom_frd = frd(G,omega);
Gp_samples = usample(Gp,10);
Gp_frd = frd(Gp_samples,omega);
Wa_frd = frd(Wa,omega);


% diff = Gnom_frd-Gp_frd;

figure
bodeplot((Gp_frd - Gnom_frd)*inv(Gnom_frd),'b--',Wa_frd,'r',opts)
% title('Absolute Gaps (blue) vs. Shaping Filter Magnitude (red)')
title('Relative Gaps (blue) vs. Shaping Filter Magnitude (red)')

%%
figure
bodeplot(usys,'b--', Gp_frd ,'r',opts)
grid on
legend('Approximated Gp', 'Actual Gp')

figure
bodeplot(usys,'b--', Gnom_frd ,'r',opts)
grid on
legend('Approximated Gp', 'Nominal G')
%%
Gnom_frd = frd(G,omega);
% Defining the block diagonal Multiplicative uncertainties
Delta_i = ultidyn('Delta_i',[3,3],'Bound',0.5);
% Delta_i =[ultidyn('do1',[1,1],'Bound',1) 0 0;...
%           0 ultidyn('do2',[1,1],'Bound',1) 0;...
%           0  0 ultidyn('do3',[1,1],'Bound',1)];

% Defining the uncertain transfer matrix
GpUN = G*(eye(3) + Wa*Delta_i);
% GpUN = (eye(3) + Delta_i*Wa)*G;
% GpUN = G +  Wa*Delta_a;    
GpUN = minreal(GpUN);

GpUN_frd = ufrd(GpUN,omega);
% Plot of the singular values of the uncertain transfer matrix
% figure
% bodeplot(usys,Gnom_frd,opts)
% title('Approximation ucover');
% grid on

figure
bodeplot(GpUN_frd,Gnom_frd,opts)
title('Custom Delta');
grid on

% figure
% bodeplot(Gp_frd,Gnom_frd,opts)
% legend('Perturbed','Nominal')

%%

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
pzplot(Gp_s,'r')
legend('Nominal plant G(s)','Simplified Perturbed plant Gp_s(s)','Location','best','FontSize',11)

% opts = sigmaoptions;
opts = bodeoptions;

opts.MagScale = 'log';
opts.MagUnits = 'abs';
opts.InputLabels.Interpreter = 'none';
opts.InputLabels.FontSize = 10;
opts.OutputLabels.FontSize = 10;
opts.XLabel.FontSize = 11;
opts.YLabel.FontSize = 11;
opts.TickLabel.FontSize = 10;
opts.Title.FontSize = 12;
% opts.XLimMode = 'manual';
% opts.Xlim = [1e-3 1e2];
opts.PhaseVisible = 'off';


figure
bodeplot(Gp,G,opts)
set(findall(gcf,'Type','line'),'LineWidth',1.2)
grid on
legend('Perturbed plant Gp(s)','Nominal plant G(s)','Location','best','FontSize',11)
% legend('G(s)','Gd','Location','best','FontSize',11)


figure
bodeplot(Gp_s,G,opts)
set(findall(gcf,'Type','line'),'LineWidth',1.2)
grid on
legend('Simplified Perturbed plant Gp_s(s)','Nominal plant G(s)','Location','best','FontSize',11)
%%
% Singular values and gamma analysis
%
% RGA
omega1 = 0;
Gf1 = freqresp(Gp,omega1);
RGAw_1(:,:) = Gf1.*inv(Gf1)';

% omega2 = 0.4*2*pi;
omega2 = 3;
Gf2 = freqresp(Gp,omega2);
RGAw_2(:,:) = Gf2.*inv(Gf2)';

[U1,S1,V1]=svd(Gf1);
sv1=diag(S1);
gamma1=sv1(1)/sv1(2);

[U2,S2,V2]=svd(Gf2);
sv2=diag(S2);
gamma2=sv2(1)/sv2(2);
%

% opts_sigma = sigmaoptions;
% opts_sigma.MagScale = 'log';
% opts_sigma.MagUnits = 'abs';
% opts.InputLabels.Interpreter = 'none';
% 
% figure
% sigmaplot(G,opts_sigma)
% grid on

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
for i=1:size(sv,2)
    gamma(i) = sv(1,i)/sv(3,i);
end    
% figure 
% plot(gamma)
%}
%% Weights design for error signal, inputs and ouputs

s = zpk('s');

M_1 = 1.5;
M_2 = 1.5;
M_3 = 1.5;
w_c1 = 0.5*2*pi;
w_c2 = 1*2*pi;
w_c3 = 1*2*pi;
A_1 = 1e-4;
A_2 = 1e-4;
A_3 = 1e-4;

n = 1;

Wp11 = (s/M_1*(1/n) + w_c1)^n/(s + w_c1*A_1^(1/n))^n;
Wp22 = (s/M_2*(1/n) + w_c2)^n/(s + w_c2*A_2*(1/n))^n;
Wp33 = (s/M_3*(1/n) + w_c3)^n/(s + w_c3*A_3*(1/n))^n;
Wp = [Wp11,   0  ,   0 ;
        0 , Wp22 ,   0 ;
        0 ,   0  , Wp33];

% Wu11 = 1/2;
% Wu22 = 1/2;
% Wu33 = 1/2;

% Wu11 = makeweight(1e-2, [2*2*pi,1e-1], 1e2);
Wu11 = makeweight(1e-3, [1*2*pi,1e-1], 1e1);
Wu22 = Wu11;
Wu33 = Wu11;

Wu = 1*[Wu11,   0  ,   0 ;
        0 , Wu22 ,   0 ;
        0 ,   0  , Wu33];
% Wu = zpk(Wu);

Wd = zpk(1*eye(6));

% zeta_1 = 1;
% zeta_2 = 1;
% zeta_3 = 1;
% Wr11 = w_c1^2/(s^2 + 2*zeta_1*w_c1*s + w_c1^2);
% Wr22 = w_c2^2/(s^2 + 2*zeta_2*w_c2*s + w_c2^2);
% Wr33 = w_c3^2/(s^2 + 2*zeta_3*w_c3*s + w_c3^2);
% Wr = [Wr11,   0  ,   0 ;
%         0 , Wr22 ,   0 ;
%         0 ,   0  , Wr33];
Wr = zpk(1*eye(3));

tau_s = ureal('tau_s',0.0001,'Range',[0, 0.13]); 
Wi11 = (tau_s*s)/(tau_s*s + 1);
Wi22 = Wi11;
Wi33 = Wi11;

Wi = blkdiag(Wi11,Wi33,Wi33);
%%
figure
bode(Wu11,opts)
%% Generalized Plant - Nominal

Wp.u = 'v';
Wp.y = 'z1';
Wu.u = 'u';
Wu.y = 'z2';
% Wd.u = 'd';
% Wd.y = 'dw';
% Wr.u = 'r';
% Wr.y = 'rw';
G.u = 'u';
G.y = 'yG';
% Gd.u = 'dw';
% Gd.y = 'yGd';

% Sum_err = sumblk('v = rw - yG - yGd',3);
Sum_err = sumblk('v = r - yG',3);
% inputs = {'r','d','u'};
inputs = {'r','u'};
outputs = {'z1','z2','v'};
P = connect(G,Wp,Wu,Sum_err,inputs,outputs);

P = minreal(P);

%% Generalized Plant - Perturbed

Wp.u = 'v';
Wp.y = 'z1';
Wu.u = 'u';
Wu.y = 'z2';
% Input multiplicative uncertainty
GpUN.u = 'u';
GpUN.y = 'yG';
% W1.u = 'u';
% W1.y = 'yW1';
% Delta_i.u = 'yW1';
% Delta_i.y = 'yDelta';
% % Output multiplicative uncertainty
% G.u = 'u';
% G.y = 'yG';
% W1.u = 'yG';
% W1.y = 'yW1';
% Delta_o.u = 'yW1';
% Delta_o.y = 'yDelta';
Gd.u = 'd';
Gd.y = 'yGd';

% W2.u = 'yDelta';        
% W2.y = 'yW2';

% Sum_in = sumblk('u_un = u + yDelta',3);
Sum_out = sumblk('y = yG + yGd',3);
% Sum_err = sumblk('v = r - yG - yGd',3);
Sum_err = sumblk('v = r - y',3);

inputs = {'r','u','d'};
outputs = {'z1','z2','v'};
% Pun = connect(Gp_s,Gd_p_s,Wp,Wu,Wd,Sum_err,inputs,outputs);
% Pun = connect(G,Wp,Wu,W1,Delta_i,Sum_in,Sum_err,inputs,outputs);
Pun = connect(GpUN,Gd,Wp,Wu,Sum_err,inputs,outputs);
% Pun = connect(G,Wp,Wu,W1,Delta_o,Sum_out,Sum_err,inputs,outputs);

Pun = minreal(Pun);

% Wp.u = 'v';
% Wp.y = 'z1';
% Wu.u = 'u_un';
% Wu.y = 'z2';
% Wi.u = 'u';
% Wi.y = 'uw';
% G.u = 'u_un';
% G.y = 'yG';
% 
% Sum_err = sumblk('v = r - yG',3);
% Sum_inp = sumblk('u_un = u + uw',3);
% inputs = {'r','u'};
% outputs = {'z1','z2','v'};
% Pun = connect(G,Wp,Wu,Wi,Sum_err,Sum_inp,inputs,outputs);

% [~,sysr] = isproper(Pun);
% Pun = minreal(Pun);

%% Hinf Controller synthesis - Nominal Plant
nmeas = 3; % number of outputs 
ncont = 3; % number of inputs

[K,CL,gamma,info] = hinfsyn(P,nmeas,ncont);
gamma
%% mu-synthesis of Hinf Controller - Perturbed Plant
opts = musynOptions('MixedMU','on','FullDG',false,'FitOrder',[5 2]);
tic;
% opts = musynOptions('MixedMU','on');
[Kunc,CLunc,info_unc] = musyn(Pun,nmeas,ncont);%,opts); 
timerun = toc;
%%
loops = loopsens(Gp,K);
L = loops.Lo;
T = loops.To;
S = loops.So;

%%
% loops_p = loopsens(G,(eye(3)-Wi)*Kunc);
loops_p = loopsens(Gp,Kunc);
Lp = loops_p.Lo;
Tp = loops_p.To;
Sp = loops_p.So;
%%
loops_i = loopsens(G,(eye(3)-Wi)*K);
Lp_i = loops_i.Lo;
Tp_i = loops_i.To;
Sp_i = loops_i.So;

%%
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

%%
%{
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
dt = 0.01; % sampling time
tend = 30; % duration of simulation in seconds
t = 0:dt:tend;

% ref = [0.5*sin(t);0*ones(size(t));0*ones(size(t))];
ref = [-0.05*square(0.5*t);0*ones(size(t));0*ones(size(t))];

x0 = [0, 0, 0, 0, 0, 0];
% [y,~,~] = lsim(T,ref,t);

figure
lsim(Tp,ref,t);

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

inp_val = lsim(-K*S,ref,t);
% inp_val = lsim(K,ref'-y,t);

figure
plot(t,rad2deg(inp_val),'LineWidth', 1.5)
title('Servo motor angles - Control inputs')
grid minor
ylabel('\boldmath{$\theta_s$} \textbf{[deg]}','interpreter','latex')
xlabel('\textbf{time [s]}','interpreter','latex')
legend('Fore hydrofoil', 'Aft port hydrofoil', 'Aft starboard hydrofoil')

%% Simulation of the closed loop system with the Hinf controller for uncertain plant
figure
step(usample(Tp,15),1);
hold on
step(T,'r-.') 
legend('Perturbed Plant', 'Nominal Plant')

%% Simulation of the closed loop system with the Hinf controller

%  Calculation of waves velocity profile for each hydrofoil

% Parameters of long-crested regular wave
wave_param.omega_0 = 2;   % Wave frequency [rad/s]
wave_param.lambda = 1;    % Wave length [m]
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
opts.MagLowerLimMode = 'manual';
opts.MagLowerLim = 1e-5;
opts.PhaseVisible = 'off';
% opts.YLimMode = 'manual';
% opts.YLim = {[1e-5,1e1]};

figure
subplot(3,1,1)
bodeplot(S(1,1),1/Wp11,opts);
legend('Sensitivity S11','1/Wp11','Location','best')
grid on
subplot(3,1,2)
bodeplot(S(2,2),1/Wp22,opts)
legend('Sensitivity S22','1/Wp22','Location','best')
grid on
subplot(3,1,3)
bodeplot(S(3,3),1/Wp33,opts)
legend('Sensitivity S33','1/W33','Location','best')
grid on
%%
figure 
bodeplot(K*S,inv(Wu),opts)
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