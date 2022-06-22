% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

clc
clear all
close all

load('Linear_State_Space_3DOF_Un.mat')
load('Parameters_3DOF.mat','param')
% load('Linear_State_Space_3DOF.mat')

% Nominal plant G(s)
% Disturbances transfer matrix Gd(s)
% Perturbed plant with uncertain parameters Gp(s)
% Perturbed disturbances transfer matrix with uncertain parameters Gd_p(s)
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
% opts.XLimMode = 'manual';
% opts.Xlim = [1e-3 1e2];
opts.PhaseVisible = 'off';

% sys_un_ss_SAMPLES = usample(sys_un_ss,10); 
figure
bodeplot(Gp,'r',G,'b',opts);
% set(findall(gcf,'Type','line'),'LineWidth',1.2)
grid on
legend('Perturbed plant Gp(s)','Nominal plant G(s)','Location','best','FontSize',11)

%%
%{
% RGA
omega1 = 0;
Gf1 = freqresp(Gp,omega1);
RGAw_1(:,:) = Gf1.*inv(Gf1)'

% omega2 = 0.4*2*pi;
omega2 = 3;
Gf2 = freqresp(Gp,omega2);
RGAw_2(:,:) = Gf2.*inv(Gf2)'

[U1,S1,V1]=svd(Gf1);
sv1=diag(S1);
gamma1=sv1(1)/sv1(2)

[U2,S2,V2]=svd(Gf2);
sv2=diag(S2);
gamma2=sv2(1)/sv2(2)
%}

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

%% Weights design for error signal, inputs and ouputs

s = zpk('s');

M_1 = 1.5;
M_2 = 1.5;
M_3 = 1.5;
w_c1 = 3*2*pi;
w_c2 = 4*2*pi;
w_c3 = 4*2*pi;
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

% a_1=0.5*2*pi;
% a_2=0.5*2*pi;
% a_3=0.5*2*pi;
% w_u1=2*2*pi;
% w_u2=2*2*pi;
% w_u3=2*2*pi;
% Wu11 = (s + a_1)/(s + w_u1);
% Wu22 = (s + a_2)/(s + w_u2);
% Wu33 = (s + a_3)/(s + w_u3);

w_u_k = 1;
w_u1 = w_c1;
w_u2 = w_c2;
w_u3 = w_c3;
Wu11 = w_u_k*s/(s + w_u1);
Wu22 = w_u_k*s/(s + w_u2);
Wu33 = w_u_k*s/(s + w_u3);

% Wu11 = 0.5;
% Wu22 = 0.5;
% Wu33 = 0.5;

Wu = [Wu11,   0  ,   0 ;
        0 , Wu22 ,   0 ;
        0 ,   0  , Wu33];

% figure
% bode(1/Wp11,opts)
%% Generalized Plant
% N = [ Wp*S  ]
%     [ Wu*K*S]

% w= [r1 r2]

% z1 = Wp*w + Wp*G*u
% z2 = Wu*u
% v = -w - G*u

% P which represents the transfer function matrix from [ w u ]' to [ z v ]'
% P = [ Wp*I   Wp*G ]
%     [  0      Wu*I]
%     [  -I      -G ]

% Construct a generalized plant for simulation reasons

warning off

% The full hydrodoil craft model
systemnames ='G Wp Wu';   
% Input to generalized plant
inputvar ='[r(3); Theta_s_f; Theta_s_ap; Theta_s_as]';   
% Output generalized plant
outputvar= '[Wp; Wu; r-G]';   
% Input to the plant
input_to_G= '[Theta_s_f; Theta_s_ap; Theta_s_as]';   
% Input to the weight of the inputs
input_to_Wu= '[Theta_s_f; Theta_s_ap; Theta_s_as]';  
% Input to the weight of the plant output
input_to_Wp= '[r-G]';  

sysoutname='P';
% cleanupsysic = 'yes';

sysic;

warning on

P = minreal(P);

%% Controller synthesis
nmeas = 3; % number of outputs 
ncont = 3; % number of inputs

[K,CL,gamma,info] = hinfsyn(P,nmeas,ncont);

% loops = loopsens(Gp,K);
loops = loopsens(G,K);
L = loops.Lo;
T = loops.To;
S = loops.So;

N = [Wp*S;Wu*K*S];
% N= minreal(N);

Nmax = hinfnorm(N)

% L=minreal(Gp*K);          % Loop tranfer function
% Lpoles=pole(L+eye(3));    % Open loop poles 
% Lzeros=tzero(L+eye(3));   % Closed loop poles

Kpoles=pole(K);        % poles of the controller
Kzeros=tzero(K);       % invariant zeros of MIMO controller

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
%%
% Simulation of the closed loop system with the LQR controller
dt = 0.01; % sampling time
tend = 30; % duration of simulation in seconds
t = 0:dt:tend;

% ref = [0.5*sin(t);0*ones(size(t));0*ones(size(t))];
ref = [-0.05*square(t);0*ones(size(t));0*ones(size(t))];
x0 = [0, 0, 0, 0, 0, 0];
[y,t,x] = lsim(sys_CL,ref,t);

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

% inp_val = lsim(K*S,ref,t);
inp_val = lsim(K,ref'-y,t);

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
bodeplot(Wu*K*S,inv(Wu),opts)
grid on
title('Bode plot of controller K*S versus its weight 1/Wu')
legend('K*S','1/Wu')
%%
figure 
bodeplot(inv(Wu),opts)
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
step(sys_CL(1,1))
title('Heave Step Response')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$z_n$} \textbf{[m]}','interpreter','latex')
grid minor

subplot(3,1,2)
step(sys_CL(2,2))
title('Roll')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\phi$} \textbf{[rad]}','interpreter','latex')
grid minor
subplot(3,1,3)
step(sys_CL(3,3))
title('Pitch')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\theta$} \textbf{[rad]}','interpreter','latex')
grid minor

% figure
% step(sys_CL)
Step_info = stepinfo(sys_CL);
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