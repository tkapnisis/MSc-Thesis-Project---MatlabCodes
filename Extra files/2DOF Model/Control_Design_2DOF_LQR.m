clc
clear all
close all

load('Linear_State_Space_2DOF.mat')
run Define_Parameters_2DOF.m
%% LQR control design

Q = diag([100 100 0.01 0.01]);
% Q = diag([1 300 1 33]);
R = diag([1, 1]);
K = lqr(A,B,Q,R);

states = {'z', 'theta', 'z_dot', 'theta_dot'};
inputs = {'theta_s_f';'theta_s_a'};
outputs = {'z'; 'theta'};
eig(A-B*K)

sys_cl = ss(A-B*K,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

Lc=inv(dcgain(sys_cl));
sys_cl_Lc = ss(A-B*K,B*Lc,C,D);%,'statename',states,'inputname',inputs,'outputname',outputs);

figure;
pzmap(sys_cl_Lc);
grid on
%%
% Simulation of the closed loop system with the LQR controller
dt = 0.01; % sampling time
tend = 5; % duration of simulation in seconds
t = 0:dt:tend;

ref = diag([-0.05, deg2rad(0)])*ones(size(C,1),size(t,2));
x0 = [0, 0, 0, 0];
[y,t,x]=lsim(sys_cl_Lc,ref,t,x0);

figure
step(sys_cl_Lc)
grid minor

figure
subplot(2,1,1)
plot(t,y(:,1) + h_0,'LineWidth',1.5)
title('Heave')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$z_n$} \textbf{[m]}','interpreter','latex')
grid minor
subplot(2,1,2)
plot(t,rad2deg(y(:,2)),'LineWidth',1.5)
title('Pitch')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\phi$} \textbf{[deg]}','interpreter','latex')
grid minor

inp_val=Lc*ref-K*x';
figure
plot(t,rad2deg(inp_val),'LineWidth', 1.5)
title('Servo motor angles - Control inputs')
grid minor
ylabel('\boldmath{$\theta_s$} \textbf{[deg]}','interpreter','latex')
xlabel('\textbf{time [s]}','interpreter','latex')
legend('Fore hydrofoil', 'Aft hydrofoils')

%%
% State space of the controller obtained from: 
% - Astrom and Murray, An Introduction for Scientists and Engineers, SECOND
% EDITION, Chapter 9, page 9-20) 
% - Alberto Bemporad, Pole placement by dynamic output feedback, 2010-2011,
% page 15/18
% - Jonathan How, Feedback Control, 2001, page 5/23
Gp = sys_ss;
% Gc = ss(A-B*K,B*Lc,-K, Lc);
Gc = ss(A-B*K,B*Lc,K, zeros(2,2));
% Gc = ss(A-B*K,zeros(4,2),-K, zeros(2,2));
loops = loopsens(Gp,Gc);

figure
opts = bodeoptions;
opts.MagScale = 'log';
opts.MagUnits = 'abs';
bodemag(loops.So,loops.To,loops.Lo,opts)
legend('Sensitivity S(s)','Complementary Sensitivity T(s)','Loop L(s)')
grid on

%%
% % Loop Tranfer Function
% L = Gp*Gc;
% 
% Complementary sensitivity transfer function
% T = L/(1 + L);
% 
% Sensitivity transfer function
% S = inv(1 + L);
% % 
% figure
% opts = bodeoptions;
% opts.MagScale = 'log';
% opts.MagUnits = 'abs';
% bodemag(S,T,opts)
% legend('Sensitivity S(s)','Complementary Sensitivity T(s)','Loop L(s)')
% grid on
