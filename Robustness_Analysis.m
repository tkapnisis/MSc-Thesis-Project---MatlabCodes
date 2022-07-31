% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

%% Singular Values Plots for both hinf and mu-synthesis controllers
clc
clear all
close all

addpath('Plotting Functions')
addpath('Data Files')

load('Controller_mu_syn.mat')
load('Controller_hinf.mat')

% Nominal plant G(s)
% Disturbances transfer matrix Gd(s)
% Perturbed plant with uncertain parameters Gp(s)
% Perturbed disturbances transfer matrix with uncertain parameters Gd_p(s)

run Bode_options.m
run Sigma_options.m

%{
%% Check the Robustness of the mu-synthesis controller

opts_robstab = robOptions('VaryFrequency','on','Display','on','Sensitivity','on');
% ,...       'MussvOptions','a'
omega=logspace(-4,3,100);

Ndk=minreal(lft(P_Delta,mu_syn_data.K));

% Robust stability of uncertain system
[stabmarg,wcu,info_robstab] = robstab(Ndk,opts_robstab);

figure
loglog(info_robstab.Frequency,info_robstab.Bounds(:,1),'r-','LineWidth',1.5)
hold on
loglog(info_robstab.Frequency,info_robstab.Bounds(:,2),'b--','LineWidth',1)
grid on
title('Stability Margin vs. Frequency')
ylabel('Margin')
xlabel('Frequency')
legend('Lower bound','Upper bound')

%%
gamma = 1;
[perfmarg,wcu,info_robgain] = robgain(Ndk,gamma,opts_robstab);

figure
loglog(info_robgain.Frequency,info_robgain.Bounds(:,1),'r-','LineWidth',1.5)
hold on
loglog(info_robgain.Frequency,info_robgain.Bounds(:,2),'b--','LineWidth',1)
grid on
title('Performance Margin vs. Frequency')
ylabel('Margin')
xlabel('Frequency')
legend('Lower bound','Upper bound')
%}
%%
% Generalized feedback interconnection of P block K block in order to
% obtain the N tranfer matrix
[M,~,Blkstruct] = lftdata(P_Delta);
Blkstruct(6).Name = 'Delta_P';
Blkstruct(6).Size = [9,6];
Blkstruct(6).Type = 'ultidyn';
Blkstruct(6).Occurrences = 1;
Blkstruct(6).Simplify = 2;

mu_syn_res.N = minreal(lft(M,mu_syn_data.K));
hinf_res.N = minreal(lft(M,hinf_data.K));
omega=logspace(-4,4,300);

mu_syn_res.N_frd = frd(mu_syn_res.N,omega);
hinf_res.N_frd = frd(hinf_res.N,omega);

% Nominal stability
mu_syn_res.max_eig_N = max(real(eig(mu_syn_res.N))); 
hinf_res.max_eig_N = max(real(eig(hinf_res.N))); 

% Nominal performance
[mubnds,~] = mussv(mu_syn_res.N_frd(10:15,13:21),Blkstruct(6));
mu_syn_res.muNP = mubnds(:,1);
[mu_syn_res.muNPinf, mu_syn_res.muNPw] = norm(mu_syn_res.muNP,inf); 

[mubnds,~] = mussv(hinf_res.N_frd(10:15,13:21),Blkstruct(6));
hinf_res.muNP = mubnds(:,1);
[hinf_res.muNPinf, hinf_res.muNPw] = norm(hinf_res.muNP,inf);  

% Robust stability
[mubnds,~] = mussv(mu_syn_res.N_frd(1:9,1:12),Blkstruct(1:5));
mu_syn_res.muRS = mubnds(:,1);
[mu_syn_res.muRSinf, mu_syn_res.muRSw] = norm(mu_syn_res.muRS,inf); 

[mubnds,~] = mussv(hinf_res.N_frd(1:9,1:12),Blkstruct(1:5));
hinf_res.muRS = mubnds(:,1);
[hinf_res.muRSinf, hinf_res.muRSw] = norm(hinf_res.muRS,inf); 

% Robust performance
[mubnds,~] = mussv(mu_syn_res.N_frd(:,:),Blkstruct);
mu_syn_res.muRP = mubnds(:,1);
[mu_syn_res.muRPinf, mu_syn_res.muRPw] = norm(mu_syn_res.muRP,inf); 

[mubnds,~] = mussv(hinf_res.N_frd(:,:),Blkstruct);
hinf_res.muRP = mubnds(:,1);
[hinf_res.muRPinf, hinf_res.muRPw] = norm(hinf_res.muRP,inf);  

%% Frequency response of the structured singular values for NP, RS and RP

figure
loglog(omega,mu_syn_res.muNP.ResponseData(:),'LineWidth',1.5,'Color','#0072BD','LineStyle','-')
hold on
loglog(omega,mu_syn_res.muRS.ResponseData(:),'LineWidth',1.5,'Color','#D95319','LineStyle','-')
loglog(omega,mu_syn_res.muRP.ResponseData(:),'LineWidth',1.5,'Color','#EDB120','LineStyle','-')
loglog(omega,hinf_res.muNP.ResponseData(:),'LineWidth',1.5,'Color','#0072BD','LineStyle','--')
loglog(omega,hinf_res.muRS.ResponseData(:),'LineWidth',1.5,'Color','#D95319','LineStyle','--')
loglog(omega,hinf_res.muRP.ResponseData(:),'LineWidth',1.5,'Color','#EDB120','LineStyle','--')
grid on
title('Structured singular values μ for NP, RS and RP','FontSize',12)
ylabel('Margin')
xlabel('Frequency')
legend('\boldmath{$\mu_{\Delta_P}(N_{22}(j\omega))$} \textbf{-NP (\boldmath{$\mu$}-synthesis)}',...
       '\boldmath{$\mu_{\Delta}(N_{11}(j\omega))$}\textbf{-RS (\boldmath{$\mu$}-synthesis)}',...
       '\boldmath{$\mu_{\hat{\Delta}}(N(j\omega))$}\textbf{-RP (\boldmath{$\mu$}-synthesis)}',...
       '\boldmath{$\mu_{\Delta_P}(N_{22}(j\omega))$} \textbf{-NP (\boldmath{$h_{\infty}$})}',...
       '\boldmath{$\mu_{\Delta}(N_{11}(j\omega))$}\textbf{-RS (\boldmath{$h_{\infty}$})}',...
       '\boldmath{$\mu_{\hat{\Delta}}(N(j\omega))$}\textbf{-RP (\boldmath{$h_{\infty}$})}',...
       'interpreter','latex','FontSize',12,'Location','best')

ax=gca;
ax.XAxis.FontSize = 12;
ax.YAxis.FontSize = 12;