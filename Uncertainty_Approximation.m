% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

clc
clear all
close all

addpath('Plotting Functions')
addpath('Data Files')

load('LTI_Perturbed_Plant.mat','G','Gd','Gp','Gd_p')
load('Multiplicative_Input_Uncertainty.mat')

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
opts_bode.PhaseVisible = 'off';
opts_bode.Grid = 'on';

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

%% Calculate weighting functions for multiplicative uncertanties
%{
% Approximate the parametric uncertainties as multiplicative uncertainties
% for each channel of the perturbed plant
omega = logspace(-2,4,200);
samples = 50;

% Approximation of the uncertainty
G_frd = frd(G,omega);
Gp_samples = usample(Gp,samples);
Gp_frd = frd(Gp_samples,omega);

Gd_frd = frd(Gd,omega);
Gd_p_samples = usample(Gd_p,samples);
Gd_p_frd = frd(Gd_p_samples,omega);

W_I_G_ss = ss([]);
W_I_Gd_ss = ss([]);

order = 3;
for i=1:size(G,1)
    for j=1:size(G,2)
        [~,Info] = ucover(Gp_frd(i,j,:,:),G(i,j),order,'InputMult');
        temp= strcat('w',num2str(i),num2str(j));
        W_I_G.(temp) = Info.W1;
        W_I_G_ss(i,j) = Info.W1;
        rel_dif_G(i,j,:,:) = (Gp_frd(i,j) - G_frd(i,j))/G_frd(i,j);
        rel_dif_G.u(j) = G.u(j);
        rel_dif_G.y(i) = G.y(i);
        j
    end
end  

for i=1:size(Gd,1)
    for j=1:size(Gd,2)
        [~,Info] = ucover(Gd_p_frd(i,j,:,:),Gd(i,j),order,'InputMult');
        temp= strcat('w',num2str(i),num2str(j));
        W_I_Gd.(temp) = Info.W1;
        W_I_Gd_ss(i,j) = Info.W1;
        rel_dif_Gd(i,j,:,:) = (Gd_p_frd(i,j) - Gd_frd(i,j))/Gd_frd(i,j);
        rel_dif_Gd.u(j) = Gd_frd.u(j);
        rel_dif_Gd.y(i) = Gd_frd.y(i);
        j
    end
end  

figure
title('Approximation of parametric uncertainties by multiplicative uncertainties for each channel')
bodeplot(rel_dif_G,'b--',W_I_G_ss,'r',omega,opts_bode);
legend('\boldmath{$|(G_p(j\omega)-G(j\omega))/G(j\omega)|$}','\boldmath{$|W_I_G(j\omega)|$}',...
       'interpreter','latex','FontSize',12)
set(gcf, 'WindowState', 'maximized');
saveas(gcf,[pwd '/Figures/Parametric to Multiplicative Input/Relative Differences G.png'])

figure
title('Approximation of parametric uncertainties by multiplicative input uncertainties')
bodeplot(rel_dif_Gd,'b--',W_I_Gd_ss,'r',omega,opts_bode);
grid on
legend('\boldmath{$|(G_{d,p}(j\omega)-G_d(j\omega))/G_d(j\omega)|$}',...
       '\boldmath{$|W_I_Gd(j\omega)|$}','interpreter','latex','FontSize',12)
set(gcf, 'WindowState', 'maximized');
saveas(gcf,[pwd '/Figures/Parametric to Multiplicative Input/Relative Differences Gd.png'])
%}

%% Define the Perturbed Plant and the Disturbance Transfer Matrix
% Multiplicative Input Uncertainty
% Define the Delta matrices with fully individual ultidyn elements

bound_G = 0.3;
Delta_I_G = ultidyn('Delta_I_G',[3,3],'Bound',bound_G);

Gp_app = G*(eye(3) + Delta_I_G*W_I_G_ss);
Gp_app = minreal(Gp_app);

bound_Gd = 0.3;
Delta_I_Gd = ultidyn('Delta_I_Gd',[6,3],'Bound',bound_Gd);

Gd_p_app = Gd*(eye(6) + Delta_I_Gd*W_I_Gd_ss);
Gd_p_app = minreal(Gd_p_app);

% Bode plot
omega = logspace(-3,3,100);
figure
bodeplot(Gp,'r--',Gp_app,'g-.',G,'b-',omega,opts_bode)
title('Plant Bode Plot');
legend('\boldmath{$G_p$}\textbf{-Parametric Uncertainty}',...
       '\boldmath{$G_{p,app}$}\textbf{-Multiplicative Input Uncertainty}',...
       '\boldmath{$G$}\textbf{-Nominal}','interpreter','latex','FontSize',10)
% set(gcf, 'WindowState', 'maximized');
% saveas(gcf,[pwd '/Figures/Parametric to Multiplicative Input/Bode Plot G.png'])

figure
bodeplot(Gd_p,'r--',Gd_p_app,'g-.',Gd,'b-',omega,opts_bode)
title('Disturbance Tranfer Function Matrix Bode Plot');
legend('\boldmath{$G_{d,p}$}\textbf{-Parametric Uncertainty}',...
       '\boldmath{$G_{d,p,app}$}\textbf{-Multiplicative Input Uncertainty}',...
       '\boldmath{$G_d$}\textbf{-Nominal Disturbance Tranfer Matrix}','interpreter','latex','FontSize',10)
% set(gcf, 'WindowState', 'maximized');
% saveas(gcf,[pwd '/Figures/Parametric to Multiplicative Input/Bode Plot Gd.png'])

% Singular Values Plot
figure
sigmaplot(Gp,'r--',Gp_app,'g.',G,'b-',omega,opts_sigma)
title('Plant Singular Values');
legend('\boldmath{$G_p$}\textbf{-Parametric Uncertainty}',...
       '\boldmath{$G_{p,app}$}\textbf{-Multiplicative Input Uncertainty}',...
       '\boldmath{$G$}\textbf{-Nominal}','interpreter','latex','FontSize',10)
% set(gcf, 'WindowState', 'maximized');
% saveas(gcf,[pwd '/Figures/Parametric to Multiplicative Input/Sigma Plot G.png'])

figure
sigmaplot(Gd_p,'r--',Gd_p_app,'g.',Gd,'b-',omega,opts_sigma)
title('Disturbance Tranfer Function Matrix Bode Plot');
legend('\boldmath{$G_{d,p}$}\textbf{-Parametric Uncertainty}',...
       '\boldmath{$G_{d,p,app}$}\textbf{-Multiplicative Input Uncertainty}',...
       '\boldmath{$G_d$}\textbf{-Nominal Disturbance Tranfer Matrix}','interpreter','latex','FontSize',10)
% set(gcf, 'WindowState', 'maximized');
% saveas(gcf,[pwd '/Figures/Parametric to Multiplicative Input/Sigma Plot Gd.png'])
%% Save results
% save('Data Files/tiplicative_Input_Uncertainty.mat')