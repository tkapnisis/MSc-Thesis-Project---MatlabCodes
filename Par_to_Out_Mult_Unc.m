% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

clc
clear all
close all

load('LTI_Perturbed_Plant.mat','G','Gd','Gp','Gd_p')

% load('Multiplicative_Uncertainty.mat')

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

%% Calculate weighting functions for multiplicative uncertanties

% Approximate the parametric uncertainties as multiplicative uncertainties
% for each channel of the perturbed plant
omega = logspace(-4,4,200);
samples = 50;

% Approximation of the uncertainty
G_frd = frd(G,omega);
Gp_samples = usample(Gp,samples);
Gp_frd = frd(Gp_samples,omega);

Gd_frd = frd(Gd,omega);
Gd_p_samples = usample(Gd_p,samples);
Gd_p_frd = frd(Gd_p_samples,omega);

W_O_G_ss = ss([]);
W_O_Gd_ss = ss([]);

order = 3;
for i=1:size(G,1)
    for j=1:size(G,2)
        [~,Info] = ucover(Gp_frd(i,j,:,:),G(i,j),order,'OutputMult');
        temp= strcat('w',num2str(i),num2str(j));
        W_O_G.(temp) = Info.W1;
        W_O_G_ss(i,j) = Info.W1;
        rel_dif_G(i,j,:,:) = (Gp_frd(i,j) - G_frd(i,j))/G_frd(i,j);
        rel_dif_G.u(j) = G.u(j);
        rel_dif_G.y(i) = G.y(i);
        j
    end
end  

for i=1:size(Gd,1)
    for j=1:size(Gd,2)
        [~,Info] = ucover(Gd_p_frd(i,j,:,:),Gd(i,j),order,'OutputMult');
        temp= strcat('w',num2str(i),num2str(j));
        W_O_Gd.(temp) = Info.W1;
        W_O_Gd_ss(i,j) = Info.W1;
        rel_dif_Gd(i,j,:,:) = (Gd_p_frd(i,j) - Gd_frd(i,j))/Gd_frd(i,j);
        rel_dif_Gd.u(j) = Gd_frd.u(j);
        rel_dif_Gd.y(i) = Gd_frd.y(i);
        j
    end
end  

figure
title('Approximation of parametric uncertainties by multiplicative uncertainties for each channel')
bodeplot(rel_dif_G,'b--',W_O_G_ss,'r',omega,opts_bode);
legend('\boldmath{$|(G_p(j\omega)-G(j\omega))/G(j\omega)|$}','\boldmath{$|W_O_G(j\omega)|$}',...
       'interpreter','latex','FontSize',12)
set(gcf, 'WindowState', 'maximized');
saveas(gcf,[pwd '/Figures/Parametric to Multiplicative/Relative Difference G.png'])

figure
title('Approximation of parametric uncertainties by multiplicative input uncertainties')
bodeplot(rel_dif_Gd,'b--',W_O_Gd_ss,'r',omega,opts_bode);
grid on
legend('\boldmath{$|(G_{d,p}(j\omega)-G_d(j\omega))/G_d(j\omega)|$}',...
       '\boldmath{$|W_O_Gd(j\omega)|$}','interpreter','latex','FontSize',12)
set(gcf, 'WindowState', 'maximized');
saveas(gcf,[pwd '/Figures/Parametric to Multiplicative/Relative Difference Gd.png'])
%% Define the Perturbed Plant and the Disturbance Transfer Matrix
% Output Multiplicative Uncertainty
bound_G = 0.4;
Delta_O_G = ultidyn('Delta_O_G',[3,3],'Bound',bound_G);
Gp_app = (eye(3) + W_O_G_ss*Delta_O_G)*G;
Gp_app = minreal(Gp_app);

bound_Gd = 0.4;
Delta_O_Gd = ultidyn('Delta_O_Gd',[6,3],'Bound',bound_Gd);
Gd_p_app = (eye(3) + W_O_Gd_ss*Delta_O_Gd)*Gd;
Gd_p_app = minreal(Gd_p_app);

%% Bode plot
omega = logspace(-3,4,100);
figure
bodeplot(Gp,'r--',Gp_app,'g-.',G,'b-',omega,opts_bode)
title('Plant Bode Plot');
legend('\boldmath{$G_p$}\textbf{-Parametric Uncertainty}',...
       '\boldmath{$G_{p,app}$}\textbf{-Multiplicative Output Uncertainty}',...
       '\boldmath{$G$}\textbf{-Nominal}','interpreter','latex','FontSize',10)

figure
bodeplot(Gd_p,'r--',Gd_p_app,'g-.',Gd,'b-',omega,opts_bode)
title('Disturbance Tranfer Function Matrix Bode Plot');
legend('\boldmath{$G_{d,p}$}\textbf{-Parametric Uncertainty}',...
       '\boldmath{$G_{d,p,app}$}\textbf{-Multiplicative Output Uncertainty}',...
       '\boldmath{$G_d$}\textbf{-Nominal Disturbance Tranfer Matrix}','interpreter','latex','FontSize',10)

%% Singular Values Plot
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
% opts_sigma.XLimMode = 'manual';
% opts_sigma.XLim = [1e-3,1e3];
% opts_sigma.YLimMode = 'manual';
% opts_sigma.YLim = [1e-6,1e8];

figure
sigmaplot(Gp,'r--',Gp_app,'g.',G,'b-',omega,opts_sigma)
title('Plant Singular Values');
legend('\boldmath{$G_p$}\textbf{-Parametric Uncertainty}',...
       '\boldmath{$G_{p,app}$}\textbf{-Multiplicative Output Uncertainty}',...
       '\boldmath{$G$}\textbf{-Nominal}','interpreter','latex','FontSize',10)

figure
sigmaplot(Gd_p,'r--',Gd_p_app,'g.',Gd,'b-',omega,opts_sigma)
title('Disturbance Tranfer Function Matrix Bode Plot');
legend('\boldmath{$G_{d,p}$}\textbf{-Parametric Uncertainty}',...
       '\boldmath{$G_{d,p,app}$}\textbf{-Multiplicative Output Uncertainty}',...
       '\boldmath{$G_d$}\textbf{-Nominal Disturbance Tranfer Matrix}','interpreter','latex','FontSize',10)

%% Save results
save('Multiplicative_Uncertainty.mat')