% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

clc
clear all
close all

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

[~,~,~,~,Gact,Gact_p] = Design_Weights();

%%
% Approximate the parametric uncertainties as multiplicative uncertainties
% for each channel of the perturbed plant
omega = logspace(-4,4,50);
samples = 30;

% Approximation of the uncertainty
Gact_frd = frd(Gact(1,1),omega);
Gact_samples = usample(Gact_p(1,1),samples);
Gact_p_frd = frd(Gact_samples,omega);
%%
order = 2;
[~,Info] = ucover(Gact_p_frd,Gact(1,1),order,'InputMult');
W_I_Gact_ss = Info.W1;

rel_dif_Gact = (Gact_p_frd - Gact_frd)/Gact_frd;
% abs_dif_Gact = (Gact_p_frd - Gact_frd);
%%
figure
title('Approximation of parametric uncertainties by multiplicative uncertainties for each channel')
bodeplot(rel_dif_Gact,'b-',W_I_Gact_ss,'r--',omega,opts_bode);
legend('\boldmath{$|(G_p(j\omega)-G(j\omega))/G(j\omega)|$}','\boldmath{$|W_I_G(j\omega)|$}',...
       'interpreter','latex','FontSize',12)
% set(gcf, 'WindowState', 'maximized');
% saveas(gcf,[pwd '/Figures/Parametric to Multiplicative Input/Relative Differences G.png'])

%%
bound_Gact = 1;
% Delta_I_Gact = blkdiag(ultidyn('Delta_I_Gact1',[1,1],'Bound',bound_Gact),...
%                        ultidyn('Delta_I_Gact2',[1,1],'Bound',bound_Gact),...
%                        ultidyn('Delta_I_Gact3',[1,1],'Bound',bound_Gact));
Delta_I_Gact = ultidyn('Delta_I_Gact1',[1,1],'Bound',bound_Gact);

Gact_p_app = Gact(1,1)*(1 + Delta_I_Gact*W_I_Gact_ss);
% Gact_p_app = Gact(1,1) + Delta_I_Gact*W_I_Gact_ss;
Gact_p_app = minreal(Gact_p_app);

%%
figure
bodeplot(Gact_p_app,opts_bode)
%%
figure
bodeplot(Gact_p(1,1),opts_bode)
%%
figure
step(usample(Gact_p_app,30),0.6)
ylim([0,1])
grid on
%%
figure
step(usample(Gact_p(1,1),30))
grid on

%% Save results
W_I_Gact_ss = blkdiag(Info.W1,Info.W1,Info.W1);
save('Data Files/Multiplicative_Input_Uncertainty_Act.mat','W_I_Gact_ss')