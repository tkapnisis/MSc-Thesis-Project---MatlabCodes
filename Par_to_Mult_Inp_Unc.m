% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

clc
clear all
close all

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
% opts_sigma.XLimMode = 'manual';
% opts_sigma.XLim = [1e-3,1e3];
% opts_sigma.YLimMode = 'manual';
% opts_sigma.YLim = [1e-6,1e8];

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
%
clear bound_G 
bound_G.delta_G11 = 0.2;
bound_G.delta_G12 = 0.2;
bound_G.delta_G13 = 0.2;
bound_G.delta_G21 = 0.2;
bound_G.delta_G22 = 0.2;
bound_G.delta_G23 = 0.2;
bound_G.delta_G31 = 0.2;
bound_G.delta_G32 = 0.2;
bound_G.delta_G33 = 0.2;

Delta_I_G_el.delta_G11 = ultidyn('delta_G11',[1,1],'Bound',bound_G.delta_G11);
Delta_I_G_el.delta_G12 = ultidyn('delta_G12',[1,1],'Bound',bound_G.delta_G12);
Delta_I_G_el.delta_G13 = ultidyn('delta_G13',[1,1],'Bound',bound_G.delta_G13);
Delta_I_G_el.delta_G21 = ultidyn('delta_G21',[1,1],'Bound',bound_G.delta_G21);
Delta_I_G_el.delta_G22 = ultidyn('delta_G22',[1,1],'Bound',bound_G.delta_G22);
Delta_I_G_el.delta_G23 = ultidyn('delta_G23',[1,1],'Bound',bound_G.delta_G23);
Delta_I_G_el.delta_G31 = ultidyn('delta_G31',[1,1],'Bound',bound_G.delta_G31);
Delta_I_G_el.delta_G32 = ultidyn('delta_G32',[1,1],'Bound',bound_G.delta_G32);
Delta_I_G_el.delta_G33 = ultidyn('delta_G33',[1,1],'Bound',bound_G.delta_G33);

Delta_I_G = [Delta_I_G_el.delta_G11,Delta_I_G_el.delta_G12,Delta_I_G_el.delta_G13;...
             Delta_I_G_el.delta_G21,Delta_I_G_el.delta_G22,Delta_I_G_el.delta_G23;
             Delta_I_G_el.delta_G31,Delta_I_G_el.delta_G32,Delta_I_G_el.delta_G33];

clear bound_Gd 
bound_Gd.delta_Gd11 = 0.2;
bound_Gd.delta_Gd12 = 0.2;
bound_Gd.delta_Gd13 = 0.2;
bound_Gd.delta_Gd14 = 0.2;
bound_Gd.delta_Gd15 = 0.2;
bound_Gd.delta_Gd16 = 0.2;
bound_Gd.delta_Gd21 = 0.2;
bound_Gd.delta_Gd22 = 0.2;
bound_Gd.delta_Gd23 = 0.2;
bound_Gd.delta_Gd24 = 0.2;
bound_Gd.delta_Gd25 = 0.2;
bound_Gd.delta_Gd26 = 0.2;
bound_Gd.delta_Gd31 = 0.2;
bound_Gd.delta_Gd32 = 0.2;
bound_Gd.delta_Gd33 = 0.2;
bound_Gd.delta_Gd34 = 0.2;
bound_Gd.delta_Gd35 = 0.2;
bound_Gd.delta_Gd36 = 0.2;

Delta_I_Gd_el.delta_Gd11 = ultidyn('delta_Gd11',[1,1],'Bound',bound_Gd.delta_Gd11);
Delta_I_Gd_el.delta_Gd12 = ultidyn('delta_Gd12',[1,1],'Bound',bound_Gd.delta_Gd12);
Delta_I_Gd_el.delta_Gd13 = ultidyn('delta_Gd13',[1,1],'Bound',bound_Gd.delta_Gd13);
Delta_I_Gd_el.delta_Gd14 = ultidyn('delta_Gd14',[1,1],'Bound',bound_Gd.delta_Gd14);
Delta_I_Gd_el.delta_Gd15 = ultidyn('delta_Gd15',[1,1],'Bound',bound_Gd.delta_Gd15);
Delta_I_Gd_el.delta_Gd16 = ultidyn('delta_Gd16',[1,1],'Bound',bound_Gd.delta_Gd16);
Delta_I_Gd_el.delta_Gd21 = ultidyn('delta_Gd21',[1,1],'Bound',bound_Gd.delta_Gd21);
Delta_I_Gd_el.delta_Gd22 = ultidyn('delta_Gd22',[1,1],'Bound',bound_Gd.delta_Gd22);
Delta_I_Gd_el.delta_Gd23 = ultidyn('delta_Gd23',[1,1],'Bound',bound_Gd.delta_Gd23);
Delta_I_Gd_el.delta_Gd24 = ultidyn('delta_Gd24',[1,1],'Bound',bound_Gd.delta_Gd24);
Delta_I_Gd_el.delta_Gd25 = ultidyn('delta_Gd25',[1,1],'Bound',bound_Gd.delta_Gd25);
Delta_I_Gd_el.delta_Gd26 = ultidyn('delta_Gd26',[1,1],'Bound',bound_Gd.delta_Gd26);
Delta_I_Gd_el.delta_Gd31 = ultidyn('delta_Gd31',[1,1],'Bound',bound_Gd.delta_Gd31);
Delta_I_Gd_el.delta_Gd32 = ultidyn('delta_Gd32',[1,1],'Bound',bound_Gd.delta_Gd32);
Delta_I_Gd_el.delta_Gd33 = ultidyn('delta_Gd33',[1,1],'Bound',bound_Gd.delta_Gd33);
Delta_I_Gd_el.delta_Gd34 = ultidyn('delta_Gd34',[1,1],'Bound',bound_Gd.delta_Gd34);
Delta_I_Gd_el.delta_Gd35 = ultidyn('delta_Gd35',[1,1],'Bound',bound_Gd.delta_Gd35);
Delta_I_Gd_el.delta_Gd36 = ultidyn('delta_Gd36',[1,1],'Bound',bound_Gd.delta_Gd36);

Delta_I_Gd = [Delta_I_Gd_el.delta_Gd11,Delta_I_Gd_el.delta_Gd21,Delta_I_Gd_el.delta_Gd31;...
              Delta_I_Gd_el.delta_Gd12,Delta_I_Gd_el.delta_Gd22,Delta_I_Gd_el.delta_Gd32;...
              Delta_I_Gd_el.delta_Gd13,Delta_I_Gd_el.delta_Gd23,Delta_I_Gd_el.delta_Gd33;...
              Delta_I_Gd_el.delta_Gd14,Delta_I_Gd_el.delta_Gd24,Delta_I_Gd_el.delta_Gd34;...
              Delta_I_Gd_el.delta_Gd15,Delta_I_Gd_el.delta_Gd25,Delta_I_Gd_el.delta_Gd35;...
              Delta_I_Gd_el.delta_Gd16,Delta_I_Gd_el.delta_Gd26,Delta_I_Gd_el.delta_Gd36];
              
%}
%%
bound_G = 0.4;
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
save('Multiplicative_Input_Uncertainty.mat')