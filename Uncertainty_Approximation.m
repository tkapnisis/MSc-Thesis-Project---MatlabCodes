% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

clc
clear all
close all

addpath('Plotting Functions')
addpath('Data Files')

load('LTI_Perturbed_Plant.mat','Gp','Gd_p','Gsm_p','g_sm_p_i')
load('LTI_Nominal_Plant.mat','G','Gd','Gsm','g_sm_i','foil_loc')

% load('Uncertainty_Weighting_Functions.mat')

run Bode_options.m
run Sigma_options.m
%% Calculate weighting functions for multiplicative input uncertanties
%
% Approximate the parametric uncertainties as multiplicative uncertainties
% for each channel of the perturbed plant
omega = logspace(-3,4,200);
samples = 100;

% Plant model
G_frd = frd(G,omega);
Gp_samples = usample(Gp,samples);
Gp_frd = frd(Gp_samples,omega);
disp('Finish Gp_samples')
disp('-----------------')

% Disturbance model
Gd_frd = frd(Gd,omega);
Gd_p_samples = usample(Gd_p,samples);
Gd_p_frd = frd(Gd_p_samples,omega);
disp('Finish Gd_p_samples')
disp('-----------------')

% Actuator model
g_sm_frd = frd(g_sm_i,omega);
g_sm_samples = usample(g_sm_p_i,samples);
g_sm_p_frd = frd(g_sm_samples,omega);

W_I_G = ss([]);
W_I_Gd = ss([]);

order = 3;
for i=1:size(G,1)
    for j=1:size(G,2)
        [~,Info] = ucover(Gp_frd(i,j,:,:),G(i,j),order,'InputMult');
        W_I_G(i,j) = Info.W1;
        rel_dif_G(i,j,:,:) = (Gp_frd(i,j) - G_frd(i,j))/G_frd(i,j);
        rel_dif_G.u(j) = G.u(j);
        rel_dif_G.y(i) = G.y(i);
    end
end  
disp('Finish ucover Gp')
disp('-----------------')

for i=1:size(Gd,1)
    for j=1:size(Gd,2)
        [~,Info] = ucover(Gd_p_frd(i,j,:,:),Gd(i,j),order,'InputMult');
        W_I_Gd(i,j) = Info.W1;
        rel_dif_Gd(i,j,:,:) = (Gd_p_frd(i,j) - Gd_frd(i,j))/Gd_frd(i,j);
        rel_dif_Gd.u(j) = Gd_frd.u(j);
        rel_dif_Gd.y(i) = Gd_frd.y(i);
    end
end  
disp('Finish ucover Gd_p')
disp('-----------------')

order = 1;
[~,Info] = ucover(g_sm_p_frd,g_sm_i,order,'OutputMult');
w_I_g_sm = Info.W1;
rel_dif_g_sm = (g_sm_p_frd - g_sm_frd)/g_sm_frd;
disp('Finish ucover g_sm_i')
disp('-----------------')
%%
figure
title('Relative error for each channel for the plant model')
bodeplot(rel_dif_G,'b--',W_I_G,'r',omega,opts_bode);
legend('\boldmath{$|\frac{g_{ij,p}(j\omega)-g_{ij}(j\omega)}{g_{ij}(j\omega)}|$}','\boldmath{$|w_{I,ij,G}(j\omega)|$}',...
       'interpreter','latex','FontSize',13)
% set(gcf, 'WindowState', 'maximized');
% saveas(gcf,[pwd '/Figures/Parametric to Multiplicative Input/Relative Differences G.png'])

figure
title('Relative error for each channel for the disturbance model')
bodeplot(rel_dif_Gd,'b--',W_I_Gd,'r',omega,opts_bode);
grid on
legend('\boldmath{$|\frac{g_{d,ij,p}(j\omega)-g_{d,ij}(j\omega)}{g_{d,ij}(j\omega)}|$}','\boldmath{$|w_{I,ij,G_d}(j\omega)|$}',...
       'interpreter','latex','FontSize',13)
% set(gcf, 'WindowState', 'maximized');
% saveas(gcf,[pwd '/Figures/Parametric to Multiplicative Input/Relative Differences Gd.png'])

figure
title('Relative error for the actuator model')
bodeplot(rel_dif_g_sm,'b--',w_I_g_sm,'r',omega,opts_bode);
legend('\boldmath{$|\frac{g_{sm,p}^i(j\omega)-g_{sm}^i(j\omega)}{g_{sm}^i(j\omega)}|$}','\boldmath{$|w_{I,g_{sm}^i}(j\omega)|$}',...
       'interpreter','latex','FontSize',18)
% set(gcf, 'WindowState', 'maximized');
% saveas(gcf,[pwd '/Figures/Parametric to Multiplicative Input/Relative Differences G.png'])

%% Define the Perturbed Plant and the Disturbance Transfer Matrix
% Multiplicative Input Uncertainty
% Define the Delta matrices with fully individual ultidyn elements

bound_G = 0.5;
Delta_I_G = ultidyn('Delta_I_G',[3,3],'Bound',bound_G);

Gp_app = G*(eye(3) + Delta_I_G*W_I_G);
Gp_app = minreal(Gp_app);

bound_Gd = 0.5;
Delta_I_Gd = ultidyn('Delta_I_Gd',[6,3],'Bound',bound_Gd);

Gd_p_app = Gd*(eye(6) + Delta_I_Gd*W_I_Gd);
Gd_p_app = minreal(Gd_p_app);

W_I_Gsm = blkdiag(w_I_g_sm,w_I_g_sm,w_I_g_sm);

Delta_I_Gsm = blkdiag(ultidyn('delta_I_g_sm_f',[1,1],'Bound',1),...
                      ultidyn('delta_I_g_sm_f',[1,1],'Bound',1),...
                      ultidyn('delta_I_g_sm_f',[1,1],'Bound',1));

Gsm_p_app = Gsm*(eye(3) + Delta_I_Gsm*W_I_Gsm);
Gsm_p_app = minreal(Gsm_p_app);


%%
%
% Bode plot
omega = logspace(-3,3,100);
figure
bodeplot(Gp,'r--',Gp_app,'g-.',G,'b-',omega,opts_bode)
title('Plant Model Bode Plot');
legend('\boldmath{$G_p$}\textbf{-Parametric Uncertainty}',...
       '\boldmath{$G_{p}^a$}\textbf{-Multiplicative Input Uncertainty}',...
       '\boldmath{$G$}\textbf{-Nominal}','interpreter','latex','FontSize',10)
% set(gcf, 'WindowState', 'maximized');
% saveas(gcf,[pwd '/Figures/Parametric to Multiplicative Input/Bode Plot G.png'])

figure
bodeplot(Gd_p,'r--',Gd_p_app,'g-.',Gd,'b-',omega,opts_bode)
title('Disturbance Model Bode Plot');
legend('\boldmath{$G_{d,p}$}\textbf{-Parametric Uncertainty}',...
       '\boldmath{$G_{d,p}^a$}\textbf{-Multiplicative Input Uncertainty}',...
       '\boldmath{$G_d$}\textbf{-Nominal}','interpreter','latex','FontSize',10)
% set(gcf, 'WindowState', 'maximized');
% saveas(gcf,[pwd '/Figures/Parametric to Multiplicative Input/Bode Plot Gd.png'])

figure
bodeplot(Gsm_p,'r--',Gsm_p_app,'g-.',Gsm,'b-',omega,opts_bode)
title('Actuator Model Bode Plot');
legend('\boldmath{$G_{sm,p}$}\textbf{-Parametric Uncertainty}',...
       '\boldmath{$G_{sm,p}^a$}\textbf{-Multiplicative Input Uncertainty}',...
       '\boldmath{$G_d$}\textbf{-Nominal}','interpreter','latex','FontSize',10)

%%
% Singular Values Plot
figure
sigmaplot(Gp,'r--',Gp_app,'g-.',G,'b-',omega,opts_sigma)
title('Plant Model Singular Values');
legend('\boldmath{$G_p$}\textbf{-Parametric Uncertainty}',...
       '\boldmath{$G_{p}^a$}\textbf{-Multiplicative Input Uncertainty}',...
       '\boldmath{$G$}\textbf{-Nominal}','interpreter','latex','FontSize',10)
% set(gcf, 'WindowState', 'maximized');
% saveas(gcf,[pwd '/Figures/Parametric to Multiplicative Input/Sigma Plot G.png'])

figure
sigmaplot(Gd_p,'r--',Gd_p_app,'g-.',Gd,'b-',omega,opts_sigma)
title('Disturbance Model Singular Values');
legend('\boldmath{$G_{d,p}$}\textbf{-Parametric Uncertainty}',...
       '\boldmath{$G_{d,p}^a$}\textbf{-Multiplicative Input Uncertainty}',...
       '\boldmath{$G_d$}\textbf{-Nominal}','interpreter','latex','FontSize',10)
% set(gcf, 'WindowState', 'maximized');
% saveas(gcf,[pwd '/Figures/Parametric to Multiplicative Input/Sigma Plot Gd.png'])
%%
omega = logspace(-1,3,100);
figure
sigmaplot(Gsm_p,'r--',Gsm_p_app,'g-.',Gsm,'b-',omega,opts_sigma)
title('Actuator Model Singular Values');
legend('\boldmath{$G_{sm,p}$}\textbf{-Parametric Uncertainty}',...
       '\boldmath{$G_{sm,p}^a$}\textbf{-Multiplicative Input Uncertainty}',...
       '\boldmath{$G_{sm}$}\textbf{-Nominal}','interpreter','latex','FontSize',11,...
       'location','best')
%}
%% Save results
save('Data Files/Uncertainty_Weighting_Functions.mat','W_I_G','W_I_Gd','W_I_Gsm',...
     'rel_dif_G','rel_dif_Gd','rel_dif_g_sm','omega','samples','Gp_app','Gd_p_app',...
     'Gsm_p_app')