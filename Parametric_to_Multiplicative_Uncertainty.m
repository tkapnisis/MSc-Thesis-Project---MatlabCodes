% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

clc
clear all
close all

load('LTI_Perturbed_Plant.mat','G','Gd','Gp','Gd_p')

%% Calculate weighting functions for multiplicative uncertanties

% Approximate the parametric uncertainties as multiplicative uncertainties
% for each channel of the perturbed plant
omega = logspace(-3,4,400);
samples = 50;
order = 4;

% Approximation of the uncertainty
Gnom_frd = frd(G,omega);
Gp_samples = usample(Gp,samples);
Gp_frd = frd(Gp_samples,omega);

W_I = ss([]);

for i=1:3
    for j=1:3
        [~,Info] = ucover(Gp_frd(i,j),Gnom_frd(i,j),order,'InputMult');
        W_I(i,j) = Info.W1;
    end
end  
W_I_frd = frd(W_I,omega);

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
bode_opts.PhaseVisible = 'off';

for i=1:3
    for j=1:3
        rel_diff(i,j,:,:) = (Gp_frd(i,j) - Gnom_frd(i,j))/Gnom_frd(i,j);
        rel_diff.u(j) = Gnom_frd.u(j);
        rel_diff.y(i) = Gnom_frd.y(i);
    end
end 

figure
title('Approximation of parametric uncertainties by multiplicative input uncertainties')
bodeplot(rel_diff,'b--',W_I_frd,'r',bode_opts);
grid on
legend('\boldmath{$|(G_p(j\omega)-G(j\omega))/G(j\omega)|$}','\boldmath{$|W_I(j\omega)|$}',...
       'interpreter','latex','Location','best','FontSize',12)

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

% Defining the uncertain transfer matrix
Gp_app = uss([]);
for i=1:3
    for j=1:3
        Gp_app(i,j) = G(i,j)*(1 + W_I(i,j)*Delta_I(i,j));
    end
end

Gp_app = minreal(Gp_app);
Gp_app_samples = usample(Gp_app,samples);

figure
bodeplot(Gp_app_samples,G,omega,bode_opts)
title('Approximated Perturbed Plant by Multiplicative Input Uncertainty');
grid on

figure
bodeplot(Gp_app_samples,G,omega,bode_opts)
title('Approximated Perturbed Plant by Multiplicative Input Uncertainty');
grid on

%% 
