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
omega = logspace(-3,3,200);
samples = 50;
order = 2;

% Approximation of the uncertainty
G_frd = frd(G,omega);
% Gd_frd = frd(Gd,omega);

Gp_samples = usample(Gp,samples);
Gp_frd = ufrd(Gp_samples,omega);

% Gd_p_samples = usample(Gd_p,samples);
% Gd_p_frd = frd(Gd_p_samples,omega);

% W_O = ss([]);
% W_I = ss([]);
% Gp_ap = uss([]);
% W_O = struct('w11',{},'w12',{},'w13',{},'w21',{},'w22',{},'w23',{},'w31',...
%              {},'w32',{},'w33',{});
% W_O_d = ss([]);


for i=1:size(G,1)
    for j=1:size(G,2)
%         [~,Info] = ucover(Gp_frd(i,j),G(i,j),order,order,'Additive');
        [~,Info] = ucover(Gp_frd(i,j),G(i,j),order,'OutputMult');
%         temp = strcat('d',num2str(i),num2str(j));
        temp= strcat('w',num2str(i),num2str(j));
        W_O.(temp) = Info.W1;
%         Gp_a.Uncertainty.InputMultDelta.Name = temp;
%         W_I(i,j) = Info.W1;
%         Gp_ap(i,j) = Gp_a;
        j
    end
end  
% Gp_ap = minreal(Gp_ap);
%%
% for i=1:size(Gd,1)
%     for j=1:size(Gd,2)
%         [~,Info] = ucover(Gd_p_frd(i,j),Gd_frd(i,j),order,'OutputMult');
%         W_O_d(i,j) = Info.W1;
%     end
% end  

% W_O_frd = frd(W_O,omega);
W_I_frd = frd(W_I,omega);
Gp_frd = frd(Gp_samples,omega);
% W_O_d_frd = frd(W_O_d,omega);

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

for i=1:size(G,1)
    for j=1:size(G,2)
        rel_diff(i,j,:,:) = (Gp_frd(i,j) - G_frd(i,j))/G_frd(i,j);
%         rel_diff(i,j,:,:) = (Gp_frd(i,j) - G(i,j))/G(i,j);        
%         rel_diff(i,j,:,:) = Gp_frd(i,j) - G_frd(i,j);
%         rel_diff.u(j) = G.u(j);
%         rel_diff.y(i) = G.y(i);
        j
    end
end 

% for i=1:size(Gd,1)
%     for j=1:size(Gd,2)
%         rel_diff_d(i,j,:,:) = (Gd_p_frd(i,j) - Gd_frd(i,j))/Gd_frd(i,j);
%         rel_diff_d.u(j) = Gd_frd.u(j);
%         rel_diff_d.y(i) = Gd_frd.y(i);
%     end
% end 

figure
title('Approximation of parametric uncertainties by multiplicative input uncertainties')
bodeplot(rel_diff,'b--',W_I_frd,'r',bode_opts);
grid on
legend('\boldmath{$|(G_p(j\omega)-G(j\omega))/G(j\omega)|$}','\boldmath{$|W_I(j\omega)|$}',...
       'interpreter','latex','FontSize',12)

% figure
% title('Approximation of parametric uncertainties by multiplicative input uncertainties')
% bodeplot(rel_diff_d,'b--',W_O_d_frd,'r',bode_opts);
% grid on
% legend('\boldmath{$|(G_{d,p}(j\omega)-G_d(j\omega))/G_d(j\omega)|$}',...
%        '\boldmath{$|W_I(j\omega)|$}','interpreter','latex','FontSize',12)
%%
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
%%
% Delta_O_d = [ultidyn('do11',[1,1],'Bound',bound),...
%              ultidyn('do12',[1,1],'Bound',bound),...
%              ultidyn('do13',[1,1],'Bound',bound),...
%              ultidyn('do14',[1,1],'Bound',bound),...
%              ultidyn('do15',[1,1],'Bound',bound),...
%              ultidyn('do16',[1,1],'Bound',bound);...
%              ultidyn('do21',[1,1],'Bound',bound),...
%              ultidyn('do22',[1,1],'Bound',bound),...
%              ultidyn('do23',[1,1],'Bound',bound),...
%              ultidyn('do24',[1,1],'Bound',bound),...
%              ultidyn('do25',[1,1],'Bound',bound),...
%              ultidyn('do26',[1,1],'Bound',bound);...
%              ultidyn('do31',[1,1],'Bound',bound),...
%              ultidyn('do32',[1,1],'Bound',bound),...
%              ultidyn('do33',[1,1],'Bound',bound),...
%              ultidyn('do34',[1,1],'Bound',bound),...
%              ultidyn('do35',[1,1],'Bound',bound),...
%              ultidyn('do36',[1,1],'Bound',bound)];

%%
Delta_I = ultidyn('dI',[3,3],'Bound',1,'SampleStateDimension',1,...
                  'SampleMaxFrequency',1e2);
%%
W_I = Info.W1;

Gp_app = (eye(3) + W_I*Delta_I)*G;
%%

% Defining the uncertain transfer matrix
Gp_app = uss([]);
% Gd_p_app = uss([]);
% mult = struct([]);
% clear mult

for i=1:size(G,1)
    for j=1:size(G,2)
        temp_var = strcat('w',num2str(i),num2str(j));
%         Gp_app(i,j) = (1 + W_O_st.(temp_var)*Delta_I(i,j))*G(i,j);
        Gp_app(i,j) = (1 + W_I(i,j)*Delta_I(i,j))*G(i,j);
%         mult.(temp_var) = (1 + W_I(i,j)*Delta_I(i,j))*G(i,j);
%         Gp_app(i,j) = W_A(i,j)*Delta_A(i,j) + G(i,j);
    end
end

Gp_app = minreal(Gp_app);

%%
res = [mult.w11, mult.w12, mult.w13;...
       mult.w21, mult.w22, mult.w23;...
       mult.w31, mult.w22, mult.w33];
res = minreal(res,1e-4);
%%
% for i=1:size(Gd,1)ans
%     for j=1:size(Gd,2)
%         Gd_p_app(i,j) = (1 + W_O_d(i,j)*Delta_O_d(i,j))*Gd(i,j);
%     end
% end

Gp_app_samples = usample(Gp_app,samples);
Gp_ap_samples = usample(Gp_ap,samples);
%%
% Gd_p_app = minreal(Gd_p_app);
% Gd_p_app_samples = usample(Gd_p_app,samples);

figure
bodeplot(Gp_app_samples,G,omega,bode_opts)
title('Approximated Perturbed Plant by Multiplicative Input Uncertainty');
grid on

% figure
% bodeplot(Gd_p_app_samples,Gd,omega,bode_opts)
% title('Approximated Perturbed Plant by Multiplicative Input Uncertainty');
% grid on

figure
bodeplot(Gp_samples,G,omega,bode_opts)
title('Actual Perturbed Plant with Parametric Uncertainty');
grid on

% figure
% bodeplot(Gd_p_samples,Gd,omega,bode_opts)
% title('Actual Perturbed Plant with Parametric Uncertainty');
% grid on

figure
bodeplot(Gp_ap_samples,G,omega,bode_opts)
title('NEW Approximated Perturbed Plant by Multiplicative Input Uncertainty');
grid on
%% 
% W_O = W_I;
% save('Uncertainty_Approximation.mat')
% save('Uncertainty_Approximation2.mat')
% save('Uncertainty_Approximation5.mat','W_A','omega')
save('Uncertainty_Approximation_OutMult.mat','W_O')