% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

clc
clear all
close all

load('LTI_Perturbed_Plant.mat','G','Gd','Gp','Gd_p')
% load('Additive_Uncertainty.mat')
%% Calculate weighting functions for additive uncertanties

% Approximate the parametric uncertainties as additive uncertainties
% for each channel of the perturbed plant
omega = logspace(-2,4,200);
samples = 50;

% Approximation of the uncertainty
G_frd = frd(G,omega);
Gp_samples = usample(Gp,samples);
Gp_frd = frd(Gp_samples,omega);

% Gd_frd = frd(Gd,omega);
% Gd_p_samples = usample(Gd_p,samples);
% Gd_p_frd = frd(Gd_p_samples,omega);

W_A_ss = ss([]);

%%
%{
order = 3;
ord1 = [order;order;order];
ord2 = [order;order;order];
[~,Info] = ucover(Gp_frd,G,ord1,ord2,'Additive');
W_1 = Info.W1;
W_2 = Info.W2;
%}
%%
order = 3;
for i=1:size(G,1)
    for j=1:size(G,2)
        [~,Info] = ucover(Gp_frd(i,j,:,:),G(i,j),order,'Additive');
        temp= strcat('w',num2str(i),num2str(j));
        W_A.(temp) = Info.W1;
        W_A_ss(i,j) = Info.W1;
        j
    end
end  
%%
% for i=1:size(Gd,1)
%     for j=1:size(Gd,2)
%         [~,Info] = ucover(Gd_p_frd(i,j),Gd_frd(i,j),order,'OutputMult');
%         W_O_d(i,j) = Info.W1;
%     end
% end  

% W_O_frd = frd(W_O,omega);
% W_I_frd = frd(W_I,omega);
% Gp_frd = frd(Gp_samples,omega);
% W_O_d_frd = frd(W_O_d,omega);

bode_opts = bodeoptions;
bode_opts.MagScale = 'log';
bode_opts.MagUnits = 'abs';
bode_opts.InputLabels.FontSize = 10;
bode_opts.OutputLabels.FontSize = 10;
bode_opts.XLabel.FontSize = 11;
bode_opts.YLabel.FontSize = 11;
bode_opts.TickLabel.FontSize = 10;
bode_opts.Title.FontSize = 12;
bode_opts.PhaseVisible = 'off';

for i=1:size(G,1)
    for j=1:size(G,2)     
        abs_diff(i,j,:,:) = Gp_frd(i,j) - G_frd(i,j);
    end
end 

% for i=1:size(G,1)
%     for j=1:size(G,2)     
%         abs_diff(i,j,:,:) = Gp_frd(i,j) - G_frd(i,j);
%         j
%     end
% end

figure
title('Approximation of parametric uncertainties by additive uncertainties')
bodeplot(abs_diff,'b--',W_A_ss,'r',bode_opts);
grid on
legend('\boldmath{$|(G_p(j\omega)-G(j\omega))|$}','\boldmath{$|W_A(j\omega)|$}',...
       'interpreter','latex','FontSize',12)
% figure
% title('Approximation of parametric uncertainties by multiplicative input uncertainties')
% bodeplot(rel_diff_d,'b--',W_O_d_frd,'r',bode_opts);
% grid on
% legend('\boldmath{$|(G_{d,p}(j\omega)-G_d(j\omega))/G_d(j\omega)|$}',...
%        '\boldmath{$|W_I(j\omega)|$}','interpreter','latex','FontSize',12)

%% Modification of the weighting functions
% The weighting functions obtained from the ucover function are modified in
% order to capture the uncertainty in low and high frequencies properly
s = tf('s');
select = 11;

cor_11 = (8e-3)^2*((1/8e-3)*s + 1)^2/(s^2*((1/1e4)*s + 1)^2);
cor_12 = (1e-2)^2*((1/1e-2)*s + 1)^2/(s^2*((1/1e4)*s + 1)^2);
cor_13 = (1e-2)^2*((1/1e-2)*s + 1)^2/(s^2*((1/1e4)*s + 1)^2);
cor_21 = 2e-3*((1/2e-3)*s + 1)/(s*((1/1e4)*s + 1)^2);
cor_22 = 4e-4*((1/4e-4)*s + 1)/(s*((1/1.5e4)*s + 1)*((1/2e4)*s + 1));
cor_23 = 1e-3*((1/1e-3)*s + 1)/(s*((1/1.5e4)*s + 1)*((1/3e4)*s + 1));
cor_31 = 4e-3*((1/4e-3)*s + 1)/(s*((1/1.5e4)*s + 1)*((1/3e4)*s + 1));
cor_32 = 4e-4*((1/4e-4)*s + 1)/(s*((1/1e4)*s + 1)*((1/1e4)*s + 1));
cor_33 = 2e-4*((1/2e-4)*s + 1)/(s*((1/1e4)*s + 1)*((1/1e4)*s + 1));

% cor_11 = 1/(((1/1e4)*s + 1)^2);
% cor_12 = 1/(((1/1.1e4)*s + 1)^2);
% cor_13 = 1/(((1/1.2e4)*s + 1)^2);
% cor_21 = 1/(((1/1.3e4)*s + 1)^2);
% cor_22 = 1/(((1/1.5e4)*s + 1)*((1/2e4)*s + 1));
% cor_23 = 1/(((1/1.4e4)*s + 1)*((1/4e4)*s + 1));
% cor_31 = 1/(((1/1.35e4)*s + 1)*((1/35e4)*s + 1));
% cor_32 = 1/(((1/1.05e4)*s + 1)*((1/1.05e4)*s + 1));
% cor_33 = 1/(((1/1.15e4)*s + 1)*((1/1.15e4)*s + 1));

% cor_11 = (8e-3)^2*((1/8e-3)*s + 1)^2/(s^2*((1/1e4)*s + 1)^2);
% cor_12 = (1e-2)^2*((1/1e-2)*s + 1)^2/(s^2*((1/1.1e4)*s + 1)^2);
% cor_13 = (1.1e-2)^2*((1/1.1e-2)*s + 1)^2/(s^2*((1/1.2e4)*s + 1)^2);
% cor_21 = 2e-3*((1/2e-3)*s + 1)/(s*((1/1.05e4)*s + 1)^2);
% cor_22 = 4.1e-4*((1/4.1e-4)*s + 1)/(s*((1/1.5e4)*s + 1)*((1/2e4)*s + 1));
% cor_23 = 1e-3*((1/1e-3)*s + 1)/(s*((1/1.6e4)*s + 1)*((1/3.1e4)*s + 1));
% cor_31 = 4e-3*((1/4e-3)*s + 1)/(s*((1/1.7e4)*s + 1)*((1/3e4)*s + 1));
% cor_32 = 4e-4*((1/4e-4)*s + 1)/(s*((1/1.15e4)*s + 1)*((1/1.15e4)*s + 1));
% cor_33 = 2e-4*((1/2e-4)*s + 1)/(s*((1/1.25e4)*s + 1)*((1/.25e4)*s + 1));
%%
run select_case.m
temp= strcat('w',num2str(i),num2str(j));
omega = logspace(-5,6,100);

figure
title('Approximation of parametric uncertainties by additive uncertainties')
bodeplot(abs_diff(i,j),'b--',W_A.(temp)*cor,'r',omega,bode_opts);
grid on
legend('\boldmath{$|(G_p(j\omega)-G(j\omega))|$}','\boldmath{$|W_A(j\omega)|$}',...
       'interpreter','latex','FontSize',12)
%%
W_A_cor.w11 = W_A.w11 * cor_11;
W_A_cor.w12 = W_A.w12 * cor_12;
W_A_cor.w13 = W_A.w13 * cor_13;
W_A_cor.w21 = W_A.w21 * cor_21;
W_A_cor.w22 = W_A.w22 * cor_22;
W_A_cor.w23 = W_A.w23 * cor_23;
W_A_cor.w31 = W_A.w31 * cor_31;
W_A_cor.w32 = W_A.w32 * cor_32;
W_A_cor.w33 = W_A.w33 * cor_33;

%% Additive uncertainty
% Defining the complex scalar uncertainties for each channel of perturbed plant
bound = 1;
% Delta_A = [ultidyn('d11',[1,1],'Bound',bound),...
%            ultidyn('d12',[1,1],'Bound',bound),...
%            ultidyn('d13',[1,1],'Bound',bound);...
%            ultidyn('d21',[1,1],'Bound',bound),...
%            ultidyn('d22',[1,1],'Bound',bound),...
%            ultidyn('d23',[1,1],'Bound',bound);...
%            ultidyn('d31',[1,1],'Bound',bound),...
%            ultidyn('d32',[1,1],'Bound',bound),...
%            ultidyn('d33',[1,1],'Bound',bound)];
d11 = ultidyn('d11',[1,1],'Bound',bound);
d12 = ultidyn('d12',[1,1],'Bound',bound);
d13 = ultidyn('d13',[1,1],'Bound',bound);
d21 = ultidyn('d21',[1,1],'Bound',bound);
d22 = ultidyn('d22',[1,1],'Bound',bound);
d23 = ultidyn('d23',[1,1],'Bound',bound);
d31 = ultidyn('d31',[1,1],'Bound',bound);
d32 = ultidyn('d32',[1,1],'Bound',bound);
d33 = ultidyn('d33',[1,1],'Bound',bound);

% W_A_Delta_mat = [Delta_A(1,1)*W_A_cor.w11,...
%                  Delta_A(1,2)*W_A_cor.w12,...
%                  Delta_A(1,3)*W_A_cor.w13;...
%                  Delta_A(2,1)*W_A_cor.w21,...
%                  Delta_A(2,2)*W_A_cor.w22,...
%                  Delta_A(2,3)*W_A_cor.w23;...
%                  Delta_A(3,1)*W_A_cor.w31,...
%                  Delta_A(3,2)*W_A_cor.w32,...
%                  Delta_A(3,3)*W_A_cor.w33];
W_A_Delta_mat = [W_A_cor.w11*d11,...
                 W_A_cor.w12*d12,...
                 W_A_cor.w13*d13;...
                 W_A_cor.w21*d21,...
                 W_A_cor.w22*d22,...
                 W_A_cor.w23*d23;...
                 W_A_cor.w31*d31,...
                 W_A_cor.w32*d32,...
                 W_A_cor.w33*d33];

% Gp_add = parallel(G,W_A_Delta_mat);
Gp_add = G + W_A_Delta_mat;
Gp_add = minreal(Gp_add);

Controlability=rank(ctrb(Gp_add.A,Gp_add.B));
Observability=rank(obsv(Gp_add.A,Gp_add.C));

%% Perturbed plant for disturbance transfer function Gd
%{
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


% Defining the uncertain transfer matrix
Gp_app = uss([]);
% Gd_p_app = uss([]);
% mult = struct([]);
% clear mult

for i=1:size(G,1)
    for j=1:size(G,2)
        temp_var = strcat('w',num2str(i),num2str(j));
%         Gp_app(i,j) = (1 + W_O.(temp_var)*Delta_O(i,j))*G(i,j);
%         Gp_app(i,j) = (1 + W_O_ss(i,j)*Delta_O(i,j))*G(i,j);
%         mult.(temp_var) = (1 + W_I(i,j)*Delta_I(i,j))*G(i,j);
        Gp_app(i,j) = W_A.(temp_var)*Delta_A(i,j) + G(i,j);
    end
end

Gp_app = minreal(Gp_app);
%}
%%
omega = logspace(-2,3,200);
figure
bodeplot(Gp_add,G,bode_opts)
title('Approximated Perturbed Plant G by Additive Uncertainty');
grid on

%%
sigma_opts = sigmaoptions;
sigma_opts.MagScale = 'log';
sigma_opts.MagUnits = 'abs';
sigma_opts.InputLabels.FontSize = 10;
sigma_opts.OutputLabels.FontSize = 10;
sigma_opts.XLabel.FontSize = 11;
sigma_opts.YLabel.FontSize = 11;
sigma_opts.TickLabel.FontSize = 10;
sigma_opts.Title.FontSize = 12;
sigma_opts.Grid = 'on';

figure
sigma(Gp_add,G,sigma_opts);
legend('\boldmath{$\sigma(Gp_app)$}','\boldmath{$\sigma(G)$}','interpreter','latex','FontSize',15)
%% Save results
save('Additive_Uncertainty2.mat')