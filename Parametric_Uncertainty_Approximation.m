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
omega = logspace(-2,4,300);
samples = 40;


% Approximation of the uncertainty
G_frd = frd(G,omega);
Gp_samples = usample(Gp,samples);
Gp_frd = ufrd(Gp_samples,omega);

% Gd_frd = frd(Gd,omega);
% Gd_p_samples = usample(Gd_p,samples);
% Gd_p_frd = frd(Gd_p_samples,omega);

% W_A_ss = ss([]);
W_I_ss = ss([]);
%%
order = 3;
for i=1:size(G,1)
    for j=1:size(G,2)
%         [~,Info] = ucover(Gp_frd(i,j),G(i,j),order,order,'Additive');
        [~,Info] = ucover(Gp_frd(i,j),G(i,j),order,'InputMult');
        temp= strcat('w',num2str(i),num2str(j));
%         W_A.(temp) = Info.W1;
%         W_A_ss(i,j) = Info.W1;
        W_I.(temp) = Info.W1;
        W_I_ss(i,j) = Info.W1;
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
bode_opts.InputLabels.Interpreter = 'none';
bode_opts.InputLabels.FontSize = 10;
bode_opts.OutputLabels.FontSize = 10;
bode_opts.XLabel.FontSize = 11;
bode_opts.YLabel.FontSize = 11;
bode_opts.TickLabel.FontSize = 10;
bode_opts.Title.FontSize = 12;
bode_opts.PhaseVisible = 'off';

Gp_frd = frd(Gp_samples,omega);
clear rel_diff
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

% figure
% title('Approximation of parametric uncertainties by additive uncertainties')
% bodeplot(rel_diff,'b--',W_A_ss,'r',omega,bode_opts);
% grid on
% legend('\boldmath{$|(G_p(j\omega)-G(j\omega))|$}','\boldmath{$|W_A(j\omega)|$}',...
%        'interpreter','latex','FontSize',12)

figure
title('Approximation of parametric uncertainties by multiplicative uncertainties')
bodeplot(rel_diff,'b--',W_I_ss,'r',omega,bode_opts);
grid on
legend('\boldmath{$|(G_p(j\omega)-G(j\omega))/G(j\omega)|$}','\boldmath{$|W_A(j\omega)|$}',...
       'interpreter','latex','FontSize',12)

% figure
% title('Approximation of parametric uncertainties by multiplicative input uncertainties')
% bodeplot(rel_diff_d,'b--',W_O_d_frd,'r',bode_opts);
% grid on
% legend('\boldmath{$|(G_{d,p}(j\omega)-G_d(j\omega))/G_d(j\omega)|$}',...
%        '\boldmath{$|W_I(j\omega)|$}','interpreter','latex','FontSize',12)
%% Additive uncertainty
% Defining the complex scalar uncertainties for each channel of perturbed plant
% bound = 1;
% Delta_A = [ultidyn('d11',[1,1],'Bound',bound),...
%            ultidyn('d12',[1,1],'Bound',bound),...
%            ultidyn('d13',[1,1],'Bound',bound);...
%            ultidyn('d21',[1,1],'Bound',bound),...
%            ultidyn('d22',[1,1],'Bound',bound),...
%            ultidyn('d23',[1,1],'Bound',bound);...
%            ultidyn('d31',[1,1],'Bound',bound),...
%            ultidyn('d32',[1,1],'Bound',bound),...
%            ultidyn('d33',[1,1],'Bound',bound)];
% 
% W_A_Delta_mat = [W_A.w11*Delta_A(1,1),...
%                  W_A.w12*Delta_A(1,2),...
%                  W_A.w13*Delta_A(1,3);...
%                  W_A.w21*Delta_A(2,1),...
%                  W_A.w22*Delta_A(2,2),...
%                  W_A.w23*Delta_A(2,3);...
%                  W_A.w31*Delta_A(3,1),...
%                  W_A.w32*Delta_A(3,2),...
%                  W_A.w33*Delta_A(3,3)];

% Gp_add = parallel(G,W_A_Delta_mat);
% 
% Gp_add = minreal(Gp_add);
% Controlability=rank(ctrb(Gp_add.A,Gp_add.B));
% Observability=rank(obsv(Gp_add.A,Gp_add.C));

%% Multiplicative uncertainty
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

% Gp_inp_mult = [  (W_I.w11*Delta_I(1,1)+1)*G(1,1),...
%                  (W_I.w12*Delta_I(1,2)+1)*G(1,2),...
%                  (W_I.w13*Delta_I(1,3)+1)*G(1,3);...
%                  (W_I.w21*Delta_I(2,1)+1)*G(2,1),...
%                  (W_I.w22*Delta_I(2,2)+1)*G(2,2),...
%                  (W_I.w23*Delta_I(2,3)+1)*G(2,3);...
%                  (W_I.w31*Delta_I(3,1)+1)*G(3,1),...
%                  (W_I.w32*Delta_I(3,2)+1)*G(3,2),...
%                  (W_I.w33*Delta_I(3,3)+1)*G(3,3)];
% Gp_inp_mult = minreal(Gp_inp_mult);
% Controlability=rank(ctrb(Gp_inp_mult.A,Gp_inp_mult.B));
% Observability=rank(obsv(Gp_inp_mult.A,Gp_inp_mult.C));

W_I_mat = [  W_I.w11*Delta_I(1,1),...
                 W_I.w12*Delta_I(1,2),...
                 W_I.w13*Delta_I(1,3);...
                 W_I.w21*Delta_I(2,1),...
                 W_I.w22*Delta_I(2,2),...
                 W_I.w23*Delta_I(2,3);...
                 W_I.w31*Delta_I(3,1),...
                 W_I.w32*Delta_I(3,2),...
                 W_I.w33*Delta_I(3,3)];

sys1 = parallel(ss(eye(3)),W_I_mat);
Gp_inp_mult = series(sys1,G);
Gp_inp_mult = minreal(Gp_inp_mult);
Controlability=rank(ctrb(Gp_inp_mult.A,Gp_inp_mult.B));
Observability=rank(obsv(Gp_inp_mult.A,Gp_inp_mult.C));
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
bodeplot(Gp_inp_mult,G,bode_opts)
title('Approximated Perturbed Plant G by Additive Uncertainty');
grid on
%% Save results
% save('Additive_Uncertainty.mat')
save('Multiplicative_Uncertainty.mat')