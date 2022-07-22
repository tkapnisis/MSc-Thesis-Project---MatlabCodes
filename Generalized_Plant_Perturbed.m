function [P_Delta,P_aug,Gp_app,Gd_p_app] = Generalized_Plant_Perturbed...
                (G,Gd,Gsm,bound_G,bound_Gd,W_I_G,W_I_Gd,W_I_Gsm,...
                 Wp,Wu,Wd,Wr)

states = {'z', 'phi', 'theta', 'z_dot', 'phi_dot', 'theta_dot'};
inputs = {'u_in_un(1)';'u_in_un(2)';'u_in_un(3)'};
disturbances = {'dw_w_un(1)';'dw_w_un(2)';'dw_w_un(3)';'dw_w_un(4)';'dw_w_un(5)';'dw_w_un(6)'};
outputs = {'y(1)';'y(2)';'y(3)'};
G_aug = ss(G.A,[G.B,Gd.B],G.C,[G.D,Gd.D],'statename',states,'inputname',...
                               [inputs;disturbances],'outputname',outputs);

% Define the Perturbed Plant and the Disturbance Transfer Matrix
% Multiplicative Input Uncertainty
Delta_I_Gsm = blkdiag( ultidyn('delta_sm_f',[1,1],'Bound',1),...
                       ultidyn('delta_sm_ap',[1,1],'Bound',1),...
                       ultidyn('delta_sm_as',[1,1],'Bound',1));

Gsm_p_app = (eye(3) + Delta_I_Gsm*W_I_Gsm)*Gsm;

Delta_I_G = ultidyn('Delta_I_G',[3,3],'Bound',bound_G);

Delta_I_Gd = ultidyn('Delta_I_Gd',[6,3],'Bound',bound_Gd);

Gp_app = G*(eye(3) + Delta_I_G*W_I_G)*Gsm_p_app;
Gp_app = minreal(Gp_app);

Gd_p_app = Gd*(eye(6) + Delta_I_Gd*W_I_Gd);
Gd_p_app = minreal(Gd_p_app);

% Weights for controller synthesis
Wp.u = 'v';
Wp.y = 'z1';
Wu.u = 'u_c';
Wu.y = 'z2';
Wd.u = 'dw';
Wd.y = 'dw_w';
Wr.u = 'r';
Wr.y = 'r_w';
Gsm.u = 'u_c_un';
Gsm.y = 'u_in';
% Multiplicative input uncertainties
W_I_Gsm.u = 'u_c';
W_I_Gsm.y = 'y_Delta_Gsm';
W_I_G.u = 'u_in';
W_I_G.y = 'y_Delta_G';
W_I_Gd.u = 'dw_w';
W_I_Gd.y = 'y_Delta_Gd';
% Perturbation blocks
Delta_I_G.u = 'y_Delta_G';
Delta_I_G.y = 'u_Delta_G';
Delta_I_Gd.u = 'y_Delta_Gd';
Delta_I_Gd.y = 'u_Delta_Gd';
Delta_I_Gsm.u = 'y_Delta_Gsm';
Delta_I_Gsm.y = 'u_Delta_Gsm';

sum_u_c_un = sumblk('u_c_un = u_c + u_Delta_Gsm',3);
sum_u_in_un = sumblk('u_in_un = u_in + u_Delta_G',3);
sum_dw_w_un = sumblk('dw_w_un = dw_w + u_Delta_Gd',6);
inputs = {'u_Delta_Gsm','u_Delta_G','u_Delta_Gd','r','dw','u_c'};
outputs = {'y_Delta_Gsm','y_Delta_G','y_Delta_Gd','z1','z2','v'};
sum_err = sumblk('v = r_w - y',3);

P_aug = connect(G_aug,Gsm,W_I_G,W_I_Gd,W_I_Gsm,Wp,Wu,Wr,Wd,...
                sum_u_c_un,sum_u_in_un,sum_dw_w_un,sum_err,inputs,outputs);

disp('Minimal Realization of Perturbed Generalized Plant')
P_aug = minreal(P_aug);

Delta_aug = append(Delta_I_Gsm,Delta_I_G, Delta_I_Gd);
P_Delta = lft(Delta_aug,P_aug);
P_Delta = minreal(P_Delta);

end