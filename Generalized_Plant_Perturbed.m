function [P_Delta,P_aug,Gp_app,Gd_p_app] = Generalized_Plant_Perturbed...
                (G,Gd,Gact,bound_G,bound_Gd,W_I_G_ss,W_I_Gd_ss,W_I_Gact_ss,...
                 Wp,Wu,Wd,Wr)

states = {'z', 'phi', 'theta', 'z_dot', 'phi_dot', 'theta_dot'};
inputs = {'u_in_un(1)';'u_in_un(2)';'u_in_un(3)'};
disturbances = {'d_un(1)';'d_un(2)';'d_un(3)';'d_un(4)';'d_un(5)';'d_un(6)'};
outputs = {'y(1)';'y(2)';'y(3)'};
G_aug = ss(G.A,[G.B,Gd.B],G.C,[G.D,Gd.D],'statename',states,'inputname',...
                               [inputs;disturbances],'outputname',outputs);

% Define the Perturbed Plant and the Disturbance Transfer Matrix
% Multiplicative Input Uncertainty
Delta_I_Gact = blkdiag(ultidyn('Delta_I_Gact1',[1,1],'Bound',1),...
                       ultidyn('Delta_I_Gact2',[1,1],'Bound',1),...
                       ultidyn('Delta_I_Gact3',[1,1],'Bound',1));

Gact_p_app = (eye(3) + Delta_I_Gact*W_I_Gact_ss)*Gact;

Delta_I_G = ultidyn('Delta_I_G',[3,3],'Bound',bound_G);

Delta_I_Gd = ultidyn('Delta_I_Gd',[6,3],'Bound',bound_Gd);

Gp_app = G*(eye(3) + Delta_I_G*W_I_G_ss)*Gact_p_app;
Gp_app = minreal(Gp_app);

Gd_p_app = Gd*(eye(6) + Delta_I_Gd*W_I_Gd_ss);
Gd_p_app = minreal(Gd_p_app);

% Weights for controller synthesis
Wp.u = 'v';
Wp.y = 'z1';
Wu.u = 'u';
Wu.y = 'z2';
Wd.u = 'd';
Wd.y = 'dw';
Wr.u = 'r';
Wr.y = 'rw';
Gact.u = 'u_act_un';
Gact.y = 'u_act';
% Multiplicative input uncertainties
W_I_Gact_ss.u = 'u';
W_I_Gact_ss.y = 'y_Delta_Gact';
W_I_G_ss.u = 'u_act';
W_I_G_ss.y = 'y_Delta_G';
W_I_Gd_ss.u = 'dw';
W_I_Gd_ss.y = 'y_Delta_Gd';
% Perturbation blocks
Delta_I_G.u = 'y_Delta_G';
Delta_I_G.y = 'u_Delta_G';
Delta_I_Gd.u = 'y_Delta_Gd';
Delta_I_Gd.y = 'u_Delta_Gd';
Delta_I_Gact.u = 'y_Delta_Gact';
Delta_I_Gact.y = 'u_Delta_Gact';

sum_u_act_un = sumblk('u_act_un = u + u_Delta_Gact',3);
sum_u_in_un = sumblk('u_in_un = u_act + u_Delta_G',3);
sum_d = sumblk('d_un = dw + u_Delta_Gd',6);
inputs = {'u_Delta_Gact','u_Delta_G','u_Delta_Gd','r','d','u'};
outputs = {'y_Delta_Gact','y_Delta_G','y_Delta_Gd','z1','z2','v'};
sum_err = sumblk('v = rw - y',3);

P_aug = connect(G_aug,Gact,W_I_G_ss,W_I_Gd_ss,W_I_Gact_ss,Wp,Wu,Wr,Wd,...
                sum_u_act_un,sum_u_in_un,sum_d,sum_err,inputs,outputs);

disp('Minimal Realization of Perturbed Generalized Plant')
P_aug = minreal(P_aug);

Delta_aug = append(Delta_I_Gact,Delta_I_G, Delta_I_Gd);
P_Delta = lft(Delta_aug,P_aug);
P_Delta = minreal(P_Delta);

end