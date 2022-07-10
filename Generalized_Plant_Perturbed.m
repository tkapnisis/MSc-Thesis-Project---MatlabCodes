function [P_Delta,Gp_app,Gd_p_app] = Generalized_Plant_Perturbed...
                (G,Gd,bound_G,bound_Gd,W_I_G_ss,W_I_Gd_ss,Wp,Wu,Wd,Wr,Wact)

% Define the Perturbed Plant and the Disturbance Transfer Matrix
% Multiplicative Input Uncertainty

Delta_I_G = ultidyn('Delta_I_G',[3,3],'Bound',bound_G);
Delta_I_Gd = ultidyn('Delta_I_Gd',[6,3],'Bound',bound_Gd);

Gp_app = G*(eye(3) + Delta_I_G*W_I_G_ss);
Gp_app = minreal(Gp_app);

Gd_p_app = Gd*(eye(6) + Delta_I_Gd*W_I_Gd_ss);
Gd_p_app = minreal(Gd_p_app);

%%
Wp.u = 'v';
Wp.y = 'z1';
Wu.u = 'u';
Wu.y = 'z2';
Wd.u = 'd';
Wd.y = 'dw';
Wr.u = 'r';
Wr.y = 'rw';
% Wact.u = 'u';
% Wact.y = 'u_Wact';
% G.u = 'uG_un';
% G.y = 'yG';
% Gd.u = 'uGd_un';
% Gd.y = 'yGd';
% W_I_G_ss.u = 'u';
% W_I_G_ss.y = 'yG_Delta';
% W_I_Gd_ss.u = 'dw';
% W_I_Gd_ss.y = 'yGd_Delta';
% Delta_I_G.u = 'yG_Delta';
% Delta_I_G.y = 'uG_Delta';
% Delta_I_Gd.u = 'yGd_Delta';
% Delta_I_Gd.y = 'uGd_Delta';
Gp_app.u = 'u';
Gp_app.y = 'yG';
Gd_p_app.u = 'dw';
Gd_p_app.y = 'yGd';

% sum_in_G = sumblk('uG_un = u + uG_Delta',3);
% sum_in_Gd = sumblk('uGd_un = dw + uGd_Delta',6);
% inputs = {'uG_Delta','uGd_Delta','r','d','u'};
% outputs = {'yG_Delta','yGd_Delta','z1','z2','v'};
Sum_err = sumblk('v = rw - yG - yGd',3);
inputs = {'r','d','u'};
outputs = {'z1','z2','v'};

% P_Delta = connect(G,Gd,W_I_G_ss,W_I_Gd_ss,Delta_I_G,Delta_I_Gd,Wp,Wu,Wr,Wd,...
%                sum_in_G,sum_in_Gd,Sum_err,inputs,outputs);
% P_aug = connect(G,Gd,W_I_G_ss,W_I_Gd_ss,Wp,Wu,Wr,Wd,...
%                sum_in_G,sum_in_Gd,Sum_err,inputs,outputs);
P_Delta = connect(Gp_app,Gd_p_app,Wp,Wu,Wr,Wd,Sum_err,inputs,outputs);
disp('Minimal Realization of Generalized Plant with Multiplicative Uncertainty')
% P_aug = minreal(P_aug);
P_Delta = minreal(P_Delta);
% 
% Delta_aug = append(Delta_I_G, Delta_I_Gd);
% P_Delta = lft(Delta_aug,P_aug);
% P_Delta = minreal(P_Delta);

end