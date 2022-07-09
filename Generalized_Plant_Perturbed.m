function [P_Delta,Paug,Gp_app,Gd_p_app] = Generalized_Plant_Perturbed...
                (G,Gd,bound_G,bound_Gd,W_I_G_ss,W_I_Gd_ss,Wp,Wu,Wd,Wr,Wact)

% Define the Perturbed Plant and the Disturbance Transfer Matrix
% Multiplicative Input Uncertainty
Delta_I_G = ultidyn('Delta_I_G',[3,3],'Bound',bound_G);
Gp_app = G*(eye(3) + W_I_G_ss*Delta_I_G);
Gp_app = minreal(Gp_app);

Delta_I_Gd = ultidyn('Delta_I_Gd',[6,3],'Bound',bound_Gd);
Gd_p_app = Gd*(eye(6) + Delta_I_Gd*W_I_Gd_ss);
Gd_p_app = minreal(Gd_p_app);

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
Gd.u = 'dw';
Gd.y = 'yGd';
Gp_app.u = 'u';
Gp_app.y = 'yG';
% W_O_G_ss.u = 'yG';
% W_O_G_ss.y = 'y_Delta';

% Sum_out = sumblk('u_un = u + u_Delta',3);
% Sum_out = sumblk('y_n = yG + u_Delta',3);
% Sum_err = sumblk('v = rw - y_n - yGd',3);
% inputs = {'u_Delta','r','d','u'};
% outputs = {'y_Delta','z1','z2','v'};

Sum_err = sumblk('v = rw - yG - yGd',3);
inputs = {'r','d','u'};
outputs = {'z1','z2','v'};

% Paug = connect(G,Gd,W_O_G_ss,Wp,Wu,Wr,Wd,Sum_out,Sum_err,inputs,outputs);
Paug = connect(Gp_app,Gd,Wp,Wu,Wr,Wd,Sum_err,inputs,outputs);
disp('Minimal Realization of Generalized Plant with Multiplicative Uncertainty')
Paug = minreal(Paug);
P_Delta = Paug;
% Generalized feedback interconnection of Delta block P block
% P_Delta = lft(Delta_O_G,Paug);
% P_Delta = minreal(P_Delta);

end