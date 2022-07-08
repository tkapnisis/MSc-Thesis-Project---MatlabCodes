function [P_Delta,Gp_app,Gd_p_app] = Generalized_Plant_Perturbed...
                (G,Gd,bound_G,bound_Gd,W_O_G_ss,W_O_Gd_ss,Wp,Wu,Wd,Wr,Wact)

% Define the Perturbed Plant and the Disturbance Transfer Matrix
% Output Multiplicative Uncertainty
Delta_O_G = ultidyn('Delta_O_G',[3,3],'Bound',bound_G);
Gp_app = (eye(3) + W_O_G_ss*Delta_O_G)*G;
Gp_app = minreal(Gp_app);

Delta_O_Gd = ultidyn('Delta_O_Gd',[6,3],'Bound',bound_Gd);
Gd_p_app = (eye(3) + W_O_Gd_ss*Delta_O_Gd)*Gd;
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
G.u = 'u_un';
G.y = 'yG';
W_O_G_ss.u = 'u';
W_O_G_ss.y = 'y_Delta';

Sum_out = sumblk('u_un = u + u_Delta',3);
Sum_err = sumblk('v = rw - yG - yGd',3);
inputs = {'u_Delta','r','d','u'};
outputs = {'y_Delta','z1','z2','v'};

% Sum_err = sumblk('v = rw - yG - yGd',3);
% inputs = {'r','d','u'};
% outputs = {'z1','z2','v'};

Paug = connect(G,Gd,W_O_G_ss,Wp,Wu,Wr,Wd,Sum_err,Sum_out,inputs,outputs);
% Paug = connect(G,Gd,Wp,Wu,Wr,Wd,Sum_err,inputs,outputs);
disp('Minimal Realization of Generalized Plant with Multiplicative Uncertainty')
Paug = minreal(Paug);

% Generalized feedback interconnection of Delta block P block
P_Delta = lft(Delta_O_G,Paug);
P_Delta = minreal(P_Delta);

end