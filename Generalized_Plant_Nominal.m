function P = Generalized_Plant_Nominal(G,Gd,Wp,Wu,Wd,Wr,Gact)

states = {'z', 'phi', 'theta', 'z_dot', 'phi_dot', 'theta_dot'};
inputs = {'u_act(1)';'u_act(2)';'u_act(3)'};
disturbances = {'dw(1)';'dw(2)';'dw(3)';'dw(4)';'dw(5)';'dw(6)'};
outputs = {'y(1)';'y(2)';'y(3)'};
G_aug = ss(G.A,[G.B,Gd.B],G.C,[G.D,Gd.D],'statename',states,'inputname',...
                               [inputs;disturbances],'outputname',outputs);
Wp.u = 'v';
Wp.y = 'z1';
Wu.u = 'u';
Wu.y = 'z2';
Wd.u = 'd';
Wd.y = 'dw';
Wr.u = 'r';
Wr.y = 'rw';
Gact.u = 'u';
Gact.y = 'u_act';
% G.u = 'u';
% G.y = 'yG';
% Gd.u = 'dw';
% Gd.y = 'yGd';
% G.u = 'u';
% G.y = 'yG';

% Sum_err = sumblk('v = rw - yG - yGd',3);
Sum_err = sumblk('v = rw - y',3);
% Sum_err = sumblk('v = r - yG',3);
inputs = {'r','d','u'};
% inputs = {'r','u'};
outputs = {'z1','z2','v'};
% P = connect(G,Gd,Wp,Wu,Wd,Wr,Gact,Sum_err,inputs,outputs);
% P = connect(G,Gd,Wp,Wu,Wd,Wr,Sum_err,inputs,outputs);
% P = connect(G,Wp,Wu,Sum_err,inputs,outputs);
P = connect(G_aug,Wp,Wu,Wd,Wr,Gact,Sum_err,inputs,outputs);
P = minreal(P);

end