function P = Generalized_Plant_Nominal(G,Gd,Wp,Wu,Wd,Wr,Gsm)

states = {'z', 'phi', 'theta', 'z_dot', 'phi_dot', 'theta_dot'};
inputs = {'u_c(1)';'u_c(2)';'u_c(3)'};
disturbances = {'dw(1)';'dw(2)';'dw(3)';'dw(4)';'dw(5)';'dw(6)'};
outputs = {'y(1)';'y(2)';'y(3)'};
G_aug = ss(G.A,[G.B,Gd.B],G.C,[G.D,Gd.D],'statename',states,'inputname',...
                               [inputs;disturbances],'outputname',outputs);
Wp.u = 'v';
Wp.y = 'z1';
Wu.u = 'u_in';
Wu.y = 'z2';
Wd.u = 'd';
Wd.y = 'dw';
Wr.u = 'r';
Wr.y = 'rw';
Gsm.u = 'u_in';
Gsm.y = 'u_c';

Sum_err = sumblk('v = rw - y',3);

inputs = {'r','d','u_in'};
outputs = {'z1','z2','v'};
P = connect(G_aug,Wp,Wu,Wd,Wr,Gsm,Sum_err,inputs,outputs);
P = minreal(P);

end