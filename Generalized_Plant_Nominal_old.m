function P = Generalized_Plant_Nominal_old(G,Gd,Wp,Wu,Wd,Wr,Gact)

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
G.u = 'u_act';
G.y = 'yG';
Gd.u = 'dw';
Gd.y = 'yGd';

Sum_err = sumblk('v = rw - yG - yGd',3);
inputs = {'r','d','u'};
outputs = {'z1','z2','v'};
P = connect(G,Gd,Wp,Wu,Wd,Wr,Gact,Sum_err,inputs,outputs);
P = minreal(P);

end