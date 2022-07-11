function P = Generalized_Plant_Nominal(G,Gd,Wp,Wu,Wd,Wr,Wact)

Wp.u = 'v';
Wp.y = 'z1';
Wu.u = 'u';
Wu.y = 'z2';
Wd.u = 'd';
Wd.y = 'dw';
Wr.u = 'r';
Wr.y = 'rw';
Wact.u = 'u';
Wact.y = 'u_act';
G.u = 'u_act';
G.y = 'yG';
Gd.u = 'dw';
Gd.y = 'yGd';
% G.u = 'u';
% G.y = 'yG';

Sum_err = sumblk('v = rw - yG - yGd',3);
% Sum_err = sumblk('v = r - y',3);
% Sum_err = sumblk('v = r - yG',3);
inputs = {'r','d','u'};
% inputs = {'r','u'};
outputs = {'z1','z2','v'};
P = connect(G,Gd,Wp,Wu,Wd,Wr,Wact,Sum_err,inputs,outputs);
% P = connect(G,Gd,Wp,Wu,Wd,Wr,Sum_err,inputs,outputs);
% P = connect(G,Wp,Wu,Sum_err,inputs,outputs);
P = minreal(P);

end