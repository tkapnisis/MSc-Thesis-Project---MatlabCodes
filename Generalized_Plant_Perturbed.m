function [P_Delta,Gp_app,Gd_p_app] = Generalized_Plant_Perturbed...
                (G,Gd,bound_G,bound_Gd,W_I_G,W_I_Gd,Wp,Wu,Wd,Wr,Wact)

% Define the Perturbed Plant and the Disturbance Transfer Matrix
% Multiplicative Input Uncertainty

delta_G11 = ultidyn('delta_G11',[1,1],'Bound',bound_G);
delta_G12 = ultidyn('delta_G12',[1,1],'Bound',bound_G);
delta_G13 = ultidyn('delta_G13',[1,1],'Bound',bound_G);
delta_G21 = ultidyn('delta_G21',[1,1],'Bound',bound_G);
delta_G22 = ultidyn('delta_G22',[1,1],'Bound',bound_G);
delta_G23 = ultidyn('delta_G23',[1,1],'Bound',bound_G);
delta_G31 = ultidyn('delta_G31',[1,1],'Bound',bound_G);
delta_G32 = ultidyn('delta_G32',[1,1],'Bound',bound_G);
delta_G33 = ultidyn('delta_G33',[1,1],'Bound',bound_G);

W_I_Delta_G = [  W_I_G.w11*delta_G11,...
                 W_I_G.w12*delta_G12,...
                 W_I_G.w13*delta_G13;...
                 W_I_G.w21*delta_G21,...
                 W_I_G.w22*delta_G22,...
                 W_I_G.w23*delta_G23;...
                 W_I_G.w31*delta_G31,...
                 W_I_G.w32*delta_G32,...
                 W_I_G.w33*delta_G33];

delta_Gd11 = ultidyn('delta_Gd11',[1,1],'Bound',bound_Gd);
delta_Gd12 = ultidyn('delta_Gd12',[1,1],'Bound',bound_Gd);
delta_Gd13 = ultidyn('delta_Gd13',[1,1],'Bound',bound_Gd);
delta_Gd14 = ultidyn('delta_Gd14',[1,1],'Bound',bound_Gd);
delta_Gd15 = ultidyn('delta_Gd15',[1,1],'Bound',bound_Gd);
delta_Gd16 = ultidyn('delta_Gd16',[1,1],'Bound',bound_Gd);
delta_Gd21 = ultidyn('delta_Gd21',[1,1],'Bound',bound_Gd);
delta_Gd22 = ultidyn('delta_Gd22',[1,1],'Bound',bound_Gd);
delta_Gd23 = ultidyn('delta_Gd23',[1,1],'Bound',bound_Gd);
delta_Gd24 = ultidyn('delta_Gd24',[1,1],'Bound',bound_Gd);
delta_Gd25 = ultidyn('delta_Gd25',[1,1],'Bound',bound_Gd);
delta_Gd26 = ultidyn('delta_Gd26',[1,1],'Bound',bound_Gd);
delta_Gd31 = ultidyn('delta_Gd31',[1,1],'Bound',bound_Gd);
delta_Gd32 = ultidyn('delta_Gd32',[1,1],'Bound',bound_Gd);
delta_Gd33 = ultidyn('delta_Gd33',[1,1],'Bound',bound_Gd);
delta_Gd34 = ultidyn('delta_Gd34',[1,1],'Bound',bound_Gd);
delta_Gd35 = ultidyn('delta_Gd35',[1,1],'Bound',bound_Gd);
delta_Gd36 = ultidyn('delta_Gd36',[1,1],'Bound',bound_Gd);

W_I_Delta_Gd = [ W_I_Gd.w11*delta_Gd11,...
                 W_I_Gd.w12*delta_Gd12,...
                 W_I_Gd.w13*delta_Gd13,...
                 W_I_Gd.w14*delta_Gd14,...
                 W_I_Gd.w15*delta_Gd15,...
                 W_I_Gd.w16*delta_Gd16;...
                 W_I_Gd.w21*delta_Gd21,...
                 W_I_Gd.w22*delta_Gd22,...
                 W_I_Gd.w23*delta_Gd23,...
                 W_I_Gd.w24*delta_Gd24,...
                 W_I_Gd.w25*delta_Gd25,...
                 W_I_Gd.w26*delta_Gd26;...
                 W_I_Gd.w31*delta_Gd31,...
                 W_I_Gd.w32*delta_Gd32,...
                 W_I_Gd.w33*delta_Gd33,...
                 W_I_Gd.w34*delta_Gd34,...
                 W_I_Gd.w35*delta_Gd35,...
                 W_I_Gd.w36*delta_Gd36];
%%
Delta_O_Gd = ultidyn('Delta_O_Gd',[6,3],'Bound',1);
%%
Gp_app = G*(eye(3) + G_W_I_Delta);
Gp_app = minreal(Gp_app);

Gd_p_app = Gd*(eye(3) + Gd_W_I_Delta);
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
Gd.u = 'uGd_un';
Gd.y = 'yGd';
G.u = 'uG_un';
G.y = 'yG';
W_I_G_ss.u = 'u';
W_I_G_ss.y = 'yG_Delta';
W_I_Gd_ss.u = 'dw';
W_I_Gd_ss.y = 'yGd_Delta';
Delta_I_G.u = 'yG_Delta';
Delta_I_G.y = 'uG_Delta';
Delta_I_Gd.u = 'yGd_Delta';
Delta_I_Gd.y = 'uGd_Delta';

sum_in_G = sumblk('uG_un = u + uG_Delta',3);
sum_in_Gd = sumblk('uGd_un = dw + uGd_Delta',6);
Sum_err = sumblk('v = rw - yG - yGd',3);
inputs = {'uG_Delta','uGd_Delta','r','d','u'};
outputs = {'yG_Delta','yGd_Delta','z1','z2','v'};

P_Delta = connect(G,Gd,W_I_G_ss,W_I_Gd_ss,Delta_I_G,Delta_I_Gd,Wp,Wu,Wr,Wd,...
               sum_in_G,sum_in_Gd,Sum_err,inputs,outputs);
disp('Minimal Realization of Generalized Plant with Multiplicative Uncertainty')
P_Delta = minreal(P_Delta);

end