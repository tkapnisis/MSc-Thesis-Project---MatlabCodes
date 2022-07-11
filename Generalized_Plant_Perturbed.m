function [P_Delta,Gp_app,Gd_p_app] = Generalized_Plant_Perturbed...
                (G,Gd,bound_G,bound_Gd,W_I_G_ss,W_I_Gd_ss,Wp,Wu,Wd,Wr,Wact)

% Define the Perturbed Plant and the Disturbance Transfer Matrix
% Multiplicative Input Uncertainty

Delta_I_G = ultidyn('Delta_I_G',[3,3],'Bound',bound_G);
Delta_I_Gd = ultidyn('Delta_I_Gd',[6,3],'Bound',bound_Gd);

% bound_G11 = bound_G;
% bound_G12 = bound_G;
% bound_G13 = bound_G;
% bound_G21 = bound_G;
% bound_G22 = bound_G;
% bound_G23 = bound_G;
% bound_G31 = bound_G;
% bound_G32 = bound_G;
% bound_G33 = bound_G;
% 
% delta_G11 = ultidyn('delta_G11',[1,1],'Bound',bound_G11);
% delta_G12 = ultidyn('delta_G12',[1,1],'Bound',bound_G12);
% delta_G13 = ultidyn('delta_G13',[1,1],'Bound',bound_G13);
% delta_G21 = ultidyn('delta_G21',[1,1],'Bound',bound_G21);
% delta_G22 = ultidyn('delta_G22',[1,1],'Bound',bound_G22);
% delta_G23 = ultidyn('delta_G23',[1,1],'Bound',bound_G23);
% delta_G31 = ultidyn('delta_G31',[1,1],'Bound',bound_G31);
% delta_G32 = ultidyn('delta_G32',[1,1],'Bound',bound_G32);
% delta_G33 = ultidyn('delta_G33',[1,1],'Bound',bound_G33);
% 
% Delta_I_G = [delta_G11,delta_G12,delta_G13;...
%              delta_G21,delta_G22,delta_G23;
%              delta_G31,delta_G32,delta_G33];
% 
% bound_Gd11 = bound_Gd;
% bound_Gd12 = bound_Gd;
% bound_Gd13 = bound_Gd;
% bound_Gd14 = bound_Gd;
% bound_Gd15 = bound_Gd;
% bound_Gd16 = bound_Gd;
% bound_Gd21 = bound_Gd;
% bound_Gd22 = bound_Gd;
% bound_Gd23 = bound_Gd;
% bound_Gd24 = bound_Gd;
% bound_Gd25 = bound_Gd;
% bound_Gd26 = bound_Gd;
% bound_Gd31 = bound_Gd;
% bound_Gd32 = bound_Gd;
% bound_Gd33 = bound_Gd;
% bound_Gd34 = bound_Gd;
% bound_Gd35 = bound_Gd;
% bound_Gd36 = bound_Gd;
% 
% delta_Gd11 = ultidyn('delta_Gd11',[1,1],'Bound',bound_Gd11);
% delta_Gd12 = ultidyn('delta_Gd12',[1,1],'Bound',bound_Gd12);
% delta_Gd13 = ultidyn('delta_Gd13',[1,1],'Bound',bound_Gd13);
% delta_Gd14 = ultidyn('delta_Gd14',[1,1],'Bound',bound_Gd14);
% delta_Gd15 = ultidyn('delta_Gd15',[1,1],'Bound',bound_Gd15);
% delta_Gd16 = ultidyn('delta_Gd16',[1,1],'Bound',bound_Gd16);
% delta_Gd21 = ultidyn('delta_Gd21',[1,1],'Bound',bound_Gd21);
% delta_Gd22 = ultidyn('delta_Gd22',[1,1],'Bound',bound_Gd22);
% delta_Gd23 = ultidyn('delta_Gd23',[1,1],'Bound',bound_Gd23);
% delta_Gd24 = ultidyn('delta_Gd24',[1,1],'Bound',bound_Gd24);
% delta_Gd25 = ultidyn('delta_Gd25',[1,1],'Bound',bound_Gd25);
% delta_Gd26 = ultidyn('delta_Gd26',[1,1],'Bound',bound_Gd26);
% delta_Gd31 = ultidyn('delta_Gd31',[1,1],'Bound',bound_Gd31);
% delta_Gd32 = ultidyn('delta_Gd32',[1,1],'Bound',bound_Gd32);
% delta_Gd33 = ultidyn('delta_Gd33',[1,1],'Bound',bound_Gd33);
% delta_Gd34 = ultidyn('delta_Gd34',[1,1],'Bound',bound_Gd34);
% delta_Gd35 = ultidyn('delta_Gd35',[1,1],'Bound',bound_Gd35);
% delta_Gd36 = ultidyn('delta_Gd36',[1,1],'Bound',bound_Gd36);
% 
% Delta_I_Gd = [delta_Gd11,delta_Gd21,delta_Gd31;...
%               delta_Gd12,delta_Gd22,delta_Gd32;...
%               delta_Gd13,delta_Gd23,delta_Gd33;...
%               delta_Gd14,delta_Gd24,delta_Gd34;...
%               delta_Gd15,delta_Gd25,delta_Gd35;...
%               delta_Gd16,delta_Gd26,delta_Gd36];

Delta_Act = blkdiag(ultidyn('Delta_Act1',[1,1],'Bound',1),...
                    ultidyn('Delta_Act2',[1,1],'Bound',1),...
                    ultidyn('Delta_Act3',[1,1],'Bound',1));

Gp_app = G*(eye(3) + Delta_I_G*W_I_G_ss + Delta_Act*Wact);
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
Gd.u = 'dw';
Gd.y = 'yGd';

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
P_Delta = connect(Gp_app,Gd,Wp,Wu,Wr,Wd,Sum_err,inputs,outputs);
disp('Minimal Realization of Generalized Plant with Multiplicative Uncertainty')
% P_aug = minreal(P_aug);
P_Delta = minreal(P_Delta);
% 
% Delta_aug = append(Delta_I_G, Delta_I_Gd);
% P_Delta = lft(Delta_aug,P_aug);
% P_Delta = minreal(P_Delta);

end