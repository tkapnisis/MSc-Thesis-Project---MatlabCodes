function [P_Delta,Gp_add,Paug] = Generalized_Plant_Additive(G,Gd,bound,W_A,Wp,Wu,Wd,Wr,Wact)

% Defining the complex scalar uncertainties for each channel of perturbed plant

Delta_A = blkdiag(  ultidyn('d11',[1,1],'Bound',bound),...
                    ultidyn('d21',[1,1],'Bound',bound),...
                    ultidyn('d31',[1,1],'Bound',bound),...
                    ultidyn('d12',[1,1],'Bound',bound),...
                    ultidyn('d22',[1,1],'Bound',bound),...
                    ultidyn('d32',[1,1],'Bound',bound),...
                    ultidyn('d13',[1,1],'Bound',bound),...
                    ultidyn('d23',[1,1],'Bound',bound),...
                    ultidyn('d33',[1,1],'Bound',bound));

W_A_ref = [W_A.w11,        0,        0, W_A.w12,        0,        0, W_A.w13,        0,        0;...
                  0, W_A.w21,        0,        0, W_A.w22,        0,        0, W_A.w23,        0;...
                  0,        0, W_A.w31,        0,        0, W_A.w32,        0,       0, W_A.w33];

aux_mat = ss([ ones(3,1), zeros(3,1), zeros(3,1);...
              zeros(3,1),  ones(3,1), zeros(3,1);...
              zeros(3,1), zeros(3,1), ones(3,1)]);

Delta_A = [ultidyn('d11',[1,1],'Bound',bound),...
           ultidyn('d12',[1,1],'Bound',bound),...
           ultidyn('d13',[1,1],'Bound',bound);...
           ultidyn('d21',[1,1],'Bound',bound),...
           ultidyn('d22',[1,1],'Bound',bound),...
           ultidyn('d23',[1,1],'Bound',bound);...
           ultidyn('d31',[1,1],'Bound',bound),...
           ultidyn('d32',[1,1],'Bound',bound),...
           ultidyn('d33',[1,1],'Bound',bound)];

W_A_Delta_mat = [W_A.w11*Delta_A(1,1),...
                 W_A.w12*Delta_A(1,2),...
                 W_A.w13*Delta_A(1,3);...
                 W_A.w21*Delta_A(2,1),...
                 W_A.w22*Delta_A(2,2),...
                 W_A.w23*Delta_A(2,3);...
                 W_A.w31*Delta_A(3,1),...
                 W_A.w32*Delta_A(3,2),...
                 W_A.w33*Delta_A(3,3)];

Gp_add = parallel(G,W_A_Delta_mat);
% Gp_add = minreal(Gp_add);


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
Gp_add.u = 'u';
Gp_add.y = 'yG';
% aux_mat.u = 'u';
% aux_mat.y = 'y_Delta';
% W_A_ref.u = 'u_Delta';
% W_A_ref.y = 'y_W_A';

% Sum_err = sumblk('v = rw - yG - yGd - y_W_A',3);
% inputs = {'u_Delta','r','d','u'};
% outputs = {'y_Delta','z1','z2','v'};

Sum_err = sumblk('v = rw - yG - yGd',3);
inputs = {'r','d','u'};
outputs = {'z1','z2','v'};

% Paug = connect(G,Gd,W_A_ref,aux_mat,Wp,Wu,Wr,Wd,Sum_err,inputs,outputs);
% Paug = connect(G,W_A_ref,aux_mat,Wp,Wu,Sum_err,inputs,outputs);
Paug = connect(Gp_add,Gd,Wp,Wu,Wr,Wd,Sum_err,inputs,outputs);

disp('Minimal realization of Generalized Plant with Additive Uncertainty')
% Paug = minreal(Paug);
P_Delta = Paug;
% Generalized feedback interconnection of Delta block P block
% P_Delta = lft(Delta_A,Paug);
% P_Delta = minreal(P_Delta);


%% Approximated Perturbed Plant by the Additive Uncertainty
%{
aux_mat.u = 'u';
aux_mat.y = 'y_Delta';
W_A_ref.u = 'u_Delta';
W_A_ref.y = 'y_W_A';
G.u = 'u';
G.y = 'y';

Sum_add = sumblk('y_un = y + y_W_A',3);
inputs = {'u_Delta','u'};
outputs = {'y_Delta','y_un'};
Gp_aug = connect(G,W_A_ref,aux_mat,Sum_add,inputs,outputs);
disp('Minimal realization of Perturbed Plant with Additive Uncertainty')
% Gp_aug = minreal(Gp_aug);

Gp_add = lft(Delta_A,Gp_aug);
% Gp_add = minreal(Gp_add);

% Alternative method ***
% sys1 = series(aux_mat,Delta_A);
% sys2 = series(sys1,W_A_ref);
% 
% Gp_app2 = parallel(G,sys2);
% Gp_app2 = minreal(Gp_app2);
%}
end