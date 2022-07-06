clc
clear all
close all

load('LTI_Perturbed_Plant.mat','G','Gd','Gp','Gd_p')
load('Uncertainty_Approximation_OutMult.mat','W_O')


bode_opts = bodeoptions;
bode_opts.MagScale = 'log';
bode_opts.MagUnits = 'abs';
bode_opts.InputLabels.Interpreter = 'none';
bode_opts.InputLabels.FontSize = 10;
bode_opts.OutputLabels.FontSize = 10;
bode_opts.XLabel.FontSize = 11;
bode_opts.YLabel.FontSize = 11;
bode_opts.TickLabel.FontSize = 10;
bode_opts.Title.FontSize = 12;
% bode_opts.XLimMode = 'manual';
% bode_opts.Xlim = [1e-3 1e2];
bode_opts.PhaseVisible = 'off';
bode_opts.Grid = 'on';

%%
bound = 1;
Delta_O = [ultidyn('d11',[1,1],'Bound',bound),...
           ultidyn('d12',[1,1],'Bound',bound),...
           ultidyn('d13',[1,1],'Bound',bound);...
           ultidyn('d21',[1,1],'Bound',bound),...
           ultidyn('d22',[1,1],'Bound',bound),...
           ultidyn('d23',[1,1],'Bound',bound);...
           ultidyn('d31',[1,1],'Bound',bound),...
           ultidyn('d32',[1,1],'Bound',bound),...
           ultidyn('d33',[1,1],'Bound',bound)];


W_O_Delta.wd11 = W_O.w11*Delta_O(1,1);
W_O_Delta.wd12 = W_O.w12*Delta_O(1,2);
W_O_Delta.wd13 = W_O.w13*Delta_O(1,3);
W_O_Delta.wd21 = W_O.w21*Delta_O(2,1);
W_O_Delta.wd22 = W_O.w22*Delta_O(2,2);
W_O_Delta.wd23 = W_O.w23*Delta_O(2,3);
W_O_Delta.wd31 = W_O.w31*Delta_O(3,1);
W_O_Delta.wd32 = W_O.w32*Delta_O(3,2);
W_O_Delta.wd33 = W_O.w33*Delta_O(3,3);

%%
G_zpk = zpk([]);


for i=1:size(G,1)
    for j=1:size(G,2)
        [z,p,k] = ss2zp(G.A,G.B(:,j),G.C(i,:),G.D(j,i));
        G_zpk(j,i) = zpk(z,p,k);
    end
end    

%%

%%

W_O_Delta_mat = [series(G_zpk(1,1),W_O.w11*Delta_O(1,1)),...
                 series(G_zpk(1,2),W_O.w12*Delta_O(1,2)),...
                 series(G_zpk(1,3),W_O.w13*Delta_O(1,3));...
                 series(G_zpk(2,1),W_O.w21*Delta_O(2,1)),...
                 series(G_zpk(2,2),W_O.w22*Delta_O(2,2)),...
                 series(G_zpk(2,3),W_O.w23*Delta_O(2,3));...
                 series(G_zpk(3,1),W_O.w31*Delta_O(3,1)),...
                 series(G_zpk(3,2),W_O.w32*Delta_O(3,2)),...
                 series(G_zpk(3,3),W_O.w33*Delta_O(3,3))];

Gp_out_mult = parallel(G_zpk,W_O_Delta_mat);
%%
Gp_out_mult = minreal(Gp_out_mult,1e-12);
%%

sys11 = series((1 + W_I_Delta.wd11),G(1,1));
sys12 = series((1 + W_I_Delta.wd12),G(1,2));
sys13 = series((1 + W_I_Delta.wd13),G(1,3));
sys21 = series((1 + W_I_Delta.wd21),G(2,1));
sys22 = series((1 + W_I_Delta.wd23),G(2,2));
sys23 = series((1 + W_I_Delta.wd23),G(2,3));
sys31 = series((1 + W_I_Delta.wd31),G(3,1));
sys32 = series((1 + W_I_Delta.wd32),G(3,2));
sys33 = series((1 + W_I_Delta.wd33),G(3,3));

Gp_app = append(sys11, sys12, sys13, sys21, sys22, sys23, sys31, sys32, sys33);



%%
%%

aux_mat = ss([ ones(3,1), zeros(3,1), zeros(3,1);...
           zeros(3,1),  ones(3,1), zeros(3,1);...
           zeros(3,1), zeros(3,1),  ones(3,1)]);

aux_mat.u = 'u';
aux_mat.y = 'yDelta';

W_I_ref = [W_I.w11,        0,        0, W_I.w21,        0,        0, W_I.w31,        0,        0;...
                  0, W_I.w12,        0,        0, W_I.w22,        0,        0, W_I.w32,       0;...
                  0,        0, W_I.w13,        0,        0, W_I.w23,        0,        0, W_I.w33];
W_I_ref.u = 'uDelta';
W_I_ref.y = 'yW_I';

bound = 1;
Delta_I = blkdiag(ultidyn('d11',[1,1],'Bound',bound),...
                  ultidyn('d12',[1,1],'Bound',bound),...
                  ultidyn('d13',[1,1],'Bound',bound),...
                  ultidyn('d21',[1,1],'Bound',bound),...
                  ultidyn('d22',[1,1],'Bound',bound),...
                  ultidyn('d23',[1,1],'Bound',bound),...
                  ultidyn('d31',[1,1],'Bound',bound),...
                  ultidyn('d32',[1,1],'Bound',bound),...
                  ultidyn('d33',[1,1],'Bound',bound));
Delta_I.u = 'yDelta';
Delta_I.y = 'uDelta';

sys1 = series(aux_mat,Delta_I);
sys2 = series(sys1,W_I_ref);
%%







%%
inputs = {'u'};
outputs = {'y'};

plant_aug = connect(G11,G12,G13,G21,G22,G23,G31,G32,G33,...
                    weight11,weight12,weight13,weight21,weight22,weight23,...
                    weight31,weight32,weight33,Sum_u_un11,Sum_u_un12,Sum_u_un13,...
                    Sum_u_un21,Sum_u_un22,Sum_u_un23,Sum_u_un31,Sum_u_un32,...
                    Sum_u_un33,inputs,outputs);

Gp_app = parallel(ss(eye(3)),sys2);

%%





W_I_Delta_mat = [W_I_Delta.wd11, W_I_Delta.wd12, W_I_Delta.wd13;...
                 W_I_Delta.wd21, W_I_Delta.wd22, W_I_Delta.wd23;...
                 W_I_Delta.wd31, W_I_Delta.wd32, W_I_Delta.wd33];

% 
% 
% W_I_Delta.wd11.u = 'u1';
% W_I_Delta.wd12.u = 'u2';
% W_I_Delta.wd13.u = 'u3';
% W_I_Delta.wd21.u = 'u1';
% W_I_Delta.wd22.u = 'u2';
% W_I_Delta.wd23.u = 'u3';
% W_I_Delta.wd31.u = 'u1';
% W_I_Delta.wd32.u = 'u2';
% W_I_Delta.wd33.u = 'u3';
% 
% W_I_Delta.wd11.y = 'uDelta1';
% W_I_Delta.wd12.y = 'uDelta1';
% W_I_Delta.wd13.y = 'uDelta1';
% W_I_Delta.wd21.y = 'uDelta2';
% W_I_Delta.wd22.y = 'uDelta2';
% W_I_Delta.wd23.y = 'uDelta2';
% W_I_Delta.wd31.y = 'uDelta3';
% W_I_Delta.wd32.y = 'uDelta3';
% W_I_Delta.wd33.y = 'uDelta3';
% 
% uD1 = sumblk('uDelta = u1 + u2 + u3',3);
%%
weight11 = W_I_Delta.wd11;
weight12 = W_I_Delta.wd12;
weight13 = W_I_Delta.wd13;
weight21 = W_I_Delta.wd21;
weight22 = W_I_Delta.wd22;
weight23 = W_I_Delta.wd23;
weight31 = W_I_Delta.wd31;
weight32 = W_I_Delta.wd32;
weight33 = W_I_Delta.wd33;

weight11.u = 'u1';
weight12.u = 'u2';
weight13.u = 'u3';
weight21.u = 'u1';
weight22.u = 'u2';
weight23.u = 'u3';
weight31.u = 'u1';
weight32.u = 'u2';
weight33.u = 'u3';

weight11.y = 'uDelta11';
weight12.y = 'uDelta12';
weight13.y = 'uDelta13';
weight21.y = 'uDelta21';
weight22.y = 'uDelta22';
weight23.y = 'uDelta23';
weight31.y = 'uDelta31';
weight32.y = 'uDelta32';
weight33.y = 'uDelta33';

G11 = G(1,1);
G12 = G(1,2);
G13 = G(1,3);
G21 = G(2,1);
G22 = G(2,2);
G23 = G(2,3);
G31 = G(3,1);
G32 = G(3,2);
G33 = G(3,3);

G11.u = 'u_un_11';
G12.u = 'u_un_12';
G13.u = 'u_un_13';
G21.u = 'u_un_21';
G22.u = 'u_un_22';
G23.u = 'u_un_23';
G31.u = 'u_un_31';
G32.u = 'u_un_32';
G33.u = 'u_un_33';

G11.y = 'y11';
G12.y = 'y12';
G13.y = 'y13';
G21.y = 'y21';
G22.y = 'y22';
G23.y = 'y23';
G31.y = 'y31';
G32.y = 'y32';
G33.y = 'y33';

Sum_u_un11 = sumblk('u_un_11 = u1 + uDelta11',1);
Sum_u_un12 = sumblk('u_un_12 = u2 + uDelta12',1);
Sum_u_un13 = sumblk('u_un_13 = u3 + uDelta13',1);
Sum_u_un21 = sumblk('u_un_21 = u1 + uDelta21',1);
Sum_u_un22 = sumblk('u_un_22 = u2 + uDelta22',1);
Sum_u_un23 = sumblk('u_un_23 = u3 + uDelta23',1);
Sum_u_un31 = sumblk('u_un_31 = u1 + uDelta31',1);
Sum_u_un32 = sumblk('u_un_32 = u2 + uDelta32',1);
Sum_u_un33 = sumblk('u_un_33 = u2 + uDelta33',1);

inputs = {'u1','u2','u3'};
outputs = {'y11','y12','y13','y21','y22','y23','y31','y32','y33',};

plant_aug = connect(G11,G12,G13,G21,G22,G23,G31,G32,G33,...
                    weight11,weight12,weight13,weight21,weight22,weight23,...
                    weight31,weight32,weight33,Sum_u_un11,Sum_u_un12,Sum_u_un13,...
                    Sum_u_un21,Sum_u_un22,Sum_u_un23,Sum_u_un31,Sum_u_un32,...
                    Sum_u_un33,inputs,outputs);

plant_aug = minreal(plant_aug);

%%
figure
bodeplot(Gp_out_mult,G,bode_opts)
set(findall(gcf,'Type','line'),'LineWidth',1.2)
legend('Approximated Perturbed plant Gp_app(s)','Nominal plant G(s)','Location','best','FontSize',11)
title('Gp')

%%
save('Gp_out_mult.mat','Gp_out_mult')