function [P_Delta,Gp_mult] = Generalized_Plant_Multiplicative(G,Gd,bound,version,W_I,Wp,Wu,Wd,Wr,Wact)

% Defining the complex scalar uncertainties for each channel of perturbed plant
Delta_I = [ultidyn('d11',[1,1],'Bound',bound),...
           ultidyn('d12',[1,1],'Bound',bound),...
           ultidyn('d13',[1,1],'Bound',bound);...
           ultidyn('d21',[1,1],'Bound',bound),...
           ultidyn('d22',[1,1],'Bound',bound),...
           ultidyn('d23',[1,1],'Bound',bound);...
           ultidyn('d31',[1,1],'Bound',bound),...
           ultidyn('d32',[1,1],'Bound',bound),...
           ultidyn('d33',[1,1],'Bound',bound)];

switch version
    case 1
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 1st method %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % The perturbed plant is defined by multpling all the scalar transfer
        % functions in each channel (check Gu et. al., Chapter 9 last section)
        disp('----- 1st method -----')
        Gp_mult = [  (W_I.w11*Delta_I(1,1)+1)*G(1,1),...
                     (W_I.w12*Delta_I(1,2)+1)*G(1,2),...
                     (W_I.w13*Delta_I(1,3)+1)*G(1,3);...
                     (W_I.w21*Delta_I(2,1)+1)*G(2,1),...
                     (W_I.w22*Delta_I(2,2)+1)*G(2,2),...
                     (W_I.w23*Delta_I(2,3)+1)*G(2,3);...
                     (W_I.w31*Delta_I(3,1)+1)*G(3,1),...
                     (W_I.w32*Delta_I(3,2)+1)*G(3,2),...
                     (W_I.w33*Delta_I(3,3)+1)*G(3,3)];
%         Gp_mult = [  G(1,1)*(1+W_I.w11*Delta_I(1,1)),...
%                  G(1,2)*(1+W_I.w12*Delta_I(1,2)),...
%                  G(1,3)*(1+W_I.w13*Delta_I(1,3));...
%                  G(2,1)*(1+W_I.w21*Delta_I(2,1)),...
%                  G(2,2)*(1+W_I.w22*Delta_I(2,2)),...
%                  G(2,3)*(1+W_I.w23*Delta_I(2,3));...
%                  G(3,1)*(1+W_I.w31*Delta_I(3,1)),...
%                  G(3,2)*(1+W_I.w32*Delta_I(3,2)),...
%                  G(3,3)*(1+W_I.w33*Delta_I(3,3))];
%         Gp_mult = minreal(Gp_mult);

    case 2
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 2nd method %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % The perturbed plant is defined by multpling all the scalar transfer
        % functions in each channel for the part G*Delta*W and the result is added
        % with the parallel transfer function of G
        disp('----- 2nd method -----')
        W_I_mat = [  G(1,1)*W_I.w11*Delta_I(1,1),...
                     G(1,2)*W_I.w12*Delta_I(1,2),...
                     G(1,3)*W_I.w13*Delta_I(1,3);...
                     G(2,1)*W_I.w21*Delta_I(2,1),...
                     G(2,2)*W_I.w22*Delta_I(2,2),...
                     G(2,3)*W_I.w23*Delta_I(2,3);...
                     G(3,1)*W_I.w31*Delta_I(3,1),...
                     G(3,2)*W_I.w32*Delta_I(3,2),...
                     G(3,3)*W_I.w33*Delta_I(3,3)];
        
        Gp_mult = parallel(G,W_I_mat);
%         Gp_mult = minreal(Gp_mult);
    case 3
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 3rd method %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Working but with low performance because the multiplicative uncertainties
        % of each channel are treated like they are diagonal because we connect
        % the branch of uncertainties in parallel with Identity and then in series
        % with G
        disp('----- 3rd method -----')
        W_I_mat = [  W_I.w11*Delta_I(1,1),...
                     W_I.w12*Delta_I(1,2),...
                     W_I.w13*Delta_I(1,3);...
                     W_I.w21*Delta_I(2,1),...
                     W_I.w22*Delta_I(2,2),...
                     W_I.w23*Delta_I(2,3);...
                     W_I.w31*Delta_I(3,1),...
                     W_I.w32*Delta_I(3,2),...
                     W_I.w33*Delta_I(3,3)];
        
        sys1 = parallel(ss(eye(3)),W_I_mat);
        Gp_mult = series(sys1,G);
%         sys1 = series(G,ss(eye(3)));
%         sys2 = series(G,W_I_mat);
%         Gp_mult = parallel(sys1,sys2);
        Gp_mult = minreal(Gp_mult);
end

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
Gp_mult.u = 'u';
Gp_mult.y = 'yG';

Sum_err = sumblk('v = rw - yG - yGd',3);
inputs = {'r','d','u'};
outputs = {'z1','z2','v'};

P_Delta = connect(Gp_mult,Gd,Wp,Wu,Wr,Wd,Sum_err,inputs,outputs);
disp('Minimal realization of Generalized Plant with Multiplicative Uncertainty')
% P_Delta = minreal(P_Delta);

end