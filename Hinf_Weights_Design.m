function [Wp,Wu,Wd,Wr,Wact] = Hinf_Weights_Design()

s = zpk('s');
M_1 = 1.5;
M_2 = 1.5;
M_3 = 1.5;
w_c1 = 0.5*2*pi;
w_c2 = 1*2*pi;
w_c3 = 1*2*pi;
A_1 = 1e-4;
A_2 = 1e-4;
A_3 = 1e-4;

n = 1;

Wp11 = (s/M_1*(1/n) + w_c1)^n/(s + w_c1*A_1^(1/n))^n;
Wp22 = (s/M_2*(1/n) + w_c2)^n/(s + w_c2*A_2*(1/n))^n;
Wp33 = (s/M_3*(1/n) + w_c3)^n/(s + w_c3*A_3*(1/n))^n;
Wp = [Wp11,   0  ,   0 ;
        0 , Wp22 ,   0 ;
        0 ,   0  , Wp33];

% Wu11 = 1/2;
% Wu22 = 1/2;
% Wu33 = 1/2;

% Wu11 = makeweight(1e-2, [2*2*pi,1e-1], 1e2);
Wu11 = makeweight(1e-3, [1*2*pi,1e-1], 1e1);
Wu22 = Wu11;
Wu33 = Wu11;

Wu = 1*[Wu11,   0  ,   0 ;
        0 , Wu22 ,   0 ;
        0 ,   0  , Wu33];
% Wu = zpk(Wu);

Wd = zpk(50*eye(6));

% zeta_1 = 1;
% zeta_2 = 1;
% zeta_3 = 1;
% Wr11 = w_c1^2/(s^2 + 2*zeta_1*w_c1*s + w_c1^2);
% Wr22 = w_c2^2/(s^2 + 2*zeta_2*w_c2*s + w_c2^2);
% Wr33 = w_c3^2/(s^2 + 2*zeta_3*w_c3*s + w_c3^2);
% Wr = [Wr11,   0  ,   0 ;
%         0 , Wr22 ,   0 ;
%         0 ,   0  , Wr33];
Wr = zpk(1*eye(3));

tau_s = ureal('tau_s',0.0001,'Range',[0, 0.13]); 
Wact_11 = (tau_s*s)/(tau_s*s + 1);
Wact_22 = Wact_11;
Wact_33 = Wact_11;

Wact = blkdiag(Wact_11,Wact_22,Wact_33);

end