function [Wp,Wu,Wd,Wr,Wact] = Hinf_Weights_Design()

s = zpk('s');

% Performance weighting functions for the sensitivity
Ms1 = 1.5;
Ms2 = 1.5;
Ms3 = 1.5;
omega_b1 = 0.5*2*pi;
omega_b2 = 0.5*2*pi;
omega_b3 = 0.5*2*pi;
A_1 = 1e-4;
A_2 = 1e-4;
A_3 = 1e-4;

Wp11 = (s/Ms1 + omega_b1)/(s + omega_b1*A_1);
Wp22 = (s/Ms2 + omega_b2)/(s + omega_b2*A_2);
Wp33 = (s/Ms3 + omega_b3)/(s + omega_b3*A_3);
Wp = blkdiag(Wp11, Wp22 , Wp33);

% Control input weighting functions 
Mu1 = 1e2;
Mu2 = 1e2;
Mu3 = 1e2;
omega_bc1 = 20*2*pi;
omega_bc2 = 20*2*pi;
omega_bc3 = 20*2*pi;
Ac_1 = 1e-2;
Ac_2 = 1e-2;
Ac_3 = 1e-2;

Wu11 = (s + omega_bc1/Mu1)/(Ac_1*s + omega_bc1);
Wu22 = (s + omega_bc2/Mu2)/(Ac_2*s + omega_bc2);
Wu33 = (s + omega_bc3/Mu3)/(Ac_3*s + omega_bc3);
Wu = blkdiag(Wu11, Wu22 , Wu33);
% Wu = zpk(0.1*eye(3));

% Wu11m = zpk(makeweight(1e-2, [2*2*pi,1e-1], 1e2));
% Wu11 = makeweight(1e-3, [1*2*pi,1e-1], 1e1);
% Wu22 = Wu11;
% Wu33 = Wu11;
% 
% Wu = 1*[Wu11,   0  ,   0 ;
%         0 , Wu22 ,   0 ;
%         0 ,   0  , Wu33];
% Wu = zpk(Wu);

Wd = zpk(5*eye(6));

% zeta_1 = 1;
% zeta_2 = 1;
% zeta_3 = 1;
% ratio = 1;
% Wr11 = (omega_b1/ratio)^2/(s^2 + 2*zeta_1*(omega_b1/ratio)*s + (omega_b1/ratio)^2);
% Wr22 = (omega_b2/ratio)^2/(s^2 + 2*zeta_2*(omega_b2/ratio)*s + (omega_b2/ratio)^2);
% Wr33 = (omega_b3/ratio)^2/(s^2 + 2*zeta_3*(omega_b3/ratio)*s + (omega_b3/ratio)^2);
% Wr = blkdiag(Wr11, Wr22 , Wr33);

% T_1 = 0.5;
% T_2 = 0.5;
% T_3 = 0.5;
% Wr11 = 1/(T_1*s^2 + 2*zeta_1*T_1*s + 1);
% Wr22 = 1/(T_2*s^2 + 2*zeta_2*T_2*s + 1);
% Wr33 = 1/(T_3*s^2 + 2*zeta_3*T_3*s + 1);
% Wr = blkdiag(Wr11, Wr22 , Wr33);
Wr = zpk(blkdiag(1,1,1));

% tau_s = ureal('tau_s',0.0001,'Range',[0, 0.13]); 
tau_s = 0.01; 
% Wact_11 = (tau_s*s)/(tau_s*s + 1);
% Wact_11 = (1)/(tau_s*s + 1);
Wact_11 = zpk(1);
Wact_22 = Wact_11;
Wact_33 = Wact_11;

Wact = blkdiag(Wact_11,Wact_22,Wact_33);
% Wact = eye(3) - Wact;

end