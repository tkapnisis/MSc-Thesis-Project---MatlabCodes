function [Wp,Wu,Wd,Wr,Wact] = Hinf_Weights_Design()

s = zpk('s');

% Performance weighting functions for the sensitivity
Ms1 = 2;
Ms2 = 2;
Ms3 = 2;
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
omega_bc1 = 30*2*pi;
omega_bc2 = 30*2*pi;
omega_bc3 = 30*2*pi;
Ac_1 = 1e-2;
Ac_2 = 1e-2;
Ac_3 = 1e-2;

Wu11 = (s + omega_bc1/Mu1)/(Ac_1*s + omega_bc1);
Wu22 = (s + omega_bc2/Mu2)/(Ac_2*s + omega_bc2);
Wu33 = (s + omega_bc3/Mu3)/(Ac_3*s + omega_bc3);
Wu = blkdiag(Wu11, Wu22 , Wu33);

Wd = zpk(2*eye(6));

Wr = zpk(blkdiag(0.5,0.1,0.1));

% tau_s = ureal('tau_s',0.05,'Range',[0, 0.1]); 
tau_s = 0.05; 
% Wact_11 = (tau_s*s)/(tau_s*s + 1);
Wact_11 = (1)/(tau_s*s + 1);
% Wact_11 = zpk(1);
Wact_22 = Wact_11;
Wact_33 = Wact_11;

Wact = blkdiag(Wact_11,Wact_22,Wact_33);
% Wact = eye(3) - Wact;

end