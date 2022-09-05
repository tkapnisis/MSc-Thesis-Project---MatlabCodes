function [Wp,Wu,Wd,Wr] = Design_Weights()

s = zpk('s');

% Performance weighting functions for the sensitivity
Ms1 = 1.5;
Ms2 = 1.5;
Ms3 = 1.5;
omega_b1 = 5;
omega_b2 = 5;
omega_b3 = 5;
A_1 = 1e-4;
A_2 = 1e-4;
A_3 = 1e-4;

Wp11 = (s/Ms1 + omega_b1)/(s + omega_b1*A_1);
Wp22 = (s/Ms2 + omega_b2)/(s + omega_b2*A_2);
Wp33 = (s/Ms3 + omega_b3)/(s + omega_b3*A_3);
Wp = blkdiag(Wp11, Wp22 , Wp33);

% Control input weighting functions 
Mu1 = 1e3;
Mu2 = 1e3;
Mu3 = 1e3;
omega_bc1 = 50;
omega_bc2 = 50;
omega_bc3 = 50;
Ac_1 = 1e-3;
Ac_2 = 1e-3;
Ac_3 = 1e-3;

Wu11 = (s + omega_bc1/Mu1)/(Ac_1*s + omega_bc1);
Wu22 = (s + omega_bc2/Mu2)/(Ac_2*s + omega_bc2);
Wu33 = (s + omega_bc3/Mu3)/(Ac_3*s + omega_bc3);
Wu = blkdiag(Wu11, Wu22 , Wu33);

% Design band-pass filters for the frequency content of the wave
% disturbances.
% Band pass filter is a combination of a low-pass filter and a high-pass
% filter (see Ogata p.493)
% https://www.analog.com/en/analog-dialogue/articles/band-pass-response-in-active-filters.html

omega_L = 1.52;
omega_H = 32.98;
omega_0 = sqrt(omega_H*omega_L);
Q = omega_0/(omega_H-omega_L);  
H0 = 1.5;
BPF_w = (H0*(omega_0/Q)*s)/(s^2 + omega_0/Q*s + omega_0^2);

Wd = blkdiag(BPF_w,BPF_w,BPF_w,BPF_w,BPF_w,BPF_w);

% Design low-pass filters for the frequency content of the reference
% signals
tau_r1 = 4;
tau_r2 = 4;
tau_r3 = 4;
mag_r1 = 0.2;
mag_r2 = 0.1;
mag_r3 = 0.1;
Wr11 = mag_r1/(tau_r1*s + 1);
Wr22 = mag_r2/(tau_r2*s + 1);
Wr33 = mag_r3/(tau_r3*s + 1);
Wr = blkdiag(Wr11, Wr22 , Wr33);

end