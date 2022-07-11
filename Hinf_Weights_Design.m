function [Wp,Wu,Wd,Wr,Gact,Gact_p,Wact] = Hinf_Weights_Design()

s = zpk('s');

% Performance weighting functions for the sensitivity
Ms1 = 1.5;
Ms2 = 1.5;
Ms3 = 1.5;
omega_b1 = 0.25*2*pi;
omega_b2 = 0.25*2*pi;
omega_b3 = 0.25*2*pi;
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
omega_bc1 = 10*2*pi;
omega_bc2 = 10*2*pi;
omega_bc3 = 10*2*pi;
Ac_1 = 1e-4;
Ac_2 = 1e-4;
Ac_3 = 1e-4;

Wu11 = (s + omega_bc1/Mu1)/(Ac_1*s + omega_bc1);
Wu22 = (s + omega_bc2/Mu2)/(Ac_2*s + omega_bc2);
Wu33 = (s + omega_bc3/Mu3)/(Ac_3*s + omega_bc3);
Wu = blkdiag(Wu11, Wu22 , Wu33);

% Design band-pass filters for the frequency content of the wave
% disturbances.
% Band pass filter is a combination of a low-pass filter and a high-pass
% filter (see Ogata p.493)
% https://www.analog.com/en/analog-dialogue/articles/band-pass-response-in-active-filters.html

omega_w_low = 0.1; 
omega_w_high = 5;
k_waves = 2e2; % gain that is used to increase the magnitude of the filter
LPF_w = omega_w_low/(s + omega_w_low);
HPF_w = s/(s + omega_w_high);
BPF_w = k_waves*LPF_w*HPF_w;

Wd = blkdiag(BPF_w,BPF_w,BPF_w,BPF_w,BPF_w,BPF_w);
% Wd = zpk(5*eye(6));

% Design low-pass filters for the frequency content of the reference
% signals
tau_z_ref = 4;
tau_phi_ref = 4;
tau_theta_ref = 4;
mag_z_ref = 5;
mag_phi_ref = 0.1;
mag_theta_ref = 0.1;
Wr11 = mag_z_ref/(tau_z_ref*s + 1);
Wr22 = mag_phi_ref/(tau_phi_ref*s + 1);
Wr33 = mag_theta_ref/(tau_theta_ref*s + 1);
Wr = blkdiag(Wr11, Wr22 , Wr33);

% Wr = zpk(blkdiag(1,0.1,0.1));
tau_s_n = 0.1; 
Gact_i_n = 1/(tau_s_n*s + 1);
Gact = blkdiag(Gact_i_n,Gact_i_n,Gact_i_n);

tau_s_p = ureal('tau_s',0.1,'Range',[0, 0.2]); 
Gact_i_p = 1/(tau_s_p*s + 1);
Gact_p = blkdiag(Gact_i_p,Gact_i_p,Gact_i_p);

tau_s_max = 0.2; 
Wact_i = (tau_s_max*s)/(tau_s_max*s + 1);
Wact = blkdiag(Wact_i,Wact_i,Wact_i);


end