function [dw,wave_param] = Wave_Model(t,wave_param,foil_loc,param)

wave_param.k = 2*pi/wave_param.lambda; % Wave number

% Encounter frequency
wave_param.omega_e = wave_param.omega_0 - wave_param.omega_0^2/param.g*...
                     param.U_0*cos(wave_param.beta);

% Extra parameters to understand the speed and the period of the wave
wave_param.T = 2*pi/wave_param.omega_0;        % Wave period [s]
wave_param.c = wave_param.lambda/wave_param.T; % Wave forward speed (called the phase speed) [m/s]


% % Inflow velocity in x-axis
u_w_f = -wave_param.omega_0*wave_param.zeta_0*exp(-wave_param.k*foil_loc.h_f)...
        *sin(wave_param.k*foil_loc.l_x_f - wave_param.omega_e*t)*cos(wave_param.beta);
u_w_ap = -wave_param.omega_0*wave_param.zeta_0*exp(-wave_param.k*foil_loc.h_ap)...
        *sin(wave_param.k*foil_loc.l_x_ap - wave_param.omega_e*t)*cos(wave_param.beta);
u_w_as = -wave_param.omega_0*wave_param.zeta_0*exp(-wave_param.k*foil_loc.h_as)...
        *sin(wave_param.k*foil_loc.l_x_as - wave_param.omega_e*t)*cos(wave_param.beta);
% Inflow velocity in z-axis
w_w_f = -wave_param.omega_0*wave_param.zeta_0*exp(-wave_param.k*foil_loc.h_f)...
        *cos(wave_param.k*foil_loc.l_x_f - wave_param.omega_e*t);
w_w_ap = -wave_param.omega_0*wave_param.zeta_0*exp(-wave_param.k*foil_loc.h_ap)...
        *cos(wave_param.k*foil_loc.l_x_ap - wave_param.omega_e*t);
w_w_as = -wave_param.omega_0*wave_param.zeta_0*exp(-wave_param.k*foil_loc.h_as)...
        *cos(wave_param.k*foil_loc.l_x_as - wave_param.omega_e*t);

dw = [u_w_f;w_w_f;u_w_ap;w_w_ap;u_w_as;w_w_as];
end