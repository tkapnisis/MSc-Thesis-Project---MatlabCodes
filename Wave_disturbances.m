function [d_DT,foil_var_loc] = Wave_disturbances(t,x,u,param,wave_param)

    z_n = x(1) + + param.z_n0;
    theta_s0_f = u(1);
    theta_s0_ap =u(2);
    theta_s0_as =u(3);

    l_x_f = param.l_xj_f + param.l_b*sin(param.gamma_0 - (param.l_s*theta_s0_f)...
            /param.h_h);
    l_x_ap = param.l_xj_ap + param.l_b*sin(param.gamma_0 - (param.l_s*theta_s0_ap)...
            /param.h_h);
    l_x_as = param.l_xj_as + param.l_b*sin(param.gamma_0 - (param.l_s*theta_s0_as)...
            /param.h_h);
    h_f = param.l_zj_f + z_n + param.l_b*cos(param.gamma_0 - (param.l_s*theta_s0_f)...
            /param.h_h);
    h_ap = param.l_zj_ap + z_n + param.l_b*cos(param.gamma_0 - (param.l_s*theta_s0_ap)...
            /param.h_h);
    h_as = param.l_zj_as + z_n + param.l_b*cos(param.gamma_0 - (param.l_s*theta_s0_as)...
            /param.h_h);


    % Inflow velocity in x-axis
    u_w_f = -wave_param.omega_0*wave_param.zeta_0*exp(-wave_param.k*h_f)*...
            sin(wave_param.k*l_x_f - wave_param.omega_e*t)*cos(wave_param.beta);
    u_w_ap = -wave_param.omega_0*wave_param.zeta_0*exp(-wave_param.k*h_ap)*....
            sin(wave_param.k*l_x_ap - wave_param.omega_e*t)*cos(wave_param.beta);
    u_w_as = -wave_param.omega_0*wave_param.zeta_0*exp(-wave_param.k*h_as)*...
            sin(wave_param.k*l_x_as - wave_param.omega_e*t)*cos(wave_param.beta);
    % Inflow velocity in z-axis
    w_w_f = -wave_param.omega_0*wave_param.zeta_0*exp(-wave_param.k*h_f)*...
            cos(wave_param.k*l_x_f - wave_param.omega_e*t);
    w_w_ap = -wave_param.omega_0*wave_param.zeta_0*exp(-wave_param.k*h_ap)*...
            cos(wave_param.k*l_x_ap - wave_param.omega_e*t);
    w_w_as = -wave_param.omega_0*wave_param.zeta_0*exp(-wave_param.k*h_as)*...
            cos(wave_param.k*l_x_as - wave_param.omega_e*t);

    d_DT = [u_w_f;w_w_f;u_w_ap;w_w_ap;u_w_as;w_w_as];

    foil_var_loc = [l_x_f;l_x_ap;l_x_as;h_f;h_ap;h_as];

end