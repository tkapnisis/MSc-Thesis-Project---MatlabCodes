z_n0 = -0.29;
phi_0 = 0.0;
theta_0 = 0.0;
dz_n0 = 0;
dphi_0 = 0;
dtheta_0 = 0;

z_meas = -0.3023;
phi_meas = -0.0267;
theta_meas = 0.0172;
dz_meas = -0.0023;
dphi_meas = -0.0003;
dtheta_meas = 0.0002;


meas = [(z_meas - z_n0);(phi_meas - phi_0);(theta_meas - theta_0);(dz_meas - dz_n0);...
        (dphi_meas - dphi_0);(dtheta_meas - dtheta_0)];
u_eq = [-0.245;-0.279;-0.323];
ref = [0;0;0];
res = Lc_dt*ref - Kdt*meas + u_eq;
res_sat = min(0.5,max(res,-0.5));
res_servo_min = -1.57;
res_servo_max = 1.57;
res_map = (res_sat - res_servo_min)*2/(res_servo_max - res_servo_min) - 1