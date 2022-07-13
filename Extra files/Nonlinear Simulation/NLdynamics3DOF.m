% function [dxdt] = NLdynamics3DOF(t, x, u, param, wave_param, dxdt_sym)
function [dxdt] = NLdynamics3DOF(t, x, param, wave_param, dxdt_s, K, Lc)

dxdt = zeros(6,1);
ref = [-0.1;0;0];
u = Lc*ref- K*x

% Enviromnent conditions
g = param.g;    % Gravity acceleration [m/s^2]
rho = param.rho;              % Water density at 25 Celcius [kg/m^3]

% Parameters of long-crested regular wave
omega_0 = param.omega_0;   % Wave frequency [rad/s]
lambda = param.lambda;    % Wave length [m]
zeta_0 = param.zeta_0;  % Wave amplitude [m]
beta = param.beta;     % Encounter angle (beta=0 for following waves) [rad] 

% Mass properties
m = param.m;    % Mass kg 
I_x = param.I_x; % Moment of inertia around x- axis [kg*m^2]
I_xy = param.I_xy;  % Product of inertia around xy-axes [kg*m^2]
I_y = param.I_y;    % Moment of inertia around y- axis [kg*m^2]
I_xz = param.I_xz;
I_yz = param.I_yz;

% Added mass coefficients
X_du = param.X_du;     % Added mass coefficient in x-axis [kg]
Y_dv = param.Y_dv;     % Added mass coefficient in y-axis [kg]
Z_dw = param.Z_dw;     % Added mass coefficient in z-axis [kg]
K_dp = param.K_dp;     % Added mass coefficient for roll [kg*m^2]
M_dq = param.M_dq;     % Added mass coefficient for pitch [kg*m^2]
N_dr = param.N_dr;     % Added mass coefficient for yaw [kg*m^2]

% Operating conditions of HEARP
U_0 = param.U_0;                 % Operating cruise speed [m/s]
z_n0 = param.z_n0; % Operating height of CG with respect to mean water surface [m]
theta_s0_f = param.theta_s0_f;   % Operating angle of servo motors for the fore hydrofoil
theta_s0_ap = param.theta_s0_ap; % Operating angle of servo motors for the aft port hydrofoil
theta_s0_as = param.theta_s0_as; % Operating angle of servo motors for the aft starboard hydrofoil

% Hydrofoils parameters
A_h = param.A_h;         % Projected area of the hydrofoil [m^2]
C_L0 = param.C_L0;        % y-intercept of the linear approximation for CL [-]
C_La = param.C_La;        % Slope of the linear approximation for CL [-]
C_D0 = param.C_D0;        % y-intercept of the linear approximation for CD [-]
C_Da = param.C_Da;        % Slope of the linear approximation for CL [-]

l_s = param.l_s; % Moment arm of the servo motor [m] 
h_h = param.h_h; % z distance between the hinge and the joint of connection
                 % rod of the strut of T-foil [m]
l_b = param.l_b;     % Distance between the centre of pressure of hydrofoil 
                     % and the hinge [m]              
mu = param.mu; % Angle between the joint of connection rod and the z axis
               % of the centre of pressure of hydrofoil [rad]
l_xj_f = param.l_xj_f;   % x distance between the hinge of fore hydrofoil and 
                         % CG [m]                     
l_xj_ap = param.l_xj_ap; % x distance between the hinge of aft port hydrofoil
                         % and CG [m]
l_xj_as = param.l_xj_as; % x distance between the hinge of aft starboard 
                         % hydrofoil and CG [m]
l_y_f = param.l_y_f;     % y distance between the centre of pressure of fore
                         % hydrofoil and CG [m]
l_y_ap = param.l_y_ap;   % y distance between the centre of pressure of aft 
                         % port hydrofoil and CG [m]
l_y_as = param.l_y_as;   % y distance between the centre of pressure of aft 
                         % starboard hydrofoil and CG [m]
l_zj_f = param.l_zj_f;   % z distance between the hinge of fore hydrofoil and 
                         % CG [m]
l_zj_ap = param.l_zj_ap; % z distance between the hinge of aft port hydrofoil  
                         % and CG [m]
l_zj_as = param.l_zj_as; % z distance between the hinge of aft starboard
                         % hydrofoil and CG [m]

% Current states of the system                         
z_n = x(1);
phi = x(2);
theta = x(3);
w = x(4);
p = x(5);
q = x(6);

% Control input
theta_s_f = u(1);
theta_s_ap = u(2);
theta_s_as = u(3);

% % Calculations of wave velocities for each hydrofoil
% k = 2*pi/lambda; % Wave number
% omega_e = omega_0 - omega_0^2/g*U_0*cos(beta); % encounter frequency
% 
% Locations and submergence of each hydrofoil
% l_x_f = double(subs(wave_param.l_x_f));
% l_x_ap = double(subs(wave_param.l_x_ap));
% l_x_as = double(subs(wave_param.l_x_as));
% h_f = double(subs(wave_param.h_f));
% h_ap = double(subs(wave_param.h_ap));
% h_as = double(subs(wave_param.h_as));
% 
% Inflow velocity in x-axis
% u_w_f = omega_0*zeta_0*exp(-k*h_f)*sin(k*l_x_f - omega_e*t)*0;
% u_w_ap = omega_0*zeta_0*exp(-k*h_ap)*sin(k*l_x_ap - omega_e*t)*0;
% u_w_as = omega_0*zeta_0*exp(-k*h_as)*sin(k*l_x_as - omega_e*t)*0;
% Inflow velocity in z-axis
% w_w_f = omega_0*zeta_0*exp(-k*h_f)*cos(k*l_x_f - omega_e*t)*0;
% w_w_ap = omega_0*zeta_0*exp(-k*h_ap)*cos(k*l_x_ap - omega_e*t)*0;
% w_w_as = omega_0*zeta_0*exp(-k*h_as)*cos(k*l_x_as - omega_e*t)*0;

u_w_f = 0;
u_w_ap = 0;
u_w_as = 0;
% Inflow velocity in z-axis
w_w_f = 0;
w_w_ap = 0;
w_w_as = 0;

% dxdt_s = arrayfun(@char, dxdt_sym, 'UniformOutput', 0);
% dxdt_s = string(dxdt_s);

% for i=1:size(dxdt_sym,1)
for i=1:size(dxdt_s,1)
    dxdt(i) = eval(dxdt_s(i));
end

end