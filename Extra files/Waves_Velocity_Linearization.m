clc
clear all
close all

syms omega zeta_a k h xf omega_e t
syms l_xj l_zj mu alpha_s l_b
syms l_x l_y l_z
syms z_n phi theta w p q z_n0 psi

x = [z_n, phi, theta, w, p, q];
x_eq = [z_n0, 0, 0, 0, 0, 0]; % operating point around stable equilibrium

% l_x_f = l_xj + sin(mu + alpha_s )*l_b;

% l_z_f = l_zj + cos(mu + alpha_s )*l_b;

R_phi = [1    0         0    ;...   % roll (x)
         0 cos(phi) -sin(phi);...
         0 sin(phi)  cos(phi)];
R_theta = [ cos(theta) 0 sin(theta);...  % pitch (y)
                0      1     0     ;...
           -sin(theta) 0 cos(theta)];
R_psi = [cos(psi) -sin(psi) 0;...  % yaw (z)
         sin(psi)  cos(psi) 0;...
           0         0     1];
R_BODY_NED= (R_psi*R_theta*R_phi); % Rotation matrix from BODY to NED

% Location of the hydrofoil with respect to the CG of the hydrofoil craft
r_i = [l_x; l_y; l_z];

% Submergence of each hydrofoil
h_i = z_n + R_BODY_NED(3,:)*r_i;

%%
% sin(a - b) = sin(a)*cos(b) - cos(a)*sin(b);
% cos(a - b) = cos(a)*cos(b) - sin(a)*sin(b);

% u_w = omega*zeta_a*exp(-k*h)*sin(k*l_x_f - omega_e*t);
% w_w = omega*zeta_a*exp(-k*h)*cos(k*l_x_f - omega_e*t);

syms d1 d2
% d1 = sin(omega_e*t);
% d2 = cos(omega_e*t);

u_w = omega*zeta_a*exp(-k*h_i)*(sin(k*l_x)*d2 - cos(k*l_x)*d1);
w_w = omega*zeta_a*exp(-k*h_i)*(cos(k*l_x)*d2 - sin(k*l_x)*d1);

u_w_x = jacobian(u_w,x);
w_w_x = jacobian(w_w,x);

u_w_x = subs(u_w_x,[x,d1,d2],[x_eq,0,0]);
w_w_x = subs(u_w_x,[x,d1,d2],[x_eq,0,0]);

u_w_d = jacobian(u_w,[d1, d2]);
w_w_d = jacobian(w_w,[d1, d2]);

u_w_d = subs(u_w_d,[x,d1,d2],[x_eq,0,0])
w_w_d = subs(u_w_d,[x,d1,d2],[x_eq,0,0])

%%
% [k*omega*zeta_a*exp(-k*(z_n - l_x_f*sin(theta) + l_z_f*cos(phi)*cos(theta) + l_y_f*cos(theta)*sin(phi)))*(d1*cos(k*l_x_f) - d2*sin(k*l_x_f)), 
%  k*omega*zeta_a*exp(-k*(z_n - l_x_f*sin(theta) + l_z_f*cos(phi)*cos(theta) + l_y_f*cos(theta)*sin(phi)))*(d1*cos(k*l_x_f) - d2*sin(k*l_x_f))*(l_y_f*cos(phi)*cos(theta) - l_z_f*cos(theta)*sin(phi)), 
% -k*omega*zeta_a*exp(-k*(z_n - l_x_f*sin(theta) + l_z_f*cos(phi)*cos(theta) + l_y_f*cos(theta)*sin(phi)))*(d1*cos(k*l_x_f) - d2*sin(k*l_x_f))*(l_x_f*cos(theta) + l_z_f*cos(phi)*sin(theta) + l_y_f*sin(phi)*sin(theta)), 0, 0, 0]
% 

% Trigonometry sum and difference identities
% sin(a - b) = sin(a)*cos(b) - cos(a)*sin(b);
% cos(a - b) = cos(a)*cos(b) - sin(a)*sin(b);
% sin(a + b) = sin(a)*cos(b) + cos(a)*sin(b);
% cos(a + b) = cos(a)*cos(b) + sin(a)*sin(b);

% Inflow velocities
% u_w = omega*zeta_a*exp(-k*h)*sin(k*l_x - omega_e*t);
% w_w = omega*zeta_a*exp(-k*h)*cos(k*l_x - omega_e*t);

% Definitions of the disturbances which represent the sinusoidal parts of
% the inflow velocities and depend on the time
% d1 = sin(omega_e*t);
% d2 = cos(omega_e*t);

k = 2*pi/lambda; % Wave number

% Inflow velocity in x-axis
u_w = omega_0*zeta_0*exp(-k*h_i)*(sin(k*l_x)*d2 - cos(k*l_x)*d1);
% Inflow velocity in z-axis
w_w = omega_0*zeta_0*exp(-k*h_i)*(cos(k*l_x)*d2 - sin(k*l_x)*d1);
