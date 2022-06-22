% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

%% Defining the symbolic variables for the parameters and the states of the dynamical system
clc
clear all
close all

syms u v r w q p                        % Velocities for the 6 DOF model
syms x_n y_n z_n phi theta psi          % Positions for the 6 DOF model
syms theta_s theta_s_f theta_s_ap theta_s_as  % Inputs
syms z_n0 U_0                           % Operating point of states
syms theta_s0 theta_s0_f theta_s0_ap theta_s0_as % Operating point of inputs
syms m I_x I_y I_z I_xy I_xz I_yz       % Mass and moment of inertia
syms g rho                              % Environment parameters
syms A_h C_L0 C_La C_D0 C_Da            % Hydrofoil parameters
syms gamma_0 l_b l_s h_h                     % Hydrofoil actuation mechanism parameters
syms l_xj l_y l_zj l_xj_f l_y_f l_zj_f  % Hydrofoil distances
syms l_xj_ap l_y_ap l_zj_ap l_xj_as l_y_as l_zj_as % Hydrofoil distances
syms u_w w_w u_w_f w_w_f u_w_ap w_w_ap u_w_as w_w_as  % Wave disturvances

% syms chord m_AF b_AF
% Velocities for the 6 DOF model
nu = [u; v; w; p; q; r];
% Location of the CG in relation to the CO (origin of the body frame)
x_g = 0;
y_g = 0;
z_g = 0;

% The reduced order system consists of 3 DOF namely the heave(z), roll(phi)
% and pitch(theta). So we use the following selection matrix to obtain the
% reduced order matrices from the 6 DOF matrices
L = [zeros(3,2), eye(3), zeros(3,1)];
% States of the reduced order system: x=[z_n, phi, theta, dz_n, dphi, dtheta]
x = [z_n, phi, theta, w, p, q];
x_eq = [z_n0, 0, 0, 0, 0, 0]; % operating point around stable equilibrium

%% Mass inertia matrices

% Rigid-body inertia matrix
M_RB = [     m,      0,      0,      0,  m*z_g, -m*y_g;
             0,      m,      0, -m*z_g,      0,  m*x_g;
             0,      0,      m,  m*y_g, -m*x_g,      0;
             0, -m*z_g,  m*y_g,     I_x,   -I_xy,   -I_xz;
         m*z_g,      0, -m*x_g,   -I_xy,     I_y,   -I_yz;
        -m*y_g,  m*x_g,      0,    I_xz,   -I_yz,     I_z];

% Reduced-order rigid-body inertia matrix
M_RB3 = L*M_RB*L';

% Inverse of the compined inertia matrices
M_3_inv = inv(M_RB3);

% Rigid-body Coriolis and centripetal matrix
C_RB = [0, 0, 0, m*(y_g*q+z_g*r) , -m*(x_g*q-w) , -m*(x_g*r+v);
		0 , 0 , 0 ,-m*(y_g*p+w) , m*(z_g*r+x_g*p) , -m*(y_g*r-u) ;
		0 , 0 , 0 ,-m*(z_g*p-v) , -m*(z_g*q+u) , m*(x_g*p+y_g*q) ;
		-m*(y_g*q + z_g*r) , m*(y_g*p + w) , m*(z_g*p - v), 0 , -I_yz*q - I_xz*p + I_z*r , I_yz*r + I_xy*p - I_y*q;
		m*(x_g*q - w) , -m*(z_g*r + x_g*p) , m*(z_g*q + u), I_yz*q + I_xz*p - I_z*r , 0 , -I_xz*r - I_xy*q + I_x*p;
		m*(x_g*r + v) , m*(y_g*r - u) , -m*(x_g*p + y_g*q),-I_yz*r - I_xy*p + I_y*q , I_xz*r + I_xy*q - I_x*p , 0];

% Hydrostatic force vector
g_eta = [m*g*sin(theta); -m*g*sin(phi)*cos(phi); -m*g*cos(phi)*cos(theta);0; 0; 0];

% Reduced-order vector for Rigid-body Coriolis and centripetal forces
% vector
C_RB3_nu3 = L*C_RB*nu;

% Substitution of the velocities of the 3 DOF which are not included in the
% reduced order system
C_RB3_nu3 = subs(C_RB3_nu3,[u, v, r],[U_0, 0, 0]);

% Reduced-order vector for hydrostatic forces
g_eta3 = L*g_eta;

%% Hydrodynamic forces for hydrofoils

% Swinging angle of the hydrofoil (pitching angle of the hydrofoil with
% respect to craft body) defined as the angle between the chord line of the
% hydrofoil and the x_f axis.
alpha_s = -l_s/h_h*theta_s;   

% Total x distance between the centre of pressure of each hydrofoil and CG
l_x = l_xj + sin(gamma_0 + alpha_s )*l_b;

% Total x distance between the centre of pressure of each hydrofoil and CG
l_z = l_zj + cos(gamma_0 + alpha_s )*l_b;

% Rotation Matrix from Body Frame to NED Frame
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

% Velocity pertibations due to the craft's motions and waves
delta_u = q*l_z;
delta_w = w + p*l_y - q*l_x;

% Effective angle of attack between the chord line of the hydrofoil and the
% water inflow direction.
alpha_e = alpha_s + theta + atan((delta_w + w_w)/(delta_u + U_0 + u_w));
    
% Relative velocity of the water inflow
V_inf = sqrt((delta_u + U_0 + u_w)^2 + (delta_w + w_w)^2);

% Relative angle between the water flow direction and the x-axis of the
% hydrofoil
alpha_r = alpha_e - alpha_s;

% Attenuating factor of lift coeffient depending on the depth
% AF = m_AF*z_n/chord + b_AF;
% Lift force acting at the centre of pressure of the hydrofoil
F_L = 1/2*rho*V_inf^2*A_h*(C_L0 + C_La*alpha_e);

% Drag force acting at the centre of pressure of the hydrofoil
F_D = 1/2*rho*V_inf^2*A_h*(C_D0 + C_Da*alpha_e);
% syms alpha_r F_D F_L
% Euler rotation matrix about y-axis with an angle alpha_r
R_y_ar = [cos(alpha_r), 0 -sin(alpha_r);
          0, 0, 0;
          sin(alpha_r), 0, cos(alpha_r)];

% Vector of forces acting on a hydrofoil in the water flow direction
f_h = [-F_D; 0; -F_L];

% Vector of forces in BODY fixed coordinate frame
f_h_b = R_y_ar * f_h;

% Vector of forces and moments due to hydrodynamic forces of a hydrofoil
% expressed in BODY frame
tau_i = [f_h_b; cross(r_i,f_h_b)];

% Reduced-order vector for hydrodynamic force vector of each hydrofoil
tau_i3 = L * tau_i;

tau_f3 = subs(tau_i3,[l_xj,l_y, l_zj, theta_s],[l_xj_f, l_y_f, l_zj_f, theta_s_f]);
tau_ap3 = subs(tau_i3,[l_xj,l_y, l_zj, theta_s],[l_xj_ap, l_y_ap, l_zj_ap, theta_s_ap]);
tau_as3 = subs(tau_i3,[l_xj,l_y, l_zj, theta_s],[l_xj_as, l_y_as, l_zj_as, theta_s_as]);

%% Non linear state space
xdot = [M_3_inv * (-C_RB3_nu3 - g_eta3 + tau_f3 + tau_ap3 + tau_as3)];

xdot_xeq_sym = subs(xdot,[x,u_w,w_w],[x_eq,0,0]);

load Parameters_3DOF.mat

xdot_xeq = subs(xdot_xeq_sym);
xdot_xeq_eq = xdot_xeq == 0;

u_eq = vpasolve(xdot_xeq_eq, [theta_s_f, theta_s_ap, theta_s_as]);

theta_s_f0 = double(u_eq.theta_s_f)
alpha_s_f0 = rad2deg(-l_s/h_h*theta_s_f0)

theta_s_ap0 = double(u_eq.theta_s_ap)
alpha_s_ap0 = rad2deg(-l_s/h_h*theta_s_ap0)

theta_s_as0 = double(u_eq.theta_s_as)
alpha_s_as0 = rad2deg(-l_s/h_h*theta_s_as0)

save('Equilibrium_input_3DOF.mat','theta_s_f0','alpha_s_f0',...
     'theta_s_ap0','alpha_s_ap0','theta_s_as0','alpha_s_as0')