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
syms X_du Y_dv Z_dw K_dp M_dq N_dr      % Added mass coefficients  
syms g rho                              % Environment parameters
syms A_h C_L0 C_La C_D0 C_Da            % Hydrofoil parameters
syms mu l_b l_s h_h                     % Hydrofoil actuation mechanism parameters
syms l_xj l_y l_zj l_xj_f l_y_f l_zj_f  % Hydrofoil distances
syms l_xj_ap l_y_ap l_zj_ap l_xj_as l_y_as l_zj_as % Hydrofoil distances
syms u_w w_w u_w_f w_w_f u_w_ap w_w_ap u_w_as w_w_as  % Wave disturvances

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

% Added mass inertia matrix (Diagonal)
M_A = -diag([X_du, Y_dv, Z_dw, K_dp, M_dq, N_dr]);

% Reduced-order rigid-body inertia matrix
M_RB3 = L*M_RB*L';

% Reduced-order added mass inertia matrix
M_A3 = L*M_A*L';

% Inverse of the compined inertia matrices
M_3_inv = inv(M_RB3 + M_A3);

% Rigid-body Coriolis and centripetal matrix
C_RB = [0, 0, 0, m*(y_g*q+z_g*r) , -m*(x_g*q-w) , -m*(x_g*r+v);
		0 , 0 , 0 ,-m*(y_g*p+w) , m*(z_g*r+x_g*p) , -m*(y_g*r-u) ;
		0 , 0 , 0 ,-m*(z_g*p-v) , -m*(z_g*q+u) , m*(x_g*p+y_g*q) ;
		-m*(y_g*q + z_g*r) , m*(y_g*p + w) , m*(z_g*p - v), 0 , -I_yz*q - I_xz*p + I_z*r , I_yz*r + I_xy*p - I_y*q;
		m*(x_g*q - w) , -m*(z_g*r + x_g*p) , m*(z_g*q + u), I_yz*q + I_xz*p - I_z*r , 0 , -I_xz*r - I_xy*q + I_x*p;
		m*(x_g*r + v) , m*(y_g*r - u) , -m*(x_g*p + y_g*q),-I_yz*r - I_xy*p + I_y*q , I_xz*r + I_xy*q - I_x*p , 0];

% Added mass Coriolis and centripetal matrix
C_A = [      0,       0,       0,       0, -Z_dw*w,  Y_dv*v;
	         0,       0,       0,  Z_dw*w,       0, -X_du*u;
	         0,       0,       0, -Y_dv*v,  X_du*u,       0;
	         0, -Z_dw*w,  Y_dv*v,       0, -N_dr*r,  M_dq*q;
	    Z_dw*w,       0, -X_du*u,  N_dr*r,       0, -K_dp*p;
	   -Y_dv*v,  X_du*u,       0, -M_dq*q,  K_dp*p,       0];

% Hydrostatic force vector
g_eta = [m*g*sin(theta); -m*g*sin(phi)*cos(phi); -m*g*cos(phi)*cos(theta);0; 0; 0];

% Reduced-order vector for Rigid-body Coriolis and centripetal forces
% vector
C_RB3_nu3 = L*C_RB*nu;

% Substitution of the velocities of the 3 DOF which are not included in the
% reduced order system
C_RB3_nu3 = subs(C_RB3_nu3,[u, v, r],[U_0, 0, 0]);

% Reduced-order vector for added masss Coriolis and centripetal forces
% vector
C_A3_nu3 = L*C_A*nu;

% Substitution of the velocities of the 3 DOF which are not included in the
% reduced order system
C_A3_nu3 = subs(C_A3_nu3,[u, v, r],[U_0, 0, 0]);

% Reduced-order vector for hydrostatic forces
g_eta3 = L*g_eta;

% Combined Coriolis and centripetal matrices with negative signs because
% they are moved to right hand side of EOM
C_comb = -C_RB3_nu3 -C_A3_nu3;

% Linearization
% Partial derivatives about the states x
C_x = jacobian(C_comb,x);
g_x = jacobian(-g_eta3,x);

% Substidution of equilibrium point values
C_x = subs(C_x,x,x_eq);
g_x = subs(g_x,x,x_eq);

%% Linearization of hydrodynamic forces for hydrofoils

% Swinging angle of the hydrofoil (pitching angle of the hydrofoil with
% respect to craft body) defined as the angle between the chord line of the
% hydrofoil and the x_f axis.
alpha_s = -l_s/h_h*theta_s;   

% Total x distance between the centre of pressure of each hydrofoil and CG
l_x = l_xj + sin(mu + alpha_s )*l_b;

% Total x distance between the centre of pressure of each hydrofoil and CG
l_z = l_zj + cos(mu + alpha_s )*l_b;

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

% Submergence of each hydrofoil
h_i = z_n + R_BODY_NED(3,:)*r_i;

% Velocity pertibations due to the craft's motions and waves
delta_u = q*l_z + u_w;
delta_w = w + p*l_y - q*l_x + w_w;

% Effective angle of attack between the chord line of the hydrofoil and the
% water inflow direction.
alpha_e = alpha_s + theta + atan(delta_w/(delta_u + U_0));

% Relative velocity of the water inflow
V_inf = sqrt((delta_u + U_0)^2 + delta_w^2);

% Relative angle between the water flow direction and the x-axis of the
% hydrofoil
alpha_r = alpha_e - alpha_s;

% Lift force acting at the centre of pressure of the hydrofoil
F_L = 1/2*rho*V_inf^2*A_h*(C_L0 + C_La*alpha_e);

% Drag force acting at the centre of pressure of the hydrofoil
F_D = 1/2*rho*V_inf^2*A_h*(C_D0 + C_Da*alpha_e);

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

% Linearization
% Partial derivative about x
tau_i_x = jacobian(tau_i3,x);
tau_i_x = subs(tau_i_x,[x,theta_s,u_w,w_w],[x_eq,theta_s0,0,0]);

% Partial derivative about u
tau_i_u = jacobian(tau_i3,theta_s);
tau_i_u = subs(tau_i_u,[x,theta_s,u_w,w_w],[x_eq,theta_s0,0,0]);

% Partial derivative about wave disturbances
tau_i_w = jacobian(tau_i3,[u_w,w_w]);
tau_i_w = subs(tau_i_w,[x,theta_s,u_w,w_w],[x_eq,theta_s0,0,0]);

% Parameters that are different for each hydrofoil
h_param = [l_xj,l_y, l_zj, theta_s0];
h_param_f = [l_xj_f, l_y_f, l_zj_f, theta_s0_f]; 
h_param_ap = [l_xj_ap, l_y_ap, l_zj_ap, theta_s0_ap];
h_param_as = [l_xj_as, l_y_as, l_zj_as, theta_s0_as];

% % Linearized hydrodynamic force matrices for each hydrofoil
tau_f_x = subs(tau_i_x,h_param,h_param_f);
tau_ap_x = subs(tau_i_x,h_param,h_param_ap);
tau_as_x = subs(tau_i_x,h_param,h_param_as);

tau_f_u = subs(tau_i_u,h_param,h_param_f);
tau_ap_u = subs(tau_i_u,h_param,h_param_ap);
tau_as_u = subs(tau_i_u,h_param,h_param_as);

tau_f_w = subs(tau_i_w,h_param,h_param_f);
tau_ap_w = subs(tau_i_w,h_param,h_param_ap);
tau_as_w = subs(tau_i_w,h_param,h_param_as);

% Linearized locations and submergence for each hydrofoil
% h_param = [l_xj,l_y, l_zj, theta_s];
% Parameters that are different for each hydrofoil
h_param = [l_xj,l_y, l_zj, theta_s, u_w, w_w];
h_param_f = [l_xj_f, l_y_f, l_zj_f, theta_s_f, u_w_f, w_w_f]; 
h_param_ap = [l_xj_ap, l_y_ap, l_zj_ap, theta_s_ap, u_w_ap, w_w_ap];
h_param_as = [l_xj_as, l_y_as, l_zj_as, theta_s_as, u_w_as, w_w_as];

l_x_f = subs(l_x,h_param,h_param_f);
l_x_ap = subs(l_x,h_param,h_param_ap);
l_x_as = subs(l_x,h_param,h_param_as);
h_f = subs(h_i,[h_param,x],[h_param_f,x_eq]);
h_ap = subs(h_i,[h_param,x],[h_param_ap,x_eq]);
h_as = subs(h_i,[h_param,x],[h_param_as,x_eq]);

%% Nonlinear dynamical state space

% Parameters that are different for each hydrofoil
h_param = [l_xj,l_y, l_zj, theta_s, u_w, w_w];
h_param_f = [l_xj_f, l_y_f, l_zj_f, theta_s_f, u_w_f, w_w_f]; 
h_param_ap = [l_xj_ap, l_y_ap, l_zj_ap, theta_s_ap, u_w_ap, w_w_ap];
h_param_as = [l_xj_as, l_y_as, l_zj_as, theta_s_as, u_w_as, w_w_as];


tau_f3 = subs(tau_i3,h_param,h_param_f);
tau_ap3 = subs(tau_i3,h_param,h_param_ap);
tau_as3 = subs(tau_i3,h_param,h_param_as);

dxdt_sym = [w; p; q;
        M_3_inv*(-C_RB3_nu3 -C_A3_nu3) - g_eta3 + tau_f3 + tau_ap3 + tau_as3];

wave_param.l_x_f = l_x_f;
wave_param.l_x_ap = l_x_ap;
wave_param.l_x_as = l_x_as;
wave_param.h_f = h_f;
wave_param.h_ap = h_ap;
wave_param.h_as = h_as;

save('NL_State_Space_3DOF_Sym.mat','dxdt_sym','wave_param')
%% Linearized dynamical model with symbolic variables
%{
A_s = [zeros(3,3), eye(3); M_3_inv*(C_x + tau_f_x + tau_ap_x + tau_as_x)];

B_s = [zeros(3,3); M_3_inv*[tau_f_u, tau_ap_u, tau_as_u]];

B_d_s = [zeros(3,6); M_3_inv*[tau_f_w, tau_ap_w, tau_as_w]];

C_s = [eye(3), zeros(3,3)];

D_s = zeros(3,3);

save('Linear_State_Space_3DOF_Sym.mat','A_s','B_s','B_d_s','C_s','D_s')
%}
%% Linearized dynamical model with numerical values

load('Parameters_3DOF.mat')

M_3_inv_n = double(subs(M_3_inv));
C_x_n = double(subs(C_x));

tau_f_x_n = double(subs(tau_f_x));
tau_ap_x_n = double(subs(tau_ap_x));
tau_as_x_n = double(subs(tau_as_x));

tau_f_u_n = double(subs(tau_f_u));
tau_ap_u_n = double(subs(tau_ap_u));
tau_as_u_n = double(subs(tau_as_u));

tau_f_w_n = double(subs(tau_f_w));
tau_ap_w_n = double(subs(tau_ap_w));
tau_as_w_n = double(subs(tau_as_w));

wave_param.l_x_f = double(subs(l_x_f));
wave_param.l_x_ap = double(subs(l_x_ap));
wave_param.l_x_as = double(subs(l_x_as));
wave_param.h_f = double(subs(h_f));
wave_param.h_ap = double(subs(h_ap));
wave_param.h_as = double(subs(h_as));

A = [zeros(3,3), eye(3);
     M_3_inv_n*(C_x_n + tau_f_x_n + tau_ap_x_n + tau_as_x_n)];

B = [zeros(3,3); M_3_inv_n*[tau_f_u_n, tau_ap_u_n, tau_as_u_n]];

B_d = [zeros(3,6); M_3_inv_n*[tau_f_w_n, tau_ap_w_n, tau_as_w_n]];

C = [eye(3), zeros(3,3)];

D = zeros(3,3);

D_d = zeros(3,6);

G = ss(A,B,C,D); % state space of the nominal plant
Gd = ss(A,B_d,C,D_d); % state space of the nominal plant

% Gp = zpk(sys_ss); % tranfer function matrix
% Gp = minreal(Gp);

% Checking if the system is controlable
Controlability=rank(ctrb(A,B));
Observability=rank(obsv(A,C));

% MIMO poles and zeros of G
ps = pole(G);
zs = tzero(G);
% pzmap(sys_ss)
% grid on

save('Linear_State_Space_3DOF.mat','G','Gd','A','B','B_d','C','D','D_d',...
                                   'wave_param','U_0','z_n0')
% save('Linear_State_Space_3DOF.mat','G','Gd')
%% Modelling of height sensor
%{
syms l_x_hs l_y_hs l_z_hs x_hs y_hs z_hs psi theta phi
r_b_hs = [l_x_hs;l_y_hs;l_z_hs];

% S = Smtrx(a) computes the 3x3 vector skew-symmetric matrix S(a) = -S(a)'.
S = [    0      -r_b_hs(3)   r_b_hs(2);
      r_b_hs(3)     0       -r_b_hs(1);
     -r_b_hs(2)  r_b_hs(1)     0     ];

H_inv = [eye(3) S; zeros(3,3) eye(3)];

eta = H_inv*[x_hs; y_hs; z_hs; phi; theta; psi];

syms x_n y_n z_n ds

p_nb_n = [x_n; y_n; z_n;];
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
R_BODY_NED= (R_psi*R_theta*R_phi); % Use transpose to take into account 
                                    % that z-axis looks upwards

p_nhs_n = p_nb_n + R_BODY_NED*r_b_hs;

ds = -p_nhs_n(3);

l_z_hs = -0.13;
l_x_hs = 0.7;
l_y_hs = 0;
theta = 0.20;
phi = 0.05;
z_n = -0.3;
h_0 = 0.2;

x_cu = [z_n, phi, theta, 0, 0, 0];
x_eq_val = double(subs(x_eq));

ds_l = jacobian(ds,x);

% Substidution of equilibrium point values
ds_l = subs(ds_l,x,x_eq);

ds_l_val = double(subs(ds_l));

ds_eq = subs(ds,x,x_eq);

ds_eq_val  =double(subs(ds_eq));

ds_lin_val = ds_eq_val + ds_l_val*(x_cu' - x_eq_val')

ds_nl_val = double(subs(ds))
%}