% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

%% Defining the symbolic variables for the parameters and the states of the dynamical system
clc
clear all
close all

syms u v r w q p                        % Velocities for the 6 DOF model
syms z_n theta phi                      % Positions of the reduced 2 DOF model
% syms theta_s theta_s0                   % Input and operating point of input
syms h_0 U_0                            % Operating point of states
% syms theta_s0_f theta_s0_a              % Operating point of inputs
syms alpha_s alpha_s0                   % Input and operating point of input
syms alpha_s0_f alpha_s0_a              % Operating point of inputs
syms m I_x I_y I_z I_xy I_xz I_yz       % Mass and moment of inertia
syms X_du Y_dv Z_dw K_dp M_dq N_dr      % Added mass coefficients  
syms g rho                              % Environment parameters
syms A_h C_L0 C_La C_D0 C_Da            % Hydrofoil parameters
syms mu l_b l_s h_h                     % Hydrofoil actuation mechanism parameters
syms l_xj l_y l_zj l_xj_f l_y_f l_zj_f  % Hydrofoil distances
syms l_xj_a l_y_a l_zj_a                % Hydrofoil distances

% Velocities for the 6 DOF model
nu = [u; v; w; p; q; r];
% Location of the CG in relation to the CO (origin of the body frame)
x_g = 0;
y_g = 0;
z_g = 0;

% The reduced order system consists of 2 DOF namely the heave(z) and
% pitch(theta). So we use the following selection matrix to obtain the
% reduced order matrices from the 6 DOF matrices
L = [0 0 1 0 0 0; 0 0 0 0 1 0];

% States of the reduced order system: x=[z_n, theta, dz_n, dtheta]
x = [z_n, theta, w, q];
x_eq = [-h_0, 0, 0, 0]; % operating point around stable equilibrium

%% Mass inertia matrices

M_RB = [     m,      0,      0,      0,  m*z_g, -m*y_g;
             0,      m,      0, -m*z_g,      0,  m*x_g;
             0,      0,      m,  m*y_g, -m*x_g,      0;
             0, -m*z_g,  m*y_g,     I_x,   -I_xy,   -I_xz;
         m*z_g,      0, -m*x_g,   -I_xy,     I_y,   -I_yz;
        -m*y_g,  m*x_g,      0,    I_xz,   -I_yz,     I_z];

% Diagonal added mass matrix
M_A = -diag([X_du, Y_dv, Z_dw, K_dp, M_dq, N_dr]);

M_RB2 = L*M_RB*L';

M_A2 = L*M_A*L';

M_2_inv = inv(M_RB2 + M_A2);

% Linearization of Coriolis and centripetal matrices and hydrostatic matrix

% Rigid-body Coriolis and centripetal matrix and hydrodynamic Coriolis and
% centripetal matrix

C_RB = [0, 0, 0, m*(y_g*q+z_g*r) , -m*(x_g*q-w) , -m*(x_g*r+v);
		0 , 0 , 0 ,-m*(y_g*p+w) , m*(z_g*r+x_g*p) , -m*(y_g*r-u) ;
		0 , 0 , 0 ,-m*(z_g*p-v) , -m*(z_g*q+u) , m*(x_g*p+y_g*q) ;
		-m*(y_g*q + z_g*r) , m*(y_g*p + w) , m*(z_g*p - v), 0 , -I_yz*q - I_xz*p + I_z*r , I_yz*r + I_xy*p - I_y*q;
		m*(x_g*q - w) , -m*(z_g*r + x_g*p) , m*(z_g*q + u), I_yz*q + I_xz*p - I_z*r , 0 , -I_xz*r - I_xy*q + I_x*p;
		m*(x_g*r + v) , m*(y_g*r - u) , -m*(x_g*p + y_g*q),-I_yz*r - I_xy*p + I_y*q , I_xz*r + I_xy*q - I_x*p , 0];

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
C_RB2_nu2 = L*C_RB*nu;
% Substitution of the velocities of the 2 DOF which are not included in the
% reduced order system
C_RB2_nu2 = subs(C_RB2_nu2,[u, v, p, r],[U_0, 0, 0, 0]);

% Reduced-order vector for added masss Coriolis and centripetal forces
% vector
C_A2_nu2 = L*C_A*nu;
% Substitution of the velocities of the 2 DOF which are not included in the
% reduced order system
C_A2_nu2 = subs(C_A2_nu2,[u, v, p, r],[U_0, 0, 0, 0]);

% Reduced-order vector for hydrostatic forces
g_eta2 = L*g_eta;

% Combined Coriolis and centripetal matrices with negative signs because
% they are moved to right hand side of EOM
C_comb = -C_RB2_nu2 - C_A2_nu2;

% Linearization
% Partial derivatives about the states x
C_x = jacobian(C_comb,x);
g_x = jacobian(-g_eta2,x);
% Substidution of equilibrium point values
C_x = subs(C_x,x,x_eq);
g_x = subs(g_x,x,x_eq);

%% Linearization of control input forces for hydrofoils

% Swinging angle of the hydrofoil (pitching angle of the hydrofoil with
% respect to craft body) defined as the angle between the chord line of the
% hydrofoil and the x_f axis.

% alpha_s = -l_s/h_h*theta_s;   

% Total x distance distance between the centre of pressure of fore
% hydrofoil and CG
l_x = l_xj + sin(mu + alpha_s)*l_b;

% Total x distance distance between the centre of pressure of fore
% hydrofoil and CG
l_z = l_zj + cos(mu + alpha_s)*l_b;

% Effective angle of attack between the chord line of the hydrofoil and the
% water flow direction.
delta_u = q*l_z;
delta_w = w - q*l_x;
alpha_e = alpha_s + theta + atan(delta_w/(delta_u + U_0));

% Relative velocity of the inflow of water
V_inf = sqrt((delta_u + U_0)^2 + delta_w^2);

% Relative angle between the water flow direction and the x-axis of the
% hydrofoil
alpha_r = alpha_e - alpha_s;

% Lift force acting at the centre of pressure of the hydrofoil
F_L = 1/2*rho*V_inf^2*A_h*(C_L0 + C_La*alpha_e);

% Drag force acting at the centre of pressure of the hydrofoil
F_D = 1/2*rho*V_inf^2*A_h*(C_D0 + C_Da*alpha_e);

% Euler rotation about y-axis with an angle alpha_r
R_y_ar = [cos(alpha_r), 0 -sin(alpha_r);
          0, 0, 0;
          sin(alpha_r), 0, cos(alpha_r)];
% Vector of forces acting on a hydrofoil in the water flow direction
f_h = [-F_D; 0; -F_L];
% Vector of forces in BODY fixed coordinate frame
f_h_b = R_y_ar * f_h;
% Location of the hydrofoil with respect to the CG of the hydrofoil craft
r_i = [l_x; l_y; l_z];
% Vector of forces and moments due to hydrodynamic forces of a hydrofoil
% expressed in BODY frame
tau_i = [f_h_b; cross(r_i,f_h_b)];

% Reduced order vector for control input force vector
tau_i2 = L * tau_i;

% Linearization
% Partial derivative about x
tau_i_x = jacobian(tau_i2,x);
% tau_i_x = simplify(subs(tau_i_x,[x,theta_s],[x_eq,theta_s0]))
% tau_i_x = subs(tau_i_x,[x,theta_s],[x_eq,theta_s0]);
tau_i_x = subs(tau_i_x,[x,alpha_s],[x_eq,alpha_s0]);

% Partial derivative about u
% tau_i_u = jacobian(tau_i2,theta_s);
tau_i_u = jacobian(tau_i2,alpha_s);
% tau_i_u = simplify(subs(tau_i_u,[x,theta_s],[x_eq,theta_s0]))
% tau_i_u = subs(tau_i_u,[x,theta_s],[x_eq,theta_s0]);
tau_i_u = subs(tau_i_u,[x,alpha_s],[x_eq,alpha_s0]);
%%
% % Linearized control input vector force matrices for each hydrofoil
% tau_f_x = subs(tau_i_x,[l_xj,l_y, l_zj, theta_s0],[l_xj_f, l_y_f, l_zj_f, theta_s0_f]);
% tau_a_x = subs(tau_i_x,[l_xj,l_y, l_zj, theta_s0],[l_xj_a, l_y_a, l_zj_a, theta_s0_a]);

tau_f_x = subs(tau_i_x,[l_xj,l_y, l_zj, alpha_s0],[l_xj_f, l_y_f, l_zj_f, alpha_s0_f]);
tau_a_x = subs(tau_i_x,[l_xj,l_y, l_zj, alpha_s0],[l_xj_a, l_y_a, l_zj_a, alpha_s0_a]);

% tau_f_u = subs(tau_i_u,[l_xj,l_y, l_zj, theta_s0],[l_xj_f, l_y_f, l_zj_f, theta_s0_f]);
% tau_a_u = subs(tau_i_u,[l_xj,l_y, l_zj, theta_s0],[l_xj_a, l_y_a, l_zj_a, theta_s0_a]);

tau_f_u = subs(tau_i_u,[l_xj,l_y, l_zj, alpha_s0],[l_xj_f, l_y_f, l_zj_f, alpha_s0_f]);
tau_a_u = subs(tau_i_u,[l_xj,l_y, l_zj, alpha_s0],[l_xj_a, l_y_a, l_zj_a, alpha_s0_f]);
%% Linearized dynamical model with numerical values
% p=0;
run Define_Parameters_2DOF.m

M_2_inv_n = double(subs(M_2_inv));
C_x_n = double(subs(C_x));
tau_f_x_n = double(subs(tau_f_x));
tau_a_x_n = double(subs(tau_a_x));
tau_f_u_n = double(subs(tau_f_u));
tau_a_u_n = double(subs(tau_a_u));

A = [zeros(2,2), eye(2);
     M_2_inv_n*(C_x_n + tau_f_x_n + 2*tau_a_x_n)]

B = [zeros(2,2);
     M_2_inv_n*tau_f_u_n, M_2_inv_n*2*tau_a_u_n]

B_d = [zeros(2,2); M_2_inv_n];

C = [eye(2), zeros(2,2)];

D = zeros(2,2);

sys_ss = ss(A,B,C,D); % state space
Gp = zpk(sys_ss); % tranfer function matrix
Gp = minreal(Gp,eps);

% % Checking if the system is controlable
Controlability=rank(ctrb(A,B));
Observability=rank(obsv(A,C));

% MIMO poles and zeros of G
% eigenmodes = eig(A);
% poles = pole(Gp);
% zeros_Gp = tzero(Gp);
[ps,zs] = pzmap(sys_ss)


% sys_ss = ss(A,B,C,D); % state space
% states = {'z', 'theta', 'z_dot', 'theta_dot'};
% inputs = {'theta_s_f';'theta_s_a'};
% outputs = {'z'; 'theta'};
% 
% Gp = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs); % state space

%{
% Pole-zero map of the open-loop Gp(s)
figure;
pzmap(Gp);
grid on

% Open-loop Bode plot of Gp(s)
figure
opts = bodeoptions;
opts.MagScale = 'log';
opts.MagUnits = 'abs';
bodemag(zpk(Gp),opts);
grid on;
%}

save('Lineare_State_Space_2DOF.mat','sys_ss','A','B','C','D')