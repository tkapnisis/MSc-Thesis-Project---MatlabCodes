% Define Parameters

% Enviromnent conditions
g = 9.81;      % Gravity acceleration [m/s2]
rho = 997;   % Water density at 25 Celcius [kg/m^3]

% Parameters of long-crested regular wave
omega_0 = 1.5;   % Wave frequency [rad/s]
lambda = 1;    % Wave length [m]
zeta_0 = 0.15;  % Wave amplitude [m]
beta = pi;     % Encounter angle (beta=0 for following waves) [rad] 

% Mass properties
m = 29.38;     % Mass kg 
I_x = 3.77;    % Moment of inertia around x- axis [kg*m^2]
I_xy = -0.32;  % Product of inertia around xy-axes [kg*m^2]
I_y = 7.60;    % Moment of inertia around y- axis [kg*m^2]
I_xz = 1.5370;
I_yz = 0.1679;

% Added mass coefficients
X_du = 0;     % Added mass coefficient in x-axis [kg]
Y_dv = 0;     % Added mass coefficient in y-axis [kg]
Z_dw = 0;     % Added mass coefficient in z-axis [kg]
K_dp = 0;     % Added mass coefficient for roll [kg*m^2]
M_dq = 0;     % Added mass coefficient for pitch [kg*m^2]
N_dr = 0;     % Added mass coefficient for yaw [kg*m^2]

% Operating conditions of HEARP
U_0 = 4;              % Operating cruise speed [m/s]
theta_s0_f = -0.062;  % Operating angle of servo motors for the fore hydrofoil
theta_s0_ap = -0.081; % Operating angle of servo motors for the aft port hydrofoil
theta_s0_as = -0.118; % Operating angle of servo motors for the aft starboard hydrofoil

z_n0 = -0.2; % Operating point of vertical position of CG with respect to 
             % mean water surface [m]

% Hydrofoils parameters
A_h = 0.036;         % Projected area of the hydrofoil [m^2]
C_L0 = 0.211;        % y-intercept of the linear approximation for CL [-]
C_La = 4.205;        % Slope of the linear approximation for CL [-]
C_D0 = 0.008;        % y-intercept of the linear approximation for CD [-]
C_Da = 0.380;        % Slope of the linear approximation for CL [-]

% Naca0012 foil
% C_L0 = 0;        % y-intercept of the linear approximation for CL [-]
% C_La = 2*pi;     % Slope of the linear approximation for CL [-]
% C_D0 = 0;        % y-intercept of the linear approximation for CD [-]
% C_Da = 2*pi/10;  % Slope of the linear approximation for CL [-]

l_s = 0.06;         % Moment arm of the servo motor [m] 
h_h = 0.178;         % z distance between the hinge and the joint of
                     % connection rod of the strut of T-foil [m]
l_b = 0.484;         % Distance between the centre of pressure of hydrofoil 
                     % and the hinge [m]
l_a0 = 0.473;        % z distance between the centre of pressure of 
                     % hydrofoil and the hinge for alpha_s =0 [m]
mu = acos(l_a0/l_b); % Angle between the joint of connection rod and the
                     % z axis of the centre of pressure of hydrofoil
                     % [rad]
l_xj_f = 0.402;      % x distance between the hinge of fore hydrofoil and 
                     % CG [m]                     
l_xj_ap = -0.298;    % x distance between the hinge of aft port hydrofoil
                     % and CG [m]
l_xj_as = -0.298;    % x distance between the hinge of aft starport 
                     % hydrofoil and CG [m]

l_y_f = -0.024;      % y distance between the centre of pressure of fore
                     % hydrofoil and CG [m]
l_y_ap = -0.481;     % y distance between the centre of pressure of aft 
                     % port hydrofoil and CG [m]
l_y_as = 0.433;      % y distance between the centre of pressure of aft 
                     % starport hydrofoil and CG [m]
l_zj_f = 0.006;      % z distance between the hinge of fore hydrofoil and 
                     % CG [m]
l_zj_ap = 0.006;     % z distance between the hinge of aft port hydrofoil  
                     % and CG [m]
l_zj_as = 0.006;     % z distance between the hinge of aft starport
                     % hydrofoil and CG [m]


%% %%%%%%%%%%%%%% Define all the parameters with a structure %%%%%%%%%%%%%%                   

% Enviromnent conditions
param.g = g;    % Gravity acceleration [m/s^2]
param.rho = rho;              % Water density at 25 Celcius [kg/m^3]

% Parameters of long-crested regular wave
param.omega_0 = omega_0;   % Wave frequency [rad/s]
param.lambda = lambda;    % Wave length [m]
param.zeta_0 = zeta_0;  % Wave amplitude [m]
param.beta = beta;     % Encounter angle (beta=0 for following waves) [rad] 

% Mass properties
param.m = m;    % Mass kg 
param.I_x = I_x; % Moment of inertia around x- axis [kg*m^2]
param.I_xy = I_xy;  % Product of inertia around xy-axes [kg*m^2]
param.I_y = I_y;    % Moment of inertia around y- axis [kg*m^2]
param.I_xz = I_xz;
param.I_yz = I_yz;

% Added mass coefficients
param.X_du = X_du;     % Added mass coefficient in x-axis [kg]
param.Y_dv = Y_dv;     % Added mass coefficient in y-axis [kg]
param.Z_dw = Z_dw;     % Added mass coefficient in z-axis [kg]
param.K_dp = K_dp;     % Added mass coefficient for roll [kg*m^2]
param.M_dq = M_dq;     % Added mass coefficient for pitch [kg*m^2]
param.N_dr = N_dr;     % Added mass coefficient for yaw [kg*m^2]

% Operating conditions of HEARP
param.U_0 = U_0;              % Operating cruise speed [m/s]
param.theta_s0_f = theta_s0_f;  % Operating angle of servo motors for the fore hydrofoil
param.theta_s0_ap = theta_s0_ap; % Operating angle of servo motors for the aft port hydrofoil
param.theta_s0_as = theta_s0_as; % Operating angle of servo motors for the aft starboard hydrofoil
param.z_n0 = z_n0; % Operating height of CG with respect to mean water surface [m]

% Hydrofoils parameters
param.A_h = A_h;         % Projected area of the hydrofoil [m^2]
param.C_L0 = C_L0;        % y-intercept of the linear approximation for CL [-]
param.C_La = C_La;        % Slope of the linear approximation for CL [-]
param.C_D0 = C_D0;        % y-intercept of the linear approximation for CD [-]
param.C_Da = C_Da;        % Slope of the linear approximation for CL [-]

param.l_s = l_s; % Moment arm of the servo motor [m] 
param.h_h = h_h; % z distance between the hinge and the joint of connection
                 % rod of the strut of T-foil [m]
param.l_b = l_b;     % Distance between the centre of pressure of hydrofoil 
                     % and the hinge [m]              
param.mu = mu; % Angle between the joint of connection rod and the z axis
               % of the centre of pressure of hydrofoil [rad]
param.l_xj_f = l_xj_f;   % x distance between the hinge of fore hydrofoil and 
                         % CG [m]                     
param.l_xj_ap = l_xj_ap; % x distance between the hinge of aft port hydrofoil
                         % and CG [m]
param.l_xj_as = l_xj_as; % x distance between the hinge of aft starboard 
                         % hydrofoil and CG [m]
param.l_y_f = l_y_f;     % y distance between the centre of pressure of fore
                         % hydrofoil and CG [m]
param.l_y_ap = l_y_ap;   % y distance between the centre of pressure of aft 
                         % port hydrofoil and CG [m]
param.l_y_as = l_y_as;   % y distance between the centre of pressure of aft 
                         % starboard hydrofoil and CG [m]
param.l_zj_f = l_zj_f;   % z distance between the hinge of fore hydrofoil and 
                         % CG [m]
param.l_zj_ap = l_zj_ap; % z distance between the hinge of aft port hydrofoil  
                         % and CG [m]
param.l_zj_as = l_zj_as; % z distance between the hinge of aft starboard
                         % hydrofoil and CG [m]      

save('Parameters_3DOF.mat')                         