% Define Parameters

% Mass properties
m = 29.38;    % Mass kg 
g = 9.81;
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
theta_s0_a = -0.081; % Operating angle of servo motors for the aft port hydrofoil

h_0 = -0.2;             % Operating height of CG with respect to mean water 
                       % surface [m]
rho = 997;              % Water density at 25 Celcius [kg/m^3]

% Hydrofoils parameters
A_h = 0.036;         % Projected area of the hydrofoil [m^2]
C_L0 = 0.211;        % y-intercept of the linear approximation for CL [-]
C_La = 4.205;        % Slope of the linear approximation for CL [-]
C_D0 = 0.008;        % y-intercept of the linear approximation for CD [-]
C_Da = 0.380;        % Slope of the linear approximation for CL [-]
l_s = 0.06;          % Moment arm of the servo motor [m] 
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
l_xj_a = -0.298;     % x distance between the hinge of aft hydrofoils and 
                     % CG [m]
l_zj_f = 0.006;      % z distance between the hinge of fore hydrofoil and 
                     % CG [m]
l_zj_a = 0.006;     % z distance between the hinge of aft hydrofoils and 
                     % CG [m]

alpha_s0_f = -l_s/h_h*theta_s0_f;  % Operating angle of servo motors for the fore hydrofoil
alpha_s0_a = -l_s/h_h*theta_s0_a; % Operating angle of servo motors for the aft hydrofoils






