% Defining the nominal values for all the parameters of the HEARP
clc
clear all
close all

% Enviromnent conditions
g = 9.81;      % Gravity acceleration [m/s2]
rho = 999.1;   % Water density at 15 Celcius [kg/m^3]

% Mass properties
m = 29.38;     % Mass kg 
I_x = 3.77;    % Moment of inertia around x- axis [kg*m^2]
I_xy = -0.32;  % Product of inertia around xy-axes [kg*m^2]
I_y = 7.60;    % Moment of inertia around y- axis [kg*m^2]

% Operating conditions of HEARP
U_0 = 4;              % Operating cruise speed [m/s]
z_n0 = -0.25; % Operating point of vertical position of CG with respect to 
             % mean water surface [m]

% Hydrofoils parameters
A_h = 0.0322;        % Projected area of the hydrofoil [m^2]
C_L0 = -3.64e-4;     % y-intercept of the linear approximation for CL [-]                
C_La = 3.925;          % Slope of the linear approximation for CL [-]
C_D0 = 0.027;        % y-intercept of the linear approximation for CD [-]
C_Da = 0.155;        % Slope of the linear approximation for CL [-]

l_s = 0.06;         % Moment arm of the servo motor [m] 
h_h = 0.178;         % z distance between the hinge and the joint of
                     % connection rod of the strut of T-foil [m]
l_b = 0.484;         % Distance between the centre of pressure of hydrofoil 
                     % and the hinge [m]
l_a0 = 0.473;        % z distance between the centre of pressure of 
                     % hydrofoil and the hinge for alpha_s =0 [m]
gamma_0 = acos(l_a0/l_b); % Angle between the joint of connection rod and the
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
l_y_ap = -0.24;      % y distance between the centre of pressure of aft 
                     % port hydrofoil and CG [m]
l_y_as = 0.19;       % y distance between the centre of pressure of aft 
                     % starport hydrofoil and CG [m]
l_zj_f = 0.006;      % z distance between the hinge of fore hydrofoil and 
                     % CG [m]
l_zj_ap = 0.006;     % z distance between the hinge of aft port hydrofoil  
                     % and CG [m]
l_zj_as = 0.006;     % z distance between the hinge of aft starport
                     % hydrofoil and CG [m]

% Time constant of servo motors first-order model                     
tau_d_f = 0.05;      % Fore foil [s]
tau_d_ap = 0.05;     % Aft port foil [s]
tau_d_as = 0.05;     % Aft starboard foil [s]

%%%%%%%%%%%%%%%% Define all the parameters with a structure %%%%%%%%%%%%%%%                   

% Enviromnent conditions
param.g = g;    % Gravity acceleration [m/s^2]
param.rho = rho;              % Water density at 25 Celcius [kg/m^3]

% Mass properties
param.m = m;    % Mass kg 
param.I_x = I_x; % Moment of inertia around x- axis [kg*m^2]
param.I_xy = I_xy;  % Product of inertia around xy-axes [kg*m^2]
param.I_y = I_y;    % Moment of inertia around y- axis [kg*m^2]

% Hydrofoils parameters
param.A_h = A_h;         % Projected area of the hydrofoil [m^2]
param.C_L0 = C_L0;        % y-intercept of the linear approximation for CL [-]
param.C_La = C_La;        % Slope of the linear approximation for CL [-]
param.C_D0 = C_D0;        % y-intercept of the linear approximation for CD [-]
param.C_Da = C_Da;        % Slope of the linear approximation for CL [-]

% Operating conditions of HEARP
param.U_0 = U_0;              % Operating cruise speed [m/s]
param.z_n0 = z_n0; % Operating height of CG with respect to mean water surface [m]

param.l_s = l_s; % Moment arm of the servo motor [m] 
param.h_h = h_h; % z distance between the hinge and the joint of connection
                 % rod of the strut of T-foil [m]
param.l_b = l_b;     % Distance between the centre of pressure of hydrofoil 
                     % and the hinge [m]              
param.gamma_0 = gamma_0; % Angle between the joint of connection rod and the z axis
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

% Time constant of servo motors first-order model                     
param.tau_d_f = tau_d_f;    %Fore foil 
param.tau_d_ap = tau_d_ap;  % Aft port foil
param.tau_d_as = tau_d_as;  % Aft starboard foil
  
save('Data Files/Parameters_Nominal.mat')                         