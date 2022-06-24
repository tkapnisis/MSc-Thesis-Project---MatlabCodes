% Define Parameters

% Mass properties
param.m = 29.38;    % Mass [kg] 
param.g = 9.81;   % Gravity acceleration [m/s^2]
param.I_x = ureal('I_x',3.77,'Percentage',[-10 10]); % Moment of inertia around x- axis [kg*m^2]
param.I_xy = ureal('I_xy',-0.32,'Percentage',[-10 10]);  % Product of inertia around xy-axes [kg*m^2]
param.I_y = ureal('I_y',7.60,'Percentage',[-10 10]);    % Moment of inertia around y- axis [kg*m^2]

% Operating conditions of HEARP
% param.U_0 = 4;  % Operating cruise speed [m/s]
param.U_0 = ureal('U_0',4,'Percentage',[-10 10]);  % Operating cruise speed [m/s]
param.theta_s0_f = -0.2452;  % Operating angle of servo motors for the fore hydrofoil
param.theta_s0_ap = -0.2788; % Operating angle of servo motors for the aft port hydrofoil
param.theta_s0_as = -0.3232; % Operating angle of servo motors for the aft starboard hydrofoil

% % Operating angle of servo motors for the fore hydrofoil
% param.theta_s0_f = ureal('theta_s0_f',-0.2452,'Percentage',[-10 10]);  
%  % Operating angle of servo motors for the aft port hydrofoil
% param.theta_s0_ap = ureal('theta_s0_ap',-0.2788,'Percentage',[-10 10]);
% % Operating angle of servo motors for the aft starboard hydrofoil
% param.theta_s0_as = ureal('theta_s0_as',-0.3232,'Percentage',[-10 10]); 

param.z_n0 = -0.2;     % Operating height of CG with respect to mean water 
                       % surface [m]
param.rho = 999.1;     % Water density at 15 Celcius [kg/m^3]

% Hydrofoils parameters (Naca0012 foil)
param.A_h = 0.0322;         % Projected area of the hydrofoil [m^2]
param.C_L0 = ureal('C_L0',-3.64e-4,'Percentage',[-10,5]);        % y-intercept of the linear approximation for CL [-]
param.C_La = ureal('C_La',3.925,'Percentage',[-10,5]);        % Slope of the linear approximation for CL [-]
param.C_D0 = ureal('C_D0',0.027,'Percentage',[-20,50]);        % y-intercept of the linear approximation for CD [-]
param.C_Da = ureal('C_Da',0.155,'Percentage',[-20,50]);        % Slope of the linear approximation for CL [-]


param.l_s = 0.06;         % Moment arm of the servo motor [m] 
param.h_h = 0.178;         % z distance between the hinge and the joint of
                     % connection rod of the strut of T-foil [m]
param.l_b = 0.484;         % Distance between the centre of pressure of hydrofoil 
                     % and the hinge [m]
param.l_a0 = 0.473;        % z distance between the centre of pressure of 
                     % hydrofoil and the hinge for alpha_s =0 [m]
param.gamma_0 = acos(param.l_a0/param.l_b); % Angle between the joint of connection rod and the
                     % z axis of the centre of pressure of hydrofoil [rad]
% x distance between the hinge of fore hydrofoil and CG [m]
param.l_xj_f = ureal('l_xj_f',0.402,'Percentage',[-10,10]); 
% x distance between the hinge of fore hydrofoil and CG [m]
param.l_xj_ap = ureal('l_xj_ap',-0.298,'Percentage',[-10,10]);   
% x distance between the hinge of aft starboard hydrofoil and CG [m]
param.l_xj_as = ureal('l_xj_as',-0.298,'Percentage',[-10,10]);    

% y distance between the centre of pressure of fore hydrofoil and CG [m]
param.l_y_f = ureal('l_y_f',-0.024,'Percentage',[-10,10]);
% y distance between the centre of pressure of aft port hydrofoil and CG [m]
param.l_y_ap = ureal('l_y_ap',-0.481,'Percentage',[-10,10]);  
% y distance between the centre of pressure of aft starboard hydrofoil and CG [m]            
param.l_y_as = ureal('l_y_as',0.433,'Percentage',[-10,10]);              

% z distance between the hinge of fore hydrofoil and CG [m]
param.l_zj_f = ureal('l_zj_f',0.006,'Percentage',[-10,10]);    
% z distance between the hinge of aft port hydrofoil and CG [m]
param.l_zj_ap = ureal('l_zj_f',0.006,'Percentage',[-10,10]);   
% z distance between the hinge of aft starboard hydrofoil and CG [m]
param.l_zj_as = ureal('l_zj_f',0.006,'Percentage',[-10,10]);     
