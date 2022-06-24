function [A_u,B_u,Bd_u,C_u,D_u,Dd_u] = Uncertain_Plant(param,A_s,B_s,Bd_s,C_s,D_s,Dd_s)

m = param.m;    % Mass kg 
g = param.g;    % Gravity acceleration [m/s^2]
I_x = param.I_x; % Moment of inertia around x- axis [kg*m^2]
I_xy = param.I_xy;  % Product of inertia around xy-axes [kg*m^2]
I_y = param.I_y;    % Moment of inertia around y- axis [kg*m^2]

% Operating conditions of HEARP
U_0 = param.U_0;              % Operating cruise speed [m/s]
theta_s0_f = param.theta_s0_f;  % Operating angle of servo motors for the fore hydrofoil
theta_s0_ap = param.theta_s0_ap; % Operating angle of servo motors for the aft port hydrofoil
theta_s0_as = param.theta_s0_as; % Operating angle of servo motors for the aft starboard hydrofoil

z_n0 = param.z_n0; % Operating height of CG with respect to mean water surface [m]
rho = param.rho;              % Water density at 25 Celcius [kg/m^3]

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
gamma_0 = param.gamma_0; % Angle between the joint of connection rod and the z axis
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

% Define the linearized matrices of the system
A_s = arrayfun(@char, A_s, 'UniformOutput', 0);
A_s = string(A_s);
A_u = umat([]);

for i=1:size(A_s,1)
    for j=1:size(A_s,2)
        A_u(i,j) = eval(A_s(i,j));
    end
end

B_s = arrayfun(@char, B_s, 'UniformOutput', 0);
B_s = string(B_s);
B_u = umat([]);

for i=1:size(B_s,1)
    for j=1:size(B_s,2)
        B_u(i,j) = eval(B_s(i,j));
    end
end

Bd_s = arrayfun(@char, Bd_s, 'UniformOutput', 0);
Bd_s = string(Bd_s);
Bd_u = umat([]);

for i=1:size(Bd_s,1)
    for j=1:size(Bd_s,2)
        Bd_u(i,j) = eval(Bd_s(i,j));
    end
end

C_u = C_s;
D_u = D_s;
Dd_u =  Dd_s;

end