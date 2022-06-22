clc
clear
close all

%Defining parameters
rho=997;      %[kg/m^3]   Water density
area=0.024;    %[m^2]      Area of the foil
U=5;        %[m/s]      Hydrofoil speed in x direction
mass=38;       %[kg]       Hydrofoil mass
d_a=0.5;       %[m]
d_f=0.5;       %[m]
I_yy=6.8;        %[kg/m^2]

%Initial conditions
z=0.4;          %[m] initial Height
z_dot=0;        %[m/s] initial velocity in z direction    
theta=0;        % initial pitch   
theta_dot=0;    % initial pitch rate

%Derivatives with respect to various parameters

Dclf_O_Dalpha=0.06;       %From Flyingfish documentation Figure 2-7
Dcla_O_Dalpha=0.06;       %From Flyingfish documentation Figure 2-7
Dcdf_O_Dalpha=0.00375;    %From Flyingfish documentation Figure 2-7
Dcda_O_Dalpha=0.00375;    %From Flyingfish documentation Figure 2-7

Dalpha_O_Dtheta=1;
Dalpha_O_Dz_dot=1/U;

Dalpha_f_O_Dtheta_dot=d_f/U;
Dalpha_a_O_Dtheta_dot=d_a/U;

Dalpha_O_Ddelta_f=1;
Dalpha_O_Ddelta_a=1;

%Defining states and input
%x_dot=[z_dot theta_dot z_dbldot theta_dbldot];
%x=[z theta z_dot theta_dot];
%u=[delta_f delta_a];

%Defining components of A matrix in state space equation x_dot=Ax+Bu
Constant_1=(-rho*area*U^2)/(2*mass);
A32=Constant_1*(Dclf_O_Dalpha*Dalpha_O_Dtheta+Dcla_O_Dalpha*Dalpha_O_Dtheta+Dcdf_O_Dalpha*Dalpha_O_Dtheta+Dcda_O_Dalpha*Dalpha_O_Dtheta);
A33=Constant_1*(Dclf_O_Dalpha*Dalpha_O_Dz_dot+Dcla_O_Dalpha*Dalpha_O_Dz_dot+Dcdf_O_Dalpha*Dalpha_O_Dz_dot+Dcda_O_Dalpha*Dalpha_O_Dz_dot);
A34=Constant_1*(Dclf_O_Dalpha*Dalpha_f_O_Dtheta_dot+Dcla_O_Dalpha*Dalpha_a_O_Dtheta_dot+Dcdf_O_Dalpha*Dalpha_f_O_Dtheta_dot+Dcda_O_Dalpha*Dalpha_a_O_Dtheta_dot);

Constant_2=(-rho*area*U^2)/(2*I_yy);
A42=Constant_2*(Dclf_O_Dalpha*Dalpha_O_Dtheta+Dcla_O_Dalpha*Dalpha_O_Dtheta+Dcdf_O_Dalpha*Dalpha_O_Dtheta+Dcda_O_Dalpha*Dalpha_O_Dtheta);
A43=Constant_2*(Dclf_O_Dalpha*Dalpha_O_Dz_dot+Dcla_O_Dalpha*Dalpha_O_Dz_dot+Dcdf_O_Dalpha*Dalpha_O_Dz_dot+Dcda_O_Dalpha*Dalpha_O_Dz_dot);
A44=Constant_2*(Dclf_O_Dalpha*Dalpha_f_O_Dtheta_dot+Dcla_O_Dalpha*Dalpha_a_O_Dtheta_dot+Dcdf_O_Dalpha*Dalpha_f_O_Dtheta_dot+Dcda_O_Dalpha*Dalpha_a_O_Dtheta_dot);

A=[0    0   1   0;
    0   0   0   1;
    0   A32 A33 A34;
    0   A42 A43 A44];

%Defining components of B matrix in state space equation x_dot=Ax+Bu
B31=Constant_1*(Dclf_O_Dalpha*Dalpha_O_Ddelta_f+Dcdf_O_Dalpha*Dalpha_O_Ddelta_f);
B32=Constant_1*(Dclf_O_Dalpha*Dalpha_O_Ddelta_a+Dcdf_O_Dalpha*Dalpha_O_Ddelta_a);
B41=-Constant_2*(Dclf_O_Dalpha*Dalpha_O_Ddelta_f+Dcdf_O_Dalpha*Dalpha_O_Ddelta_f);
B42=Constant_2*(Dclf_O_Dalpha*Dalpha_O_Ddelta_a+Dcdf_O_Dalpha*Dalpha_O_Ddelta_a);

B=[0    0;
   0    0;
   B31  B32;
   B41	B42];

%Defining C and D
C = [1 0 0 0;
     0 1 0 0];
 
D = [0  0;
     0  0];

%Print matrixes A B C D
A,B,C,D

%Defining states and input and output and finding the open loop poles

states = {'z' 'theta' 'z_dot' 'theta_dot'};
inputs = {'delta_f';'delta_a'};
outputs = {'z'; 'theta'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

poles = eig(A)

%generate the controllability matrix
co = ctrb(sys_ss);
controllability = rank(co)

%LQR Controller
Q = C'*C;
Q(1,1) = 100;
Q(2,2) = 500;
R = [500  0;
    0   100];
K = lqr(A,B,Q,R)

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
% Dc = [D];
Dc_new =     [0  0  0   0;
              0  0  0   0];

states = {'z' 'theta' 'z_dot' 'theta_dot'};
% inputs = {'delta_cr_f';'delta_cr_a'};
inputs =  {'z2';'theta2';'z_dot2';'theta_dot2'};
outputs = {'z'; 'theta'};

% sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
% Lc=inv(dcgain(sys_cl))
% sys_cl = ss(Ac,Bc*Lc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
sys_cl = ss(Ac,Bc*K,Cc,Dc_new,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:20;

inp =[1*ones(size(t));zeros(size(t));zeros(size(t));zeros(size(t))];
[y,t,x]=lsim(sys_cl,inp,t);
hold on
[Axis,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(Axis(1),'Ylabel'),'String','Heave (m)')
set(get(Axis(2),'Ylabel'),'String','Pitch (radians)')
title('LQR Controler response to arbitrary input')
