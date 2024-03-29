clc
clear
close all

%Defining parameters
rho=997;        %[kg/m^3]   Water density
U=4.12;         %[m/s]      Hydrofoil speed in x direction
mass=29.16;     %[kg]       Hydrofoil mass
I_yy=7.6;       %[kg/m^2]
d1=0.4;
d2=0.5;
d3=0.4;
d4=0.5;
d_f=d1;           %0.5;  %[m]
d_a=d2;           %0.3;  %[m]
S_f=0.036;       %[m^2] Area of the fore foil   
S_a=0.036*2;     %[m^2] Area of the aft foil (2X)
d_h=0;

%Initial conditions
z=0.4;          %[m] initial Height
z_dot=0;        %[m/s] initial velocity in z direction    
theta=0;        %[rad] initial pitch   
theta_dot=0;    %[rad] initial pitch rate

%Derivatives with respect to various parameters

Dclf_O_Dalpha=0.06*(360/2*pi);       %From Flyingfish documentation Figure 2-7  Converted to 1/rad
Dcla_O_Dalpha=0.06*(360/2*pi);       %From Flyingfish documentation Figure 2-7
Dcdf_O_Dalpha=0.00375*(360/2*pi);    %From Flyingfish documentation Figure 2-7
Dcda_O_Dalpha=0.00375*(360/2*pi);    %From Flyingfish documentation Figure 2-7

Dalpha_O_Dtheta=1;
Dalpha_O_Dz_dot=1/U;

Dalpha_f_O_Dtheta_dot=-d_f/U;
Dalpha_a_O_Dtheta_dot=-d_a/U;

Dalpha_O_Ddelta_f=1;
Dalpha_O_Ddelta_a=1;

%Defining states and input
%x_dot=[z_dot theta_dot z_dbldot theta_dbldot];
%x=[z theta z_dot theta_dot];
%u=[delta_f delta_a];

%Defining components of A matrix in state space equation x_dot=Ax+Bu
Constant_1=(-rho*U^2)/(2*mass);
A32=Constant_1*(S_f*Dclf_O_Dalpha*Dalpha_O_Dtheta         +S_a*Dcla_O_Dalpha*Dalpha_O_Dtheta             +S_f*Dcdf_O_Dalpha*Dalpha_O_Dtheta           +S_a*Dcda_O_Dalpha*Dalpha_O_Dtheta);
A33=Constant_1*(S_f*Dclf_O_Dalpha*Dalpha_O_Dz_dot         +S_a*Dcla_O_Dalpha*Dalpha_O_Dz_dot             +S_f*Dcdf_O_Dalpha*Dalpha_O_Dz_dot           +S_a*Dcda_O_Dalpha*Dalpha_O_Dz_dot);
A34=Constant_1*(S_f*Dclf_O_Dalpha*Dalpha_f_O_Dtheta_dot   +S_a*Dcla_O_Dalpha*Dalpha_a_O_Dtheta_dot       +S_f*Dcdf_O_Dalpha*Dalpha_f_O_Dtheta_dot     +S_a*Dcda_O_Dalpha*Dalpha_a_O_Dtheta_dot);

Constant_2=(-rho*U^2)/(2*I_yy);
A42=Constant_2*(S_f*Dclf_O_Dalpha*Dalpha_O_Dtheta*d1        +S_a*Dcla_O_Dalpha*Dalpha_O_Dtheta*d2            +S_f*Dcdf_O_Dalpha*Dalpha_O_Dtheta*d3          +S_a*Dcda_O_Dalpha*Dalpha_O_Dtheta*d4);
A43=Constant_2*(S_f*Dclf_O_Dalpha*Dalpha_O_Dz_dot*d1        +S_a*Dcla_O_Dalpha*Dalpha_O_Dz_dot*d2            +S_f*Dcdf_O_Dalpha*Dalpha_O_Dz_dot*d3          +S_a*Dcda_O_Dalpha*Dalpha_O_Dz_dot*d4);
A44=Constant_2*(S_f*Dclf_O_Dalpha*Dalpha_f_O_Dtheta_dot*d1  +S_a*Dcla_O_Dalpha*Dalpha_a_O_Dtheta_dot*d2      +S_f*Dcdf_O_Dalpha*Dalpha_f_O_Dtheta_dot*d3    +S_a*Dcda_O_Dalpha*Dalpha_a_O_Dtheta_dot*d4);

A=[0    0   1   0;
    0   0   0   1;
    0   A32 A33 A34;
    0   A42 A43 A44];

%Defining components of B matrix in state space equation x_dot=Ax+Bu
B31=Constant_1*S_f*(Dclf_O_Dalpha*Dalpha_O_Ddelta_f+Dcdf_O_Dalpha*Dalpha_O_Ddelta_f);
B32=Constant_1*S_a*(Dclf_O_Dalpha*Dalpha_O_Ddelta_a+Dcdf_O_Dalpha*Dalpha_O_Ddelta_a);
B41=-Constant_2*S_f*(Dclf_O_Dalpha*Dalpha_O_Ddelta_f+Dcdf_O_Dalpha*Dalpha_O_Ddelta_f);
B42=Constant_2*S_a*(Dclf_O_Dalpha*Dalpha_O_Ddelta_a+Dcdf_O_Dalpha*Dalpha_O_Ddelta_a);

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
Q(1,1) = 1;
Q(2,2) = 5;
R = [5  0;
    0   1];
K = lqr(A,B,Q,R)

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];
Dc_new =     [0  0  0   0;
              0  0  0   0];

states = {'z' 'theta' 'z_dot' 'theta_dot'};
inputs = {'delta_cr_f';'delta_cr_a'};
% inputs =  {'z2';'theta2';'z_dot2';'theta_dot2'};
outputs = {'z'; 'theta'};

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
Lc=inv(dcgain(sys_cl))
sys_cl = ss(Ac,Bc*Lc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
% sys_cl = ss(Ac,Bc*K,Cc,Dc_new,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;

% inp =[1*ones(size(t));zeros(size(t));zeros(size(t));zeros(size(t))];
inp =[0*ones(size(t));ones(size(t))];
[y,tout,x]=lsim(sys_cl,inp,t);
hold on
[Axis,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(Axis(1),'Ylabel'),'String','Heave (m)')
set(get(Axis(2),'Ylabel'),'String','Pitch (radians)')
title('LQR Controler response to arbitrary input')

%%
%Defining C and D
% C = [1 0 0 0;
%      0 1 0 0];
%  
% D = [0  0;
%      0  0];

C = eye(4);
 
D = zeros(4,2);

sys_ss = ss(A,B,C,D);
dt = 0.05;
t = 0:dt:5;
sys_DT = c2d(sys_ss,dt);
Kdt = dlqr(sys_DT.A,sys_DT.B,Q,R);

% sys_cl_dt = ss(sys_DT.A - sys_DT.B*Kdt, sys_DT.B, sys_DT.C,sys_DT.D,dt);
% Lc_dt = inv(dcgain(sys_cl_dt));

ref = [0*ones(size(t));ones(size(t));zeros(size(t));zeros(size(t))];
% ref = [0*ones(size(t));ones(size(t))];
x = zeros(4,length(t));
u = zeros(2,length(t));

for i=1:length(t)-1
    u(:,i) = Kdt*(ref(:,i) - x(:,i));
%     u(:,i) = Lc_dt*ref(:,i) -Kdt*x(:,i);
    x(:,i+1) = sys_DT.A*x(:,i) + sys_DT.B*u(:,i);
end
%%
figure
hold on
[Axis,H1,H2] = plotyy(t,x(1,:),t,x(2,:),'plot');
set(get(Axis(1),'Ylabel'),'String','Heave (m)')
set(get(Axis(2),'Ylabel'),'String','Pitch (radians)')
title('LQR Controler response to arbitrary input')
