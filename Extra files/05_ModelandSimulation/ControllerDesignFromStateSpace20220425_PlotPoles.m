%Plotting poles and zeros for combination of arm length and installation
%distances of the foils on HEARP
clc
clear
close all

%Defining parameters
rho=997;        %[kg/m^3]   Water density
U=4.12;         %[m/s]      Hydrofoil speed in x direction
mass=29.16;     %[kg]       Hydrofoil mass
I_yy=7.6;       %[kg/m^2]

% d1=0.4;
for d1=0.3:0.05:0.4
    % d2=0.5;
    for d2=0.2:0.05:0.5
        % d3=0.4;
        for d3=0.3:0.05:0.4
            % d4=0.5;
            for d4=0.3:0.05:0.4
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

                %Defining states and input and output and finding the open loop poles

                states = {'z' 'theta' 'z_dot' 'theta_dot'};
                inputs = {'delta_f';'delta_a'};
                outputs = {'z'; 'theta'};

                sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

                pzplot(sys_ss)
                xlim([-180,50])
                ylim([-50,50])
                title(['d1=',num2str(d1),'  d2=',num2str(d2),'  d3=',num2str(d3),'  d4=',num2str(d4),])
                pause(0.1);
            end
        end
    end
end