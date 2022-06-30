% Theodoulos Kapnisis
% Student ID: 5271355
% Thesis Project: Modelling and control of experimental scale hydrofoil craft

%% Define the perturbed plant for HEARP
clc
clear all
close all

load('LTI_Symbolic_Matrices.mat')
run Define_Parameters_Uncertainties.m

[A_u,B_u,Bd_u,C_u,D_u,Dd_u] = Perturbed_Plant_Matrices(param,A_s,B_s,Bd_s,C_s,D_s,Dd_s);

states = {'z', 'phi', 'theta', 'z_dot', 'phi_dot', 'theta_dot'};
inputs = {'theta_sf';'theta_sap';'theta_sas'};
outputs = {'z'; 'phi'; 'theta'};
disturbances = {'u_w_f';'w_w_f';'u_w_ap';'w_w_ap';'u_w_as';'w_w_as'};

%% Estimation of uncertain matrices by considering uncertainty on the elements of the matrices
%
% Random sampling of the uncertain matrices in order to calculate the
% min/max values of the elements of each matrix.

% Then we redefine the uncertain matrices by just using the nominal values
% and min-max values of each element

% Number of samples that are used
no_samples = 50;

A_u_samples = usample(A_u,no_samples);
A_u_max = zeros(size(A_u,1),size(A_u,2));
A_u_min = zeros(size(A_u,1),size(A_u,2));
for i=1:size(A_u,1)
    for j=1:size(A_u,2)
        A_u_max(i,j) = max(A_u_samples(i,j,:));
        A_u_min(i,j) = min(A_u_samples(i,j,:));
    end
end

A_u_elem = umat([]);
for i=1:size(A_u,1)
    for j=1:size(A_u,2)
        if A_u_max(i,j)~=A_u_min(i,j)
            temp_var = strcat('A',num2str(i),num2str(j));
            A_u_elem(i,j) = ureal(temp_var,A_u.NominalValue(i,j),'Range',...
                                 [A_u_min(i,j), A_u_max(i,j)]);
        else
            A_u_elem(i,j) = A_u(i,j);
        end    
    end
end

B_u_samples = usample(B_u,no_samples);
B_u_max = zeros(size(B_u,1),size(B_u,2));
B_u_min = zeros(size(B_u,1),size(B_u,2));
for i=1:size(B_u,1)
    for j=1:size(B_u,2)
        B_u_max(i,j) = max(B_u_samples(i,j,:));
        B_u_min(i,j) = min(B_u_samples(i,j,:));
    end
end

B_u_elem = umat([]);
for i=1:size(B_u,1)
    for j=1:size(B_u,2)
        if B_u_max(i,j)~=B_u_min(i,j)
            temp_var = strcat('B',num2str(i),num2str(j));
            B_u_elem(i,j) = ureal(temp_var,B_u.NominalValue(i,j),'Range',...
                                 [B_u_min(i,j), B_u_max(i,j)]);
        else
            B_u_elem(i,j) = B_u(i,j);
        end    
    end
end

Bd_u_samples = usample(Bd_u,no_samples);
Bd_u_max = zeros(size(Bd_u,1),size(Bd_u,2));
Bd_u_min = zeros(size(Bd_u,1),size(Bd_u,2));
for i=1:size(Bd_u,1)
    for j=1:size(Bd_u,2)
        Bd_u_max(i,j) = max(Bd_u_samples(i,j,:));
        Bd_u_min(i,j) = min(Bd_u_samples(i,j,:));
    end
end

Bd_u_elem = umat([]);
for i=1:size(Bd_u,1)
    for j=1:size(Bd_u,2)
        if Bd_u_max(i,j)~=Bd_u_min(i,j)
            temp_var = strcat('Bd',num2str(i),num2str(j));
            Bd_u_elem(i,j) = ureal(temp_var,Bd_u.NominalValue(i,j),'Range',...
                                 [Bd_u_min(i,j), Bd_u_max(i,j)]);
        else
            Bd_u_elem(i,j) = Bd_u(i,j);
        end    
    end
end

%% Define the tranfer matrices for nominal and perturbed plant
% Nominal plant
G = ss(A_u.NominalValue,B_u.NominalValue,C_u,D_u,...
       'statename',states,'inputname',inputs,'outputname',outputs);
% Disturbances transfer matrix
Gd = ss(A_u.NominalValue,Bd_u.NominalValue,C_u,Dd_u,...
        'statename',states,'inputname',disturbances,'outputname',outputs);
% Perturbed plant with uncertain parameters
Gp = ss(A_u,B_u,C_u,D_u,...
        'statename',states,'inputname',inputs,'outputname',outputs);
Gp = simplify(Gp,'full');
% Perturbed disturbances transfer matrix with uncertain parameters
Gd_p = ss(A_u,Bd_u,C_u,Dd_u,...
        'statename',states,'inputname',disturbances,'outputname',outputs);
Gd_p = simplify(Gd_p,'full');
% Simplified perturbed plant with uncertain elements
Gp_s = ss(A_u_elem,B_u_elem,C_u,D_u,...
        'statename',states,'inputname',inputs,'outputname',outputs);
Gp_s = simplify(Gp_s,'full');
% Simplified perturbed disturbance transfer matrix with uncertain elements
Gd_p_s = ss(A_u_elem,Bd_u_elem,C_u,Dd_u,...
        'statename',states,'inputname',disturbances,'outputname',outputs);
Gd_p_s = simplify(Gd_p_s,'full');

save('LTI_Perturbed_Plant.mat')