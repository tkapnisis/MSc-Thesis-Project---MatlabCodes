clc
clear all
close all
%%
load('Linear_State_Space_3DOF.mat')
load('Parameters_3DOF.mat','param')
% Simulation time 
dt = 0.05; % sampling time
tend = 20; % duration of simulation in seconds
t = 0:dt:tend;

% Definitons of the state space
states = {'z_n', 'phi', 'theta', 'z_dot', 'phi_dot', 'theta_dot'};
inputs = {'theta_s_f';'theta_s_ap';'theta_s_as'};
outputs = {'z_n'; 'phi'; 'theta'};
disturbances = {'u_w_f';'w_w_f';'u_w_ap';'w_w_ap';'u_w_as';'w_w_as'};

G_DT = c2d(G,dt);
Gd_DT = c2d(Gd,dt);
%% LQR control design

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Continuous-time %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% LQR design weights
Q = diag([500 100 500 1e-1 1e-1 1e-1]);
R = diag([1 1 1]);
% Q = diag([100 100 100 1e-1 1e-1 1e-1]);
% R = diag([1 1 1]);
% LQR controller gain
K_lqr = lqr(G.A,G.B,Q,R);
% Closed-loop poles of the continuous-time system
ps = eig(G.A - G.B*K_lqr);

reference_signals = {'z_n_ref'; 'phi_ref'; 'theta_ref'};
% Closed-loop state stace with the LQR controller and without disturbances
sys_CL = ss(G.A - G.B*K_lqr,G.B,G.C,G.D);
Lc = inv(dcgain(sys_CL));
sys_CL = ss(G.A - G.B*K_lqr,G.B*Lc,G.C,G.D,'statename',states,'inputname',...
            reference_signals,'outputname',outputs);

% Closed-loop state stace with the LQR controller and with wave disturbances
sys_CL_w = ss(G.A - G.B*K_lqr,[G.B*Lc, Gd.B],G.C,[G.D,Gd.D],'statename',states,...
              'inputname',[reference_signals;disturbances],'outputname',outputs);
figure
step(sys_CL)
hold on
grid minor

Step_info = stepinfo(sys_CL);
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Discrete-time %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
% LQR controller gain
K_DT = dlqr(G_DT.A,G_DT.B,Q,R);
% Closed-loop poles of the discrete-time system
ps_DT = eig(G_DT.A - G_DT.B*K_DT);

% Closed-loop state stace with the LQR controller and without disturbances
sys_CL_DT = ss(G_DT.A - G_DT.B*K_DT,G_DT.B,G_DT.C,G_DT.D,dt);
Lc_DT = inv(dcgain(sys_CL_DT));
sys_CL_DT = ss(G_DT.A - G_DT.B*K_DT,G_DT.B*Lc_DT,G_DT.C,G_DT.D,dt,'statename',...
               states,'inputname',reference_signals,'outputname',outputs);

% Closed-loop state stace with the LQR controller and with wave disturbances
sys_CL_DT_w = ss(G_DT.A - G_DT.B*K_DT,[G_DT.B*Lc_DT, Gd_DT.B],G_DT.C,...
                 [G_DT.D,Gd_DT.D],dt,'statename',states,'inputname',...
                 [inputs;disturbances],'outputname',outputs);
%}
%% Calculation of waves velocity profile for each hydrofoil

% Parameters of long-crested regular wave
wave_param.omega_0 = 1.5;   % Wave frequency [rad/s]
wave_param.lambda = 1.5;    % Wave length [m]
wave_param.zeta_0 = 0.15;  % Wave amplitude [m]
wave_param.beta = 0*pi;     % Encounter angle (beta=0 for following waves) [rad] 

dw = Wave_Model(t,wave_param,foil_loc,param);


%% Continous-time simulations
ref = [-0.05*square(t)*0;0*ones(size(t));0*ones(size(t))];
% ref = [0.1*sin(2*t);0*ones(size(t));0.05*sin(1*t)];

x0 = [0, 0, 0, 0, 0, 0];
[y_CT,~,x_CT] = lsim(sys_CL,ref,t,x0);
[y_CT_w,~,x_CT_w] = lsim(sys_CL_w,[ref;dw],t,x0);

figure
subplot(3,1,1)
plot(t,y_CT_w(:,1) + param.z_n0,'LineWidth',1.5)
title('Heave')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$z_n$} \textbf{[m]}','interpreter','latex')
grid minor
subplot(3,1,2)
plot(t,rad2deg(y_CT_w(:,2)),'LineWidth',1.5)
title('Roll')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\phi$} \textbf{[deg]}','interpreter','latex')
grid minor
subplot(3,1,3)
plot(t,rad2deg(y_CT_w(:,3)),'LineWidth',1.5)
title('Pitch')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\theta$} \textbf{[deg]}','interpreter','latex')
grid minor

u_CT = Lc*ref - K_lqr*x_CT_w';
figure
plot(t,rad2deg(u_CT),'LineWidth', 1.5)
title('Servo motor angles - Control inputs')
grid minor
ylabel('\boldmath{$\theta_s$} \textbf{[deg]}','interpreter','latex')
xlabel('\textbf{time [s]}','interpreter','latex')
legend('Fore hydrofoil', 'Aft port hydrofoil', 'Aft starboard hydrofoil')

%% %% Discrete-time simulations
%{
x_DT = zeros(size(G_DT.A,1),length(t));
x_DT(:,1) = x0; 
u_DT = zeros(size(G_DT.B,2),length(t));
d_DT = zeros(size(Gd_DT.B,2),length(t));
foil_var_loc = zeros(6,length(t));

for k=1:length(t)-1
    u_DT(:,k) = Lc_DT*ref(:,k) - K_DT*x_DT(:,k);
    u_DT(:,k) = min(deg2rad(30), max(deg2rad(-30), u_DT(:,k)));
    [d_DT(:,k), foil_var_loc(:,k)] = Wave_disturbances(t(k),x_DT(:,k),...
                                               u_DT(:,k),param,wave_param);
%     d_DT(:,k) = Wave_disturbances(t(k),x_DT(:,k),u_DT(:,k),param,wave_param);
    x_DT(:,k+1) = G_DT.A*x_DT(:,k) + [G_DT.B, Gd_DT.B]*[u_DT(:,k);d_DT(:,k)];
end

figure
subplot(3,1,1)
plot(t,x_DT(1,:) + param.z_n0,'LineWidth',1.5)
title('Heave')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$z_n$} \textbf{[m]}','interpreter','latex')
grid minor
subplot(3,1,2)
plot(t,rad2deg(x_DT(2,:)),'LineWidth',1.5)
title('Roll')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\phi$} \textbf{[deg]}','interpreter','latex')
grid minor
subplot(3,1,3)
plot(t,rad2deg(x_DT(3,:)),'LineWidth',1.5)
title('Pitch')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\theta$} \textbf{[deg]}','interpreter','latex')
grid minor

figure
plot(t,rad2deg(u_DT),'LineWidth', 1.5)
title('Servo motor angles - Control inputs')
grid minor
ylabel('\boldmath{$\theta_s$} \textbf{[deg]}','interpreter','latex')
xlabel('\textbf{time [s]}','interpreter','latex')
legend('Fore hydrofoil', 'Aft port hydrofoil', 'Aft starboard hydrofoil')
%}
%% Optical simulation
%
figure
view(3)
set(gcf, 'WindowState', 'maximized');
tic;

for k=1:length(t)
    Visualization_3DOF(x_CT_w(k,:));
%     Visualization_3DOF(x_DT(:,k));
    title(['t= ',num2str(t(k),3),' [s]']);
    xlabel('\boldmath{$x_n$} \textbf{[m]}','interpreter','latex','FontSize',15,'Interpreter','latex')
    ylabel('\boldmath{$y_n$} \textbf{[m]}','interpreter','latex','FontSize',15,'Interpreter','latex')
    zlabel('\boldmath{$z_n$} \textbf{[m]}','interpreter','latex','FontSize',15,'Interpreter','latex')
%     view(0,90) % for view x-y axis
    view(0,0) % for view x-z axis
%     view(90,0) % for view y-z axis
%     drawnow

    real_dt = toc;
    while real_dt<=dt
        pause(1e-3)
        real_dt = toc;
    end
    drawnow
    tic;
%     pause(2e-2)
end

%}