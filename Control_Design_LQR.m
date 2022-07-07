clc
clear all
close all

load('LTI_Nominal_Plant.mat')
load('Parameters_Nominal.mat','param')
load('LTI_Perturbed_Plant.mat','Gp')

% Simulation time 
dt = 0.05; % sampling time
tend = 20; % duration of simulation in seconds
t = 0:dt:tend;

% Definitons of the state space
states = {'z', 'phi', 'theta', 'z_dot', 'phi_dot', 'theta_dot'};
inputs = {'theta.s.f';'theta.s.ap';'theta.s.as'};
outputs = {'z'; 'phi'; 'theta'};
disturbances = {'u.w.f';'w.w.f';'u.w.ap';'w.w.ap';'u.w.as';'w.w.as'};
ref_signals = {'z.ref'; 'phi.ref'; 'theta.ref'};

G_DT = c2d(G,dt);
Gd_DT = c2d(Gd,dt);
%% LQR control design

Q = diag([100 10 100 1e-2 1e-2 1e-2]); % Weights for the states
R = diag([1 1 1]);                     % Weights for the inputs
K = lqr(G.A,G.B,Q,R);                  % LQR Controller gain for continuous-time
K_DT = dlqr(G_DT.A,G_DT.B,Q,R);         % LQR Controller gain for discrete-time

% Closed-loop system for continuous-time
sys_CL = ss(G.A - G.B*K,G.B,G.C,G.D);
% Closed-loop system for discrete-time
sys_CL_DT = ss(G_DT.A - G_DT.B*K_DT,G_DT.B,G_DT.C,G_DT.D,dt);

% DC-gain of the closed-loop system (Zero frequency gain)
% It affects the steady-state solution but do not affect the stability of
% the system
Lc = inv(dcgain(sys_CL));        % DC-gain for continuous-time closed-loop system
Lc_DT = inv(dcgain(sys_CL_DT));  % DC-gain for discrete-time closed-loop system

% Closed-loop system for continuous-time including the DC-gain
sys_CL = ss(G.A - G.B*K,G.B*Lc,G.C,G.D,'statename',states,'inputname',...
                                       inputs,'outputname',outputs);
% Closed-loop system for disrete-time including the DC-gain
sys_CL_DT = ss(G_DT.A - G_DT.B*K_DT,G_DT.B*Lc_DT,G_DT.C,G_DT.D,dt,...
               'statename',states,'inputname',inputs,'outputname',outputs);

% Closed-loop system for continuous-time including wave disturbances
sys_CL_w = ss(G.A - G.B*K,[G.B*Lc, Gd.B],G.C,[G.D,Gd.D],'statename',states,...
              'inputname',[ref_signals;disturbances],'outputname',outputs);

% Closed-loop system of the perturbed-plant in continuous-time
sys_CL_p = ss(Gp.A - Gp.B*K,Gp.B*Lc,Gp.C,Gp.D,'statename',states,'inputname',...
                                       inputs,'outputname',outputs);
%% Simulations in calm water

% Step responses of the closed-loop system for continuous and discrete-time
figure
step(sys_CL)
hold on
step(sys_CL_DT)
grid minor
legend('Continuous','Discrete')

set(gcf, 'WindowState', 'maximized');
saveas(gcf,[pwd '/Figures/LQR/Step Response CT-DT.png'])

%%
% Step responses of the nominal and the perturbed closed-loop system
figure
step(sys_CL_p)
hold on
step(sys_CL)
grid minor
legend('Perturbed','Nominal')

set(gcf, 'WindowState', 'maximized');
saveas(gcf,[pwd '/Figures/LQR/Step Response Nominal-Perurbed.png'])
%%
% Reference signal
ref = [-0.05*square(t);0*ones(size(t));0*ones(size(t))];
% ref = [0.1*sin(2*t);0*ones(size(t));0.05*sin(1*t)];
% Initial sate of the system
x0 = [0, 0, 0, 0, 0, 0];
[y,~,x]=lsim(sys_CL,ref,t,x0);

figure
subplot(3,1,1)
plot(t,y(:,1) + param.z_n0,'LineWidth',1.5)
hold on
plot(t,ref(1,:) + param.z_n0,'r--','LineWidth',1.5)
title('Heave')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$z_n$} \textbf{[m]}','interpreter','latex')
legend('Response','Reference Signal','Location','best')
grid minor

subplot(3,1,2)
plot(t,rad2deg(y(:,2)),'LineWidth',1.5)
hold on
plot(t,ref(2,:),'r--','LineWidth',1.5)
title('Roll')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\phi$} \textbf{[deg]}','interpreter','latex')
legend('Response','Reference Signal','Location','best')
grid minor

subplot(3,1,3)
plot(t,rad2deg(y(:,3)),'LineWidth',1.5)
hold on
plot(t,ref(3,:),'r--','LineWidth',1.5)
title('Pitch')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\theta$} \textbf{[deg]}','interpreter','latex')
legend('Response','Reference Signal','Location','best')
grid minor

eq_input = [param.theta_s_f0;param.theta_s_ap0;param.theta_s_as0];
u = Lc*ref - K*x' + eq_input;
figure
plot(t,rad2deg(u),'LineWidth', 1.5)
title('Servo motor angles - Control inputs')
grid minor
ylabel('\boldmath{$\theta_s$} \textbf{[deg]}','interpreter','latex')
xlabel('\textbf{time [s]}','interpreter','latex')
legend('Fore hydrofoil', 'Aft port hydrofoil', 'Aft starboard hydrofoil')

Step_info = stepinfo(sys_CL);

%% %% Simulations in long-crested regular waves

% Calculation of waves velocity profile for each hydrofoil
% Parameters of long-crested regular wave
wave_param.omega_0 = 1.5;   % Wave frequency [rad/s]
wave_param.lambda = 2;    % Wave length [m]
wave_param.zeta_0 = 0.10;  % Wave amplitude [m]
wave_param.beta = pi;     % Encounter angle (beta=0 for following waves) [rad] 

dw = Wave_Model(t,wave_param,foil_loc,param);

ref = [-0.05*square(t)*0;0*ones(size(t));0*ones(size(t))];
% ref = [0.1*sin(2*t);0*ones(size(t));0.05*sin(1*t)];

x0 = [0, 0, 0, 0, 0, 0];
[y_w,~,x_w] = lsim(sys_CL_w,[ref;dw],t,x0);

figure
subplot(3,1,1)
plot(t,y_w(:,1) + param.z_n0,'LineWidth',1.5)
title('Heave')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$z_n$} \textbf{[m]}','interpreter','latex')
grid minor
subplot(3,1,2)
plot(t,rad2deg(y_w(:,2)),'LineWidth',1.5)
title('Roll')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\phi$} \textbf{[deg]}','interpreter','latex')
grid minor
subplot(3,1,3)
plot(t,rad2deg(y_w(:,3)),'LineWidth',1.5)
title('Pitch')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\theta$} \textbf{[deg]}','interpreter','latex')
grid minor

u_waves = Lc*ref - K*x_w' + eq_input;

figure
plot(t,rad2deg(u_waves),'LineWidth', 1.5)
title('Servo motor angles - Control inputs')
grid minor
ylabel('\boldmath{$\theta_s$} \textbf{[deg]}','interpreter','latex')
xlabel('\textbf{time [s]}','interpreter','latex')
legend('Fore hydrofoil', 'Aft port hydrofoil', 'Aft starboard hydrofoil')

%%
%% Optical simulation
%{
figure
view(3)
set(gcf, 'WindowState', 'maximized');
tic;

for k=1:length(t)
    Visualization(x_w(k,:));
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