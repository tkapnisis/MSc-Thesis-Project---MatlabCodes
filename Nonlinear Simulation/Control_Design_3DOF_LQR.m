clc
clear all
close all

load('Linear_State_Space_3DOF.mat')
load('Parameters_3DOF.mat','param')

% Simulation of the closed loop system with the LQR controller
dt = 0.01; % sampling time
tend = 40; % duration of simulation in seconds
t = 0:dt:tend;
%%
% Descretization of the model

G_dt = c2d(G,dt);


%% LQR control design

Q = diag([100 100 100 1e-1 1e-1 1e-1]);
R = diag([1 1 1]);
K = lqr(A,B,Q,R);

[K_dt,~,~] = dlqr(G_dt.A,G_dt.B,Q,R);    

%%


states = {'z', 'phi', 'theta', 'z_dot', 'phi_dot', 'theta_dot'};
inputs = {'theta_s_f';'theta_s_ap';'theta_s_as'};
outputs = {'z'; 'phi'; 'theta'};
eig(A-B*K)

sys_cl = ss(A-B*K,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

Lc=inv(dcgain(sys_cl));

sys_cl_dt = ss(G_dt.A - G_dt.B*K_dt,G_dt.B,G_dt.C,G_dt.D,'statename',states,'inputname',inputs,'outputname',outputs);
Lc_dt = inv(dcgain(sys_cl_dt));

sys_cl_Lc = ss(A-B*K,B*Lc,C,D);%,'statename',states,'inputname',inputs,'outputname',outputs);

save('LQR_controller','K','Lc','K_dt','Lc_dt')

figure;
pzmap(sys_cl_Lc);
grid on

%%
% ref = [-0.05*ones(size(t));0*ones(size(t));0*ones(size(t))];
ref = [0.1*sin(2*t);0*ones(size(t));0.05*sin(1*t)];
x0 = [0, 0, 0, 0, 0, 0];
[y,t,x]=lsim(sys_cl_Lc,ref,t,x0);

figure
step(sys_cl_Lc)
grid minor

figure
subplot(3,1,1)
plot(t,y(:,1) + param.z_n0,'LineWidth',1.5)
title('Heave')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$z_n$} \textbf{[m]}','interpreter','latex')
grid minor
subplot(3,1,2)
plot(t,rad2deg(y(:,2)),'LineWidth',1.5)
title('Roll')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\phi$} \textbf{[deg]}','interpreter','latex')
grid minor
subplot(3,1,3)
plot(t,rad2deg(y(:,3)),'LineWidth',1.5)
title('Pitch')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\theta$} \textbf{[deg]}','interpreter','latex')
grid minor

inp_val=Lc*ref-K*x';
figure
plot(t,rad2deg(inp_val),'LineWidth', 1.5)
title('Servo motor angles - Control inputs')
grid minor
ylabel('\boldmath{$\theta_s$} \textbf{[deg]}','interpreter','latex')
xlabel('\textbf{time [s]}','interpreter','latex')
legend('Fore hydrofoil', 'Aft port hydrofoil', 'Aft starboard hydrofoil')

figure
step(sys_cl_Lc)
Step_info = stepinfo(sys_cl_Lc);
%%
%% Optical simulation
figure
view(3)
set(gcf, 'WindowState', 'maximized');
tic;

for k=1:length(t)
    Visualization_3DOF(x(k,:),param);
    title(['t= ',num2str(t(k),3),' [s]']);
    xlabel('\boldmath{$x_n$} \textbf{[m]}','interpreter','latex','FontSize',15,'Interpreter','latex')
    ylabel('\boldmath{$y_n$} \textbf{[m]}','interpreter','latex','FontSize',15,'Interpreter','latex')
    zlabel('\boldmath{$z_n$} \textbf{[m]}','interpreter','latex','FontSize',15,'Interpreter','latex')
%     view(0,90) % for view x-y axis
%     view(0,0) % for view x-z axis
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