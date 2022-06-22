%% Initialize

clear all;clc;close all

h = 0.01;  % Sampling time in seconds
addpath('Identification Data')

%% Linear Grey Box Identification

% Load input data and initial model 
load('Initial_Linear_Model.mat')
load('random_id.mat','alpha','theta','u')

% Define data to be identified 
y_ident = [theta(:,2), alpha(:,2)]; % output data / measurements
Vs_ident = u(:,2); % input data
data_ident = iddata(y_ident, Vs_ident, h, 'Name', 'Qube_Rotary_Pendulum');
%%
%{
% Parameters to be estimated
a21 = init_sys.A(2,1);
a22 = init_sys.A(2,2);
a23 = init_sys.A(2,3);
a24 = init_sys.A(2,4);
a41 = init_sys.A(4,1);
a42 = init_sys.A(4,2);
a43 = init_sys.A(4,3);
a44 = init_sys.A(4,4);
b2 = init_sys.B(2);
b4 = init_sys.B(4);

% Create initial model
odefun = 'Ident_Model';
parameters = {'a21',a21;'a22',a22;'a23',a23;'a24',a24;'a41',a41;'a42',a42;...
              'a43',a43;'a44',a44;'b2',b2;'b4',b4};
fcn_type = 'cd';
Ts = 0;
init_ident_sys = idgrey(odefun,parameters,fcn_type,{},Ts);

% Options for Estimator
opt = greyestOptions;
opt.InitialState = 'zero';
opt.Focus = 'simulation';
opt.EnforceStability = true;
opt.SearchOptions.MaxIterations = 100;
opt.Display = 'on';

% Estimate model parameters
ident_sys = greyest(data_ident,init_ident_sys,opt);
%}
%%
% Parameters to be estimated
a21 = init_sys.A(2,1);
a22 = init_sys.A(2,2);
a23 = init_sys.A(2,3);
a24 = init_sys.A(2,4);
a41 = init_sys.A(4,1);
a42 = init_sys.A(4,2);
a43 = init_sys.A(4,3);
a44 = init_sys.A(4,4);
b2 = init_sys.B(2);
b4 = init_sys.B(4);

% Create initial model
odefun = 'Ident_Model';
parameters = {'a21',a21;'a22',a22;'a23',a23;'a24',a24;'a41',a41;'a42',a42;...
              'a43',a43;'a44',a44;'b2',b2;'b4',b4};
fcn_type = 'cd';
Ts = 0.01;
init_ident_sys = idgrey(odefun,parameters,fcn_type,{},Ts);

% Options for Estimator
opt = greyestOptions;
opt.InitialState = 'zero';
opt.Focus = 'simulation';
opt.EnforceStability = true;
opt.SearchOptions.MaxIterations = 100;
opt.Display = 'on';

% Estimate model parameters
ident_sys = greyest(data_ident,init_ident_sys,opt);
%%
% Plot Input Signal
figure;
subplot(3,1,1)
plot(u(:,1),u(:,2),'Linewidth',1);
grid on;box on;
title('\textbf{Input signal: Random}','interpreter','latex')
ylabel('\boldmath{$V_s$ $[V]$}','interpreter','latex')
ylim([min(u(:,2))-0.2, max(u(:,2))+0.2])

% Plot Results
data = compare(data_ident,ident_sys);
subplot(3,1,2); % theta angle
hold on;grid on;box on; 
time = alpha(:,1);
y_hat = data.y(:,1); 
y = data_ident.y(:,1); 
plot(time,y*180/pi,'Linewidth',1)
plot(time,y_hat*180/pi,'Linewidth',1)
vaf = max(0,(1-norm(y-y_hat)^2./norm(y)^2)*100); 
ylabel('\boldmath{$\theta$ $[deg]$}','interpreter','latex')
legend('\textbf{Measured}','\textbf{Identified}','interpreter','latex')
title(['\textbf{Theta VAF:} \boldmath{$',num2str(round(vaf,2)),'\%$}'],'interpreter','latex')

subplot(3,1,3); % alpha angle
hold on;grid on;box on; 
y_hat = data.y(:,2); 
y = data_ident.y(:,2);
plot(time,y*180/pi,'Linewidth',1)
plot(time,y_hat*180/pi,'Linewidth',1)
vaf = max(0,(1-norm(y-y_hat)^2./norm(y)^2)*100); 
ylabel('\boldmath{$\alpha$ $[deg]$}','interpreter','latex')
legend('\textbf{Measured}','\textbf{Identified}','interpreter','latex')
title(['\textbf{Alpha VAF:} \boldmath{$',num2str(round(vaf,2)),'\%$}'],'interpreter','latex')
xlabel('\textbf{Time [s]}','interpreter','latex')


%% Validation

for i = 1:3
    
% Load Test Data
    switch i
        case 1
            load('random_val.mat','alpha','theta','u')
            signal = 'Random';
        case 2
            load('sawtooth_val.mat','alpha','theta','u')
            signal = 'Sawtooth';
        case 3
            load('square_val.mat','alpha','theta','u')
            signal = 'Square';
    end

% Validation data
y_val = [theta(:,2), alpha(:,2)];
Vs_val = u(:,2);
data_val = iddata(y_val, Vs_val, h, 'Name', 'Qube_Rotary_Pendulum');

% Plot Results
data = compare(data_val,ident_sys);
figure;
subplot(3,1,1); % input signal
plot(u(:,1),u(:,2),'Linewidth',1);
grid on;box on;
title(['\textbf{Input signal: ',signal,'}'],'interpreter','latex')
ylabel('\boldmath{$V_s$ $[V]$}','interpreter','latex')
ylim([min(u(:,2))-0.2, max(u(:,2))+0.2])

subplot(3,1,2); % theta angle
hold on;grid on;box on; 
time = alpha(:,1);
y_hat = data.y(:,1); 
y = data_val.y(:,1);
plot(time,y*180/pi,'Linewidth',1)
plot(time,y_hat*180/pi,'Linewidth',1)
vaf = max(0,(1-norm(y-y_hat)^2./norm(y)^2)*100); 
ylabel('\boldmath{$\theta$ $[deg]$}','interpreter','latex')
legend('\textbf{Measured}','\textbf{Identified}','interpreter','latex')
title(['\textbf{Theta VAF:} \boldmath{$',num2str(round(vaf,2)),'\%$}'],'interpreter','latex')

subplot(3,1,3); % alpha angle
hold on;grid on;box on; 
y_hat = data.y(:,2); 
y = data_val.y(:,2);
plot(time,y*180/pi,'Linewidth',1)
plot(time,y_hat*180/pi,'Linewidth',1)
vaf = max(0,(1-norm(y-y_hat)^2./norm(y)^2)*100); 
ylabel('\boldmath{$\alpha$ $[deg]$}','interpreter','latex')
legend('\textbf{Measured}','\textbf{Identified}','interpreter','latex')
title(['\textbf{Alpha VAF:} \boldmath{$',num2str(round(vaf,2)),'\%$}'],'interpreter','latex')
xlabel('\textbf{Time [s]}','interpreter','latex')

end

%% Simulation and Discretization
%
Tend = theta(end,1); % simulation time 

% Discretize identified system
ident_sys_ss = ss(ident_sys.A,ident_sys.B,ident_sys.C,ident_sys.D); % continuous
ident_sys_ss_d = c2d(ident_sys_ss,h); % discrete

%{
% Simulate Response
x_lin = zeros(4,Tend/h);
for i=1:Tend/h-1
    x_lin(:,i+1) = ident_sys_ss_d.A*x_lin(:,i) + ident_sys_ss_d.B*Vs_ident(i);
end    

% Plot Results
figure
subplot(2,2,1)
plot(0:h:Tend-h,x_lin(1,:)*180/pi)
ylabel('\boldmath{$\theta$ $[Deg]$}','interpreter','latex')
title('\textbf{Angle} \boldmath{$\theta$}', 'interpreter','latex')

subplot(2,2,2)
plot(0:h:Tend-h,x_lin(2,:)*180/pi)
ylabel('\boldmath{$\dot \theta$ $[Deg/s]$}','interpreter','latex')
title('\textbf{Velocity} \boldmath{$\theta$}', 'interpreter','latex')

subplot(2,2,3)
plot(0:h:Tend-h,x_lin(3,:)*180/pi)
ylabel('\boldmath{$\alpha$ $[Deg]$}','interpreter','latex')
title('\textbf{Angle} \boldmath{$\alpha$}', 'interpreter','latex')

subplot(2,2,4)
plot(0:h:Tend-h,x_lin(4,:)*180/pi)
ylabel('\boldmath{$\dot \alpha$ $[Deg/s]$}','interpreter','latex')
title('\textbf{Velocity} \boldmath{$\alpha$}', 'interpreter','latex')

%}
%% 

% Identified system for downward equilibrium (stable)
sys_down = ident_sys_ss; % continuous
sys_down_d = ident_sys_ss_d; % discrete

% State space dynamics for the upright position
% The state space of linearized system at the upright position is similar to
% the state space of linearized system at the downward position but just with
% few different signs at some entries of matrices A and B

% Identified system for upright equilibrium (unstable)
sys_up = ss([],[],[],[]);
sys_up.A = [sys_down.A(1,:);
            sys_down.A(2,1:3), -sys_down.A(2,4);
            sys_down.A(3,:);
            -sys_down.A(4,1:3), sys_down.A(4,4)];
sys_up.B = [0; sys_down.B(2); 0; -sys_down.B(4)]; 
sys_up.C = ident_sys.C; 
sys_up.D = ident_sys.D;
sys_up_d = c2d(sys_up,h);

%%  Identification of Upright Position
%{
% Load Data
addpath('LQR Control Tuning')
load('LQR1.mat', 'theta', 'alpha', 'u')


% Validation data
y_val = [theta.signals(2).values(5/h+1:end)*pi/180, alpha.signals(1).values(5/h+1:end)];
Vs_val = u(5/h+1:end,2);
data_val = iddata(y_val, Vs_val, h, 'Name', 'Qube_Rotary_Pendulum');

% ident_sys_up = ident_sys;
% Plot Results
data = compare(data_val,ident_sys);
figure;
subplot(3,1,1); % input signal
time = u(5/h+1:end,1)-5;
plot(time,Vs_val,'Linewidth',1);
grid on;box on;
title(['\textbf{Input signal: ',signal,'}'],'interpreter','latex')
ylabel('\boldmath{$V_s$ $[V]$}','interpreter','latex')
ylim([min(u(:,2))-0.2, max(u(:,2))+0.2])

subplot(3,1,2); % theta angle
hold on;grid on;box on; 
y_hat = data.y(:,1); 
y = data_val.y(:,1);
plot(time,y*180/pi,'Linewidth',1)
plot(time,y_hat*180/pi,'Linewidth',1)
vaf = max(0,(1-norm(y-y_hat)^2./norm(y)^2)*100); 
ylabel('\boldmath{$\theta$ $[deg]$}','interpreter','latex')
legend('\textbf{Measured}','\textbf{Identified}','interpreter','latex')
title(['\textbf{Theta VAF:} \boldmath{$',num2str(round(vaf,2)),'\%$}'],'interpreter','latex')

subplot(3,1,3); % alpha angle
hold on;grid on;box on; 
y_hat = data.y(:,2); 
y = data_val.y(:,2);
plot(time,y*180/pi,'Linewidth',1)
plot(time,y_hat*180/pi,'Linewidth',1)
vaf = max(0,(1-norm(y-y_hat)^2./norm(y)^2)*100); 
ylabel('\boldmath{$\alpha$ $[deg]$}','interpreter','latex')
legend('\textbf{Measured}','\textbf{Identified}','interpreter','latex')
title(['\textbf{Alpha VAF:} \boldmath{$',num2str(round(vaf,2)),'\%$}'],'interpreter','latex')
xlabel('\textbf{Time [s]}','interpreter','latex')
%%
% Simulate Response
Tend = time(end);
x_lin = zeros(4,Tend/h);
for i=1:Tend/h
%     u(i) = Klqr2*([theta.signals(1).values(5/h+i)*pi/180;0;0;0] - x_lin(:,i));
    x_lin(:,i+1) = sys_up_d.A*x_lin(:,i) + sys_up_d.B*Vs_val(i);
end 

figure
subplot(2,1,1)
plot(time,x_lin(1,:)*180/pi)
ylabel('\boldmath{$\theta$ $[Deg]$}','interpreter','latex')
title('\textbf{Angle} \boldmath{$\theta$}', 'interpreter','latex')

subplot(2,1,2)
plot(time,x_lin(3,:)*180/pi)
ylabel('\boldmath{$\alpha$ $[Deg]$}','interpreter','latex')
title('\textbf{Angle} \boldmath{$\alpha$}', 'interpreter','latex')
%}

save('Identified_Linear_Model.mat','sys_up_d','sys_down_d','ident_sys','sys_up')