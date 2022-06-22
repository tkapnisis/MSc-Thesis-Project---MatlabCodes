clc
clear all
close all

xlim = [0 10];
dx = 0.1;
d = 2;   % depth
ylim = [-d 1];

x = xlim(1):dx:xlim(2); % horizontal distance
x = x(1:end);
y = linspace(-1,1,200);
L = 8;  % wavelength
omega = 2;        % wave speed
T = 2*pi/omega;
c = L/T;
A = 0.3;      % wave amplitude 
x_bar = [xlim(2)/2, xlim(2)/2, xlim(2)/2];
z_bar = [-0.2,-0.8,-1.5];
dt = 0.1; % sampling time
tend = 15; % duration of simulation in seconds
t = 0:dt:tend;

u_x_store = [];
u_z_store = [];

tic;
figure
view(3)
set(gcf, 'WindowState', 'maximized');
% Make an animation: 
for k=1:length(t)
    [u_x,u_z] = waves_sim_2D(x,t,k,L,A,omega,xlim,ylim,d,x_bar,z_bar);
    u_x_store = [u_x_store;u_x];
    u_z_store = [u_z_store;u_z];
    title(['t= ',num2str(t(k),3),' [s]']); 
    xlabel('distance (m)')
    ylabel('wave height (m)') 
    axis equal
    axis([xlim ylim])

    real_dt = toc;
    while real_dt<=dt
        pause(1e-4)
        real_dt = toc;
    end
    drawnow
    tic;
end