clc
clear all
close all

xlim = [0 10];
dx = 0.1;
d = 4;   % depth
ylim = [-2 2];
zlim = [-d 2];

x = xlim(1):dx:xlim(2); % horizontal distance
x = x(1:end-1);
y = linspace(ylim(1),ylim(2),length(x));
L = 10;  % wavelength
omega = 5;        % wave speed
% T = 2*pi/omega;
% c = L/T;
A = 1;      % wave amplitude 
x_bar = [xlim(2)/2, xlim(2)/2, xlim(2)/2];
z_bar = [-1,-2,-2.5];
dt = 0.1; % sampling time
tend = 20; % duration of simulation in seconds
t = 0:dt:tend-dt;

[X,Y] = meshgrid(x,y);

tic;
figure
view(3)
% set(gcf, 'WindowState', 'maximized');
% Make an animation: 
for k=1:length(t)
    waves_sim_3D(x,X,Y,t,k,L,A,omega,xlim,zlim)
    title(['t= ',num2str(t(k),3),' [s]']); 
    xlabel('x (m)')
    ylabel('y (m)') 
    zlabel('z (m)') 
    axis equal
    axis([xlim, ylim, zlim])
%     view(0,90) % for view x-y axis
%     view(0,0) % for view x-z axis
%     view(90,0) % for view y-z axis

    real_dt = toc;
    while real_dt<=dt
        pause(1e-3)
        real_dt = toc;
    end
    drawnow
    tic;
end