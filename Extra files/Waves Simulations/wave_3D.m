clc
clear all
close all

xlim = [0 10];
dx = 0.1;
ylim = [-1 1];
x = xlim(1):dx:xlim(2); % horizontal distance
x = x(1:end-1);

y = linspace(-1,1,length(x));

L = 10;  % wavelength
omega = 5;        % wave speed
A = 1;      % wave amplitude 
dt = 0.1; % sampling time
tend = 40; % duration of simulation in seconds
t = 0:dt:tend;
z = A*sin(omega*t(1) - (2*pi/L)*x);

[X,Y] = meshgrid(x,y);


Z = ones(100,1)*z;
mesh(X,Y,Z,'EdgeColor','none','FaceColor','b','FaceAlpha','0.2')

