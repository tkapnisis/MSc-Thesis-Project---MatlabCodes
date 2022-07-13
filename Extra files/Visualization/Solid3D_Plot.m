width = 1;
length = 1.5;

L = 10;  % wavelength
omega = 5;        % wave speed
A = 1;      % wave amplitude 
dt = 0.1; % sampling time
tend = 40; % duration of simulation in seconds
t = 0:dt:tend;
z = A*sin(omega*t(k) - (2*pi/L)*x);
height = 0.1;
figure
hold on
water = [length+0.2, width+0.2, -0.2];
water_vert = [0 0 0; water(1) 0 0; water(1) water(2) 0; 0 water(2) 0;0 0 water(3);...
        water(1) 0 water(3);water(1) water(2) water(3);0 water(2) water(3)];
water_fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
patch('Vertices',water_vert,'Faces',water_fac,'facealpha', 0.2,'FaceColor','blue')


% fig = gca;
view(3)
axis equal
grid on

% set(fig, 'xlim', [0 length], 'ylim', [0 width], 'zlim', [0 height], ...
%     'dataaspectratio', [1 1 1]); % last one same as "axis equal"
% set(fig, 'dataaspectratio', [1 1 1]); % last one same as "axis equal"