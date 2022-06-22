function Visualization_3DOF(x)

z_n = x(1);
phi = x(2);
theta = x(3);
psi = 0;

h_0 = 0; % Initial distance from water free surface

% Rotation Matrix from Body Frame to NED Frame (z-axis looks downards)
R_phi = [1    0         0    ;...   % roll (x)
         0 cos(phi) -sin(phi);...
         0 sin(phi)  cos(phi)];
R_theta = [ cos(theta) 0 sin(theta);...  % pitch (y)
                0      1     0     ;...
           -sin(theta) 0 cos(theta)];
R_psi = [cos(psi) -sin(psi) 0;...  % yaw (z)
         sin(psi)  cos(psi) 0;...
           0         0     1];
R_BODY_NED= (R_psi*R_theta*R_phi)'; % use transpose to rotate the z-axis

width_HEARP = 1; % Width of the HEARP frame
length_HEARP = 1.5; % Length of HEARP frame
height_HEARP = 0.1; % Height of HEARP frame

% x location of CG with respect to the back midpoint of the frame
x_CG = 0.58;
%  y location of CG with respect to the midline of the frame
y_CG = 0.02 + width_HEARP/2;
%  z location of CG with respect to upper part of the frame (is at 0)
z_CG = h_0 + z_n;

CG_location = [x_CG; y_CG; z_CG];

frame_vert = [-x_CG, width_HEARP-y_CG, 0;
              length_HEARP-x_CG, width_HEARP-y_CG, 0;
              length_HEARP-x_CG, -y_CG, 0;
              -x_CG, -y_CG, 0;
              -x_CG, width_HEARP-y_CG, height_HEARP;
              length_HEARP-x_CG, width_HEARP-y_CG, height_HEARP;
              length_HEARP-x_CG, -y_CG, height_HEARP;
              -x_CG, -y_CG, height_HEARP];
frame_fac = [1 2 3 4; 1 2 6 5; 1 4 8 5; 3 4 8 7; 2 3 7 6; 5 6 7 8];

frame_coordinates = (R_BODY_NED*frame_vert' + diag(CG_location)*ones(3,8))';

% Plot of CG of HEARP
plot3(CG_location(1),CG_location(2),CG_location(3),'.','Color','k','MarkerSize',50)

x_lim = [-0.1, length_HEARP+0.1];
y_lim = [-0.2 width_HEARP+0.2];
z_lim = [-0.3 0.6];

hold on
grid on
set(gca,'xlim', x_lim, 'ylim', y_lim, 'zlim', z_lim, 'dataaspectratio', [1 1 1]);

patch('Vertices',frame_coordinates,'Faces',frame_fac,'facealpha', 0.4,'FaceColor','red')

water_vert = [x_lim(1) y_lim(1) z_lim(1); x_lim(2) y_lim(1) z_lim(1);...
              x_lim(2) y_lim(2) z_lim(1); x_lim(1) y_lim(2) z_lim(1);...
              x_lim(1) y_lim(1) 0; x_lim(2) y_lim(1) 0;...
              x_lim(2) y_lim(2) 0; x_lim(1) y_lim(2) 0];
water_fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
patch('Vertices',water_vert,'Faces',water_fac,'facealpha', 0.5,'FaceColor','blue')


% point1_init = [-x_CG; 0; 0];
% point2_init = [length_HEARP-x_CG; 0; 0];
% 
% point1 = R_BODY_NED*point1_init + CG_location
% point2 = R_BODY_NED*point2_init + CG_location
% 
% line([point1(1), point2(1)], [point1(2), point2(2)], [point1(3), point2(3)],...
%      'LineWidth',2,'Color','red','LineStyle', '-')


% point3_init = [-length_HEARP/2; 0; -height_HEARP];
% point4_init = [ length_HEARP/2; 0; -height_HEARP];
% 
% point3 = R_BODY_NED*point3_init + CG_location;
% point4 = R_BODY_NED*point4_init + CG_location;
% 
% line([point3(1), point4(1)], [point3(2), point4(2)], [point3(3), point4(3)],...
%      'LineWidth',2)

hold off

end

