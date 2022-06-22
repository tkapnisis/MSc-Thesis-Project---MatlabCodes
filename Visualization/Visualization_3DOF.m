function Visualization_3DOF(x,param)

z_n = x(1);
phi = x(2);
theta = x(3);
psi = 0;

h_0 = 0.2; % Initial distance from water free surface

width_HEARP = 1;
length_HEARP = 1.5;
height_HEARP = 0.1;

% Rotation Matrix from Body Frame to NED Frame
R_phi = [1    0         0    ;...   % roll (x)
         0 cos(phi) -sin(phi);...
         0 sin(phi)  cos(phi)];
R_theta = [ cos(theta) 0 sin(theta);...  % pitch (y)
                0      1     0     ;...
           -sin(theta) 0 cos(theta)];
R_psi = [cos(psi) -sin(psi) 0;...  % yaw (z)
         sin(psi)  cos(psi) 0;...
           0         0     1];
   
R_BODY_NED= (R_psi*R_theta*R_phi);

CG_location = [length_HEARP/2; width_HEARP/2; h_0 + z_n];


% Dimensions
% Total x distances between the centre of pressure of hydrofoils and CG:
% param.l_x_f
% param.l_x_ap
% param.l_x_as
% Total y distances between the centre of pressure of hydrofoils and CG:
% param.l_y_f
% param.l_y_ap
% param.l_y_as
% Total z distances between the centre of pressure of hydrofoils and CG:
% param.l_z_f
% param.l_z_ap
% param.l_z_as

% Width of the HEARP frame
% width = 1;
% Length of HEARP frame
% length = 1.5;
% x location of CG with respect to the back midline of the frame
x_CG = 0.58;
%  y location of CG with respect to the back midline of the frame
y_CG = 0.02;

width_HEARP = 1;
length_HEARP = 1.5;
height_HEARP = 0.1;

CG_x = length_HEARP/2; 
CG_y = width_HEARP/2;
CG_z = h_0 + z_n;
dimensions = diag([length_HEARP,width_HEARP,height_HEARP]);
vert = [0 0 0;1 0 0;1 1 0;0 1 0;0 0 1;1 0 1;1 1 1;0 1 1]*dimensions;
fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];

% Plot of CG of HEARP
plot3(CG_location(1),CG_location(2),CG_location(3),'.','Color','b','MarkerSize',50)

hold on
axis vis3d
grid on
set(gca,'xlim', [0-0.2 length_HEARP+0.2], 'ylim', [0 width_HEARP], 'zlim', [-0.5 0.5],...
    'dataaspectratio', [1 1 1]);



% patch('Vertices',vert,'Faces',fac,'facealpha', 0.1)

point1_init = [-length_HEARP/2; 0; 0];
point2_init = [length_HEARP/2; 0; 0];

point1 = R_BODY_NED*point1_init + CG_location;
point2 = R_BODY_NED*point2_init + CG_location;

line([point1(1), point2(1)], [point1(2), point2(2)], [point1(3), point2(3)],...
     'LineWidth',2,'Color','red','LineStyle', '--')


point3_init = [-length_HEARP/2; 0; -height_HEARP];
point4_init = [ length_HEARP/2; 0; -height_HEARP];
R_BODY_NED*point3_init
R_BODY_NED*point4_init
point3 = R_BODY_NED*point3_init + CG_location
point4 = R_BODY_NED*point4_init + CG_location

line([point3(1), point4(1)], [point3(2), point4(2)], [point3(3), point4(3)],...
     'LineWidth',2)

% line([-length_HEARP*cos(theta),length_HEARP*cos(theta)],...
%      [CG_y,CG_y],...
%      [CG_z-length_HEARP*sin(theta),CG_z+length_HEARP*sin(theta)],...
%      'LineWidth',2)



%drawnow
hold off