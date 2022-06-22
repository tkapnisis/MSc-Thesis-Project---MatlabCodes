%% Optical simulation

% Use low pass filter for the distance sensor measurements
fs = 20;
fpass = 2;
distance_sensor_f = lowpass(distance_sensor,fpass,fs);

figure
set(gcf, 'WindowState', 'maximized');
view(3)
start_time = 5900;
x = [distance_sensor_f(start_time:9011), deg2rad(attitude_eul(start_time:9011,3)),...
     deg2rad(attitude_eul(start_time:9011,2))];
t = time_attidute(start_time:9011);
for k=1:length(t)
    Visualization_3DOF(x(k,:));
    title(['t= ',num2str(t(k),4),' [s]']);
    xlabel('\boldmath{$x_n$} \textbf{[m]}','interpreter','latex','FontSize',15,'Interpreter','latex')
    ylabel('\boldmath{$y_n$} \textbf{[m]}','interpreter','latex','FontSize',15,'Interpreter','latex')
    zlabel('\boldmath{$z_n$} \textbf{[m]}','interpreter','latex','FontSize',15,'Interpreter','latex')
%     view(0,90) % for view x-y axis
    view(0,0) % for view x-z axis
%     view(90,0) % for view y-z axis
    drawnow
    pause(1e-2)
end