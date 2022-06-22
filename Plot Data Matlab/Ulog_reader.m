clc
clear all
close all

addpath('HEARP')
addpath('RacingBoatSR65')
ulog = ulogreader('log_1_2022-5-25-13-19-26.ulg');
params = readParameters(ulog);
systeminfo = readSystemInformation(ulog);
loggedoutput = readLoggedOutput(ulog);
msg = readTopicMsgs(ulog);

d1 = ulog.StartTime;
d2 = ulog.EndTime;
%%
data_attitude = readTopicMsgs(ulog,'TopicNames',{'vehicle_attitude'}, ... 
'InstanceID',{0},'Time',[d1 d2]);
data_angular_velocity = readTopicMsgs(ulog,'TopicNames',{'vehicle_angular_velocity'}, ... 
'InstanceID',{0},'Time',[d1 d2]);
% data_angular_acceleration = readTopicMsgs(ulog,'TopicNames',{'vehicle_angular_acceleration'}, ... 
% 'InstanceID',{0},'Time',[d1 d2]);
data_distance_sensor = readTopicMsgs(ulog,'TopicNames',{'distance_sensor'}, ... 
'InstanceID',{0},'Time',[d1 d2]);
data_local_position_z = readTopicMsgs(ulog,'TopicNames',{'vehicle_local_position'}, ... 
'InstanceID',{0},'Time',[d1 d2]);
 
% attitude_quad = timetable2table(data_attitude.TopicMessages{1,1}(:,2));
attitude_quad = timetable2table(data_attitude.TopicMessages{1,1}(:,1));
attitude_quad = table2array(attitude_quad(:,2));
attitude_eul = rad2deg(quat2eul(attitude_quad));
% attitude_eul = quat2eul(attitude_quad);

angular_velocity = timetable2table(data_angular_velocity.TopicMessages{1,1}(:,2));
angular_velocity = rad2deg(table2array(angular_velocity(:,2)));
% angular_velocity = table2array(angular_velocity(:,2));

% angular_acceleration = timetable2table(data_angular_acceleration.TopicMessages{1,1}(:,2));
% angular_acceleration = rad2deg(table2array(angular_acceleration(:,2)));

distance_sensor = timetable2table(data_distance_sensor.TopicMessages{1,1}(:,3));
distance_sensor = table2array(distance_sensor(:,2));

local_position_z = timetable2table(data_local_position_z.TopicMessages{1,1}(:,6));
local_position_z = table2array(local_position_z(:,2));

time_attidute = seconds(data_attitude.TopicMessages{1,1}.timestamp);
time_attidute = time_attidute - time_attidute(1);

time_angular_velocity = seconds(data_angular_velocity.TopicMessages{1,1}.timestamp);
time_angular_velocity = time_angular_velocity - time_angular_velocity(1);

% time_angular_acceleration = seconds(data_angular_acceleration.TopicMessages{1,1}.timestamp);
% time_angular_acceleration = time_angular_acceleration - time_angular_acceleration(1);

time_distance_sensor = seconds(data_distance_sensor.TopicMessages{1,1}.timestamp);
time_distance_sensor = time_distance_sensor - time_distance_sensor(1);

time_local_position_z = seconds(data_local_position_z.TopicMessages{1,1}.timestamp);
time_local_position_z = time_local_position_z - time_local_position_z(1);
%%
figure
subplot(3,1,1)
plot(time_attidute,attitude_eul(:,3),'LineWidth',1.5)
hold on
line([0 time_attidute(end)],[0 0],'LineWidth',1,'Color','k')
title('Roll')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\phi$} \textbf{[deg]}','interpreter','latex')
xlim([0, time_attidute(end)])
grid minor
subplot(3,1,2)
plot(time_attidute,attitude_eul(:,2),'LineWidth',1.5)
hold on
line([0 time_attidute(end)],[0 0],'LineWidth',1,'Color','k')
title('Pitch')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\theta$} \textbf{[deg]}','interpreter','latex')
xlim([0, time_attidute(end)])
grid minor
subplot(3,1,3)
plot(time_attidute,attitude_eul(:,1),'LineWidth',1.5)
hold on
line([0 time_attidute(end)],[0 0],'LineWidth',1,'Color','k')
title('Yaw')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\psi$} \textbf{[deg]}','interpreter','latex')
xlim([0, time_attidute(end)])
grid minor

figure
subplot(3,1,1)
plot(time_angular_velocity,angular_velocity(:,1),'LineWidth',1.5)
hold on
line([0 time_angular_velocity(end)],[0 0],'LineWidth',1,'Color','k')
title('Roll Angular Rate')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$p$} \textbf{[deg/s]}','interpreter','latex')
xlim([0, time_angular_velocity(end)])
grid minor
subplot(3,1,2)
plot(time_angular_velocity,angular_velocity(:,2),'LineWidth',1.5)
hold on
line([0 time_angular_velocity(end)],[0 0],'LineWidth',1,'Color','k')
title('Pitch Angular Rate')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$q$} \textbf{[deg/s]}','interpreter','latex')
xlim([0, time_angular_velocity(end)])
grid minor
subplot(3,1,3)
plot(time_angular_velocity,angular_velocity(:,3),'LineWidth',1.5)
hold on
line([0 time_angular_velocity(end)],[0 0],'LineWidth',1,'Color','k')
title('Yaw Angular Rate')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$r$} \textbf{[deg/s]}','interpreter','latex')
xlim([0, time_angular_velocity(end)])
grid minor

figure
hold on
plot(time_distance_sensor,distance_sensor(:,1),'LineWidth',1.5)
plot(time_local_position_z,local_position_z(:,1),'LineWidth',1.5)
line([0 time_local_position_z(end)],[0 0],'LineWidth',1,'Color','k')
title('Distance from free-surface of water')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$h$} \textbf{[m]}','interpreter','latex')
xlim([0, time_angular_velocity(end)])
ylim([-0.5, 0.5])
grid minor
legend('Distance sensor','Z_n')



%%

data_optical_flow = readTopicMsgs(ulog,'TopicNames',{'optical_flow'}, ... 
'InstanceID',{0},'Time',[d1 d2]);
 
optical_flow = timetable2table(data_optical_flow.TopicMessages{1,1}(:,1:6));
optical_flow = table2array(optical_flow(:,2:end));

time_optical_flow = seconds(data_optical_flow.TopicMessages{1,1}.timestamp);
time_optical_flow = time_optical_flow - time_optical_flow(1);

figure
subplot(3,1,1)
plot(time_optical_flow,optical_flow(:,1),'LineWidth',1.5)
title('Pixel flow x integral')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$x$} \textbf{[m]}','interpreter','latex')
xlim([0, time_angular_velocity(end)])
grid minor
subplot(3,1,2)
plot(time_optical_flow,optical_flow(:,2),'LineWidth',1.5)
title('Pixel flow y integral')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$y$} \textbf{[m]}','interpreter','latex')
xlim([0, time_angular_velocity(end)])
grid minor
subplot(3,1,3)
plot(time_optical_flow,optical_flow(:,6),'LineWidth',1.5)
title('Distance')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$z$} \textbf{[m]}','interpreter','latex')
xlim([0, time_angular_velocity(end)])
grid minor

figure
subplot(3,1,1)
plot(time_optical_flow,optical_flow(:,3),'LineWidth',1.5)
title('gyro x rate integral')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$dx$} \textbf{[m/s]}','interpreter','latex')
xlim([0, time_angular_velocity(end)])
grid minor
subplot(3,1,2)
plot(time_optical_flow,optical_flow(:,4),'LineWidth',1.5)
title('gyro y rate_integral')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$dx$} \textbf{[m/s]}','interpreter','latex')
xlim([0, time_angular_velocity(end)])
grid minor
subplot(3,1,3)
plot(time_optical_flow,optical_flow(:,5),'LineWidth',1.5)
title('gyro z rate integral')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$dx$} \textbf{[m/s]}','interpreter','latex')
xlim([0, time_angular_velocity(end)])
grid minor

%%
data_vehicle_local_position = readTopicMsgs(ulog,'TopicNames',{'vehicle_local_position'}, ... 
'InstanceID',{0},'Time',[d1 d2]);

vehicle_local_position = timetable2table(data_vehicle_local_position.TopicMessages{1,1}(:,1:19));
vehicle_local_position = table2array(vehicle_local_position(:,6:end));

time_vehicle_local_position = seconds(data_vehicle_local_position.TopicMessages{1,1}.timestamp);
time_vehicle_local_position = time_vehicle_local_position - time_vehicle_local_position(1);
%%
figure
subplot(3,1,1)
plot(time_vehicle_local_position,vehicle_local_position(:,7),'LineWidth',1.5)
hold on
line([0 time_vehicle_local_position(end)],[0 0],'LineWidth',1,'Color','k')
title('Vx')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$dxdt$} \textbf{[m/s]}','interpreter','latex')
xlim([0, time_vehicle_local_position(end)])
grid minor
subplot(3,1,2)
plot(time_vehicle_local_position,vehicle_local_position(:,8),'LineWidth',1.5)
hold on
line([0 time_vehicle_local_position(end)],[0 0],'LineWidth',1,'Color','k')
title('Vy')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$dydt$} \textbf{[m/s]}','interpreter','latex')
xlim([0, time_vehicle_local_position(end)])
grid minor
subplot(3,1,3)
plot(time_vehicle_local_position,vehicle_local_position(:,9),'LineWidth',1.5)
hold on
line([0 time_vehicle_local_position(end)],[0 0],'LineWidth',1,'Color','k')
title('Vz')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$dzdt$} \textbf{[m/s]}','interpreter','latex')
xlim([0, time_vehicle_local_position(end)])
grid minor

%%
data_vehicle_acceleration = readTopicMsgs(ulog,'TopicNames',{'vehicle_acceleration'}, ... 
'InstanceID',{0},'Time',[d1 d2]);

vehicle_acceleration = timetable2table(data_vehicle_acceleration.TopicMessages{1,1}(:,2));
vehicle_acceleration = table2array(vehicle_acceleration(:,2));

time_vehicle_acceleration = seconds(data_vehicle_acceleration.TopicMessages{1,1}.timestamp);
time_vehicle_acceleration = time_vehicle_acceleration - time_vehicle_acceleration(1);
%%
figure
subplot(3,1,1)
plot(time_vehicle_acceleration,vehicle_acceleration(:,1),'LineWidth',1.5)
hold on
line([0 time_vehicle_acceleration(end)],[0 0],'LineWidth',1,'Color','k')
title('ax')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$dudt$} \boldmath{$[m^2/s]$}','interpreter','latex')
xlim([0, time_vehicle_acceleration(end)])
grid minor
subplot(3,1,2)
plot(time_vehicle_acceleration,vehicle_acceleration(:,2),'LineWidth',1.5)
hold on
line([0 time_vehicle_acceleration(end)],[0 0],'LineWidth',1,'Color','k')
title('ay')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$dvdt$} \boldmath{$[m^2/s]$}','interpreter','latex')
xlim([0, time_vehicle_acceleration(end)])
grid minor
subplot(3,1,3)
plot(time_vehicle_acceleration,vehicle_acceleration(:,3),'LineWidth',1.5)
hold on
line([0 time_vehicle_acceleration(end)],[0 0],'LineWidth',1,'Color','k')
title('az')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$dwdt$} \boldmath{$[m^2/s]$}','interpreter','latex')
xlim([0, time_vehicle_acceleration(end)])
grid minor