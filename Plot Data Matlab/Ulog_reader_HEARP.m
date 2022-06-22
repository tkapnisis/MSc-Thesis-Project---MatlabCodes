clc
clear all
close all

addpath('HEARP')
addpath('RacingBoatSR65')
ulog = ulogreader('log_8_2022-4-29-17-09-36.ulg');
params = readParameters(ulog);
systeminfo = readSystemInformation(ulog);
loggedoutput = readLoggedOutput(ulog);
msg = readTopicMsgs(ulog);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Uncomment if the distance sensor start time do not match with all the
% other start times
% start_end_time = readTopicMsgs(ulog,'TopicNames',{'distance_sensor'}, ... 
% 'InstanceID',{0});
% % % d1 = start_end_time.StartTimestamp;
% d1 = start_end_time.TopicMessages{1,1}.timestamp(2);
% d2 = start_end_time.LastTimestamp;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% StartTimestamp
d1 = ulog.StartTime;
d2 = ulog.EndTime;
%% Vehicle Euler angles and angular velocities
data_attitude = readTopicMsgs(ulog,'TopicNames',{'vehicle_attitude'}, ... 
'InstanceID',{0},'Time',[d1 d2]);
data_angular_velocity = readTopicMsgs(ulog,'TopicNames',{'vehicle_angular_velocity'}, ... 
'InstanceID',{0},'Time',[d1 d2]);

attitude_quad = timetable2table(data_attitude.TopicMessages{1,1}(:,1));
attitude_quad = table2array(attitude_quad(:,2));
attitude_eul = rad2deg(quat2eul(attitude_quad));
% attitude_eul = quat2eul(attitude_quad);

angular_velocity = timetable2table(data_angular_velocity.TopicMessages{1,1}(:,2));
angular_velocity = rad2deg(table2array(angular_velocity(:,2)));
% angular_velocity = table2array(angular_velocity(:,2));

time_attidute = seconds(data_attitude.TopicMessages{1,1}.timestamp);
time_attidute = time_attidute - time_attidute(1);

time_angular_velocity = seconds(data_angular_velocity.TopicMessages{1,1}.timestamp);
time_angular_velocity = time_angular_velocity - time_angular_velocity(1);

figure
subplot(3,2,1)
plot(time_attidute,attitude_eul(:,3),'LineWidth',1.5)
hold on
line([0 time_attidute(end)],[0 0],'LineWidth',1,'Color','k')
title('Roll')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\phi$} \textbf{[deg]}','interpreter','latex')
xlim([0, time_attidute(end)])
grid minor
subplot(3,2,3)
plot(time_attidute,attitude_eul(:,2),'LineWidth',1.5)
hold on
line([0 time_attidute(end)],[0 0],'LineWidth',1,'Color','k')
title('Pitch')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\theta$} \textbf{[deg]}','interpreter','latex')
xlim([0, time_attidute(end)])
grid minor
subplot(3,2,5)
plot(time_attidute,attitude_eul(:,1),'LineWidth',1.5)
hold on
line([0 time_attidute(end)],[0 0],'LineWidth',1,'Color','k')
title('Yaw')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\psi$} \textbf{[deg]}','interpreter','latex')
xlim([0, time_attidute(end)])
grid minor

subplot(3,2,2)
plot(time_angular_velocity,angular_velocity(:,1),'LineWidth',1.5)
hold on
line([0 time_angular_velocity(end)],[0 0],'LineWidth',1,'Color','k')
title('Roll Angular Rate')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$p$} \textbf{[deg/s]}','interpreter','latex')
xlim([0, time_angular_velocity(end)])
grid minor
subplot(3,2,4)
plot(time_angular_velocity,angular_velocity(:,2),'LineWidth',1.5)
hold on
line([0 time_angular_velocity(end)],[0 0],'LineWidth',1,'Color','k')
title('Pitch Angular Rate')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$q$} \textbf{[deg/s]}','interpreter','latex')
xlim([0, time_angular_velocity(end)])
grid minor
subplot(3,2,6)
plot(time_angular_velocity,angular_velocity(:,3),'LineWidth',1.5)
hold on
line([0 time_angular_velocity(end)],[0 0],'LineWidth',1,'Color','k')
title('Yaw Angular Rate')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$r$} \textbf{[deg/s]}','interpreter','latex')
xlim([0, time_angular_velocity(end)])
grid minor

%% Vehicle Local Position and Velocity
data_vehicle_local_position = readTopicMsgs(ulog,'TopicNames',{'vehicle_local_position'}, ... 
'InstanceID',{0},'Time',[d1 d2]);

vehicle_local_position = timetable2table(data_vehicle_local_position.TopicMessages{1,1}(:,1:19));

local_position = table2array(vehicle_local_position(:,5:7));
local_velocity = table2array(vehicle_local_position(:,10:12));

time_vehicle_local_position = seconds(data_vehicle_local_position.TopicMessages{1,1}.timestamp);
time_vehicle_local_position = time_vehicle_local_position - time_vehicle_local_position(1);

figure
subplot(3,2,1)
plot(time_vehicle_local_position,local_position(:,1),'LineWidth',1.5)
hold on
line([0 time_vehicle_local_position(end)],[0 0],'LineWidth',1,'Color','k')
title('x position')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$x$} \textbf{[m]}','interpreter','latex')
xlim([0, time_vehicle_local_position(end)])
grid minor

subplot(3,2,3)
plot(time_vehicle_local_position,local_position(:,2),'LineWidth',1.5)
hold on
line([0 time_vehicle_local_position(end)],[0 0],'LineWidth',1,'Color','k')
title('y position')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$y$} \textbf{[m]}','interpreter','latex')
xlim([0, time_vehicle_local_position(end)])
grid minor

subplot(3,2,5)
plot(time_vehicle_local_position,local_position(:,3),'LineWidth',1.5)
hold on
line([0 time_vehicle_local_position(end)],[0 0],'LineWidth',1,'Color','k')
title('z position')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$z$} \textbf{[m]}','interpreter','latex')
xlim([0, time_vehicle_local_position(end)])
grid minor

subplot(3,2,2)
plot(time_vehicle_local_position,local_velocity(:,1),'LineWidth',1.5)
hold on
line([0 time_vehicle_local_position(end)],[0 0],'LineWidth',1,'Color','k')
title('Surge velocity')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$u$} \textbf{[m/s]}','interpreter','latex')
xlim([0, time_vehicle_local_position(end)])
grid minor

subplot(3,2,4)
plot(time_vehicle_local_position,local_velocity(:,2),'LineWidth',1.5)
hold on
line([0 time_vehicle_local_position(end)],[0 0],'LineWidth',1,'Color','k')
title('Sway velocity')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$v$} \textbf{[m/s]}','interpreter','latex')
xlim([0, time_vehicle_local_position(end)])
grid minor
subplot(3,2,6)
plot(time_vehicle_local_position,local_velocity(:,3),'LineWidth',1.5)
hold on
line([0 time_vehicle_local_position(end)],[0 0],'LineWidth',1,'Color','k')
title('Heave velocity')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$w$} \textbf{[m/s]}','interpreter','latex')
xlim([0, time_vehicle_local_position(end)])
grid minor

%% Distance Sensor
data_distance_sensor = readTopicMsgs(ulog,'TopicNames',{'distance_sensor'}, ... 
'InstanceID',{0},'Time',[d1 d2]);


distance_sensor = timetable2table(data_distance_sensor.TopicMessages{1,1}(:,3));
distance_sensor = table2array(distance_sensor(:,2));

time_distance_sensor = seconds(data_distance_sensor.TopicMessages{1,1}.timestamp);
time_distance_sensor = time_distance_sensor - time_distance_sensor(1);

%%
fs = 20;
[y_ds,t_ds] = resample(double(distance_sensor),time_distance_sensor,fs);
[y_at, t_at] = resample(attitude_eul,time_attidute,fs);

fpass = 2;
y_ds_f=lowpass(y_ds,fpass,fs);
y_at_f=lowpass(y_at,fpass,fs);

%%
theta = deg2rad(y_at(:,2));
theta_f = deg2rad(y_at_f(:,2));

l_z_hs = -0.13;
l_x_hs = 0.7;
z_n = -y_ds(1:end-2) - l_z_hs*cos(theta) + l_x_hs*sin(theta);
z_n_f = -y_ds_f(1:end-2) - l_z_hs*cos(theta_f) + l_x_hs*sin(theta_f);
figure
hold on
plot(t_ds,y_ds,'LineWidth',1.5)
plot(t_ds,y_ds_f,'LineWidth',1.5)
plot(t_at,z_n,'LineWidth',1.5)
plot(t_at,z_n_f,'LineWidth',1.5)
line([0 time_vehicle_local_position(end)],[0 0],'LineWidth',1,'Color','k')
title('Distance from free-surface of water')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$h$} \textbf{[m]}','interpreter','latex')
xlim([0, time_vehicle_local_position(end)])
% ylim([-0.6, 0.6])
grid minor
legend('Distance sensor','Z_n')

%% Actuator outputs
data_actuator_outputs_0 = readTopicMsgs(ulog,'TopicNames',{'actuator_outputs'}, ... 
'InstanceID',{0},'Time',[d1 d2]);
data_actuator_outputs_1 = readTopicMsgs(ulog,'TopicNames',{'actuator_outputs'}, ... 
'InstanceID',{1},'Time',[d1 d2]);

actuator_outputs_0 = timetable2table(data_actuator_outputs_0.TopicMessages{1,1}(:,2));
actuator_outputs_0 = table2array(actuator_outputs_0(:,2));

actuator_outputs_1 = timetable2table(data_actuator_outputs_1.TopicMessages{1,1}(:,2));
actuator_outputs_1 = table2array(actuator_outputs_1(:,2));

time_actuator_outputs = seconds(data_actuator_outputs_0.TopicMessages{1,1}.timestamp);
time_actuator_outputs = time_actuator_outputs - time_actuator_outputs(1);

figure
hold on
plot(time_actuator_outputs,actuator_outputs_0(:,1),'LineWidth',1.5)
plot(time_actuator_outputs,actuator_outputs_0(:,3),'LineWidth',1.5)
plot(time_actuator_outputs,actuator_outputs_0(:,5),'LineWidth',1.5)
line([0 time_actuator_outputs(end)],[1250, 1250],'LineWidth',1,'Color','k','LineStyle','--')
title('Hydrofoils PWM signals')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\textbf{PWM [ms]}','interpreter','latex')
xlim([0, time_actuator_outputs(end)])
grid minor
legend('Fore','Aft starboard', 'Aft port','PWM DISARMED')

figure
subplot(2,1,1)
hold on
plot(time_actuator_outputs,actuator_outputs_0(:,7),'LineWidth',1.5)
line([0 time_actuator_outputs(end)],[1250, 1250],'LineWidth',1,'Color','k','LineStyle','--')
title('Rudder PWM signal')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\textbf{PWM [ms]}','interpreter','latex')
xlim([0, time_actuator_outputs(end)])
grid minor
legend('Rudder','PWM DISARMED')

subplot(2,1,2)
hold on
plot(time_actuator_outputs,actuator_outputs_1(:,5),'LineWidth',1.5)
line([0 time_actuator_outputs(end)],[900, 900],'LineWidth',1,'Color','k','LineStyle','--')
title('Propeller PWM signal')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\textbf{PWM [ms]}','interpreter','latex')
xlim([0, time_actuator_outputs(end)])
grid minor
legend('Propeller','PWM DISARMED')

%% Actuator controls
data_actuator_controls = readTopicMsgs(ulog,'TopicNames',{'actuator_controls_0'}, ... 
'InstanceID',{0},'Time',[d1 d2]);

actuator_controls = timetable2table(data_actuator_controls.TopicMessages{1,1}(:,2));
actuator_controls = table2array(actuator_controls(:,2));

time_actuator_controls = seconds(data_actuator_controls.TopicMessages{1,1}.timestamp);
time_actuator_controls = time_actuator_controls - time_actuator_controls(1);

figure
subplot(5,1,1)
plot(time_actuator_controls,actuator_controls(:,1),'LineWidth',1.5)
title('Roll controller signal')
line([0 time_actuator_controls(end)],[0 0],'LineWidth',1,'Color','k')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\textbf{Signal [-]}','interpreter','latex')
xlim([0, time_actuator_controls(end)])
ylim([-1, 1])
grid minor

subplot(5,1,2)
plot(time_actuator_controls,actuator_controls(:,2),'LineWidth',1.5)
title('Pitch controller signal')
line([0 time_actuator_controls(end)],[0 0],'LineWidth',1,'Color','k')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\textbf{Signal [-]}','interpreter','latex')
xlim([0, time_actuator_controls(end)])
ylim([-1, 1])
grid minor

subplot(5,1,3)
plot(time_actuator_controls,actuator_controls(:,3),'LineWidth',1.5)
title('Yaw controller signal')
line([0 time_actuator_controls(end)],[0 0],'LineWidth',1,'Color','k')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\textbf{Signal [-]}','interpreter','latex')
xlim([0, time_actuator_controls(end)])
ylim([-1, 1])
grid minor

subplot(5,1,4)
plot(time_actuator_controls,actuator_controls(:,4),'LineWidth',1.5)
title('Surge velocity controller signal')
line([0 time_actuator_controls(end)],[0 0],'LineWidth',1,'Color','k')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\textbf{Signal [-]}','interpreter','latex')
xlim([0, time_actuator_controls(end)])
ylim([-1, 1])
grid minor

subplot(5,1,5)
plot(time_actuator_controls,actuator_controls(:,5),'LineWidth',1.5)
title('Heave controller signal')
line([0 time_actuator_controls(end)],[0 0],'LineWidth',1,'Color','k')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\textbf{Signal [-]}','interpreter','latex')
xlim([0, time_actuator_controls(end)])
ylim([-1, 1])
grid minor
%% Vehicle Acceleration
%{
local_acceleration = table2array(vehicle_local_position(:,16:18));

figure
subplot(3,1,1)
plot(time_vehicle_local_position,local_acceleration(:,1),'LineWidth',1.5)
hold on
line([0 time_vehicle_local_position(end)],[0 0],'LineWidth',1,'Color','k')
title('ax')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$dudt$} \boldmath{$[m^2/s]$}','interpreter','latex')
xlim([0, time_vehicle_local_position(end)])
grid minor
subplot(3,1,2)
plot(time_vehicle_local_position,local_acceleration(:,2),'LineWidth',1.5)
hold on
line([0 time_vehicle_local_position(end)],[0 0],'LineWidth',1,'Color','k')
title('ay')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$dvdt$} \boldmath{$[m^2/s]$}','interpreter','latex')
xlim([0, time_vehicle_local_position(end)])
grid minor
subplot(3,1,3)
plot(time_vehicle_local_position,local_acceleration(:,3),'LineWidth',1.5)
hold on
line([0 time_vehicle_local_position(end)],[0 0],'LineWidth',1,'Color','k')
title('az')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$dwdt$} \boldmath{$[m^2/s]$}','interpreter','latex')
xlim([0, time_vehicle_local_position(end)])
grid minor
%}

%% Optical Flow Camera
%{
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
%}


%% Save data
% save('log_8_2022-4-29-17-09-36.ulg.mat','distance_sensor','time_distance_sensor')
