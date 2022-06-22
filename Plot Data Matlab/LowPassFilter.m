clc
clear all
close all

load log_8_2022-4-29-17-09-36.ulg.mat
rowTimes = seconds(time_distance_sensor);
TT = array2timetable(distance_sensor,'RowTimes',rowTimes);
tsout = setuniformtime(TT,'Interval',0.05);
%%
fs = 20;
[pxx,f] = pwelch(distance_sensor,fs);

figure

plot(f,10*log10(pxx))

xlabel('Frequency (Hz)')
ylabel('PSD (dB/Hz)')
%%
fs = 20;
fpass = 2;
figure
% y=lowpass(distance_sensor,fpass,fs);
lowpass(distance_sensor,fpass,fs,'ImpulseResponse','iir');
distance_sensor_f = lowpass(distance_sensor,fpass,fs);

%%
figure
hold on
plot(time_distance_sensor,y,'LineWidth',1.5)
plot(time_distance_sensor,distance_sensor,'LineWidth',1.5)
line([0 time_distance_sensor(end)],[0 0],'LineWidth',1,'Color','k')
title('Distance from free-surface of water')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$h$} \textbf{[m]}','interpreter','latex')
xlim([0, time_distance_sensor(end)])
ylim([-0.5, 0.5])
grid minor
legend('Filtered','Raw')