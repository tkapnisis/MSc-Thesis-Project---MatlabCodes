clc
clear all
close all

load log_8_2022-4-29-17-09-36.ulg.mat

%%
fs = 20;
fpass = 1;
s = tf('s');
womega_c = 2*pi*fpass;
lpf = zpk(womega_c/(womega_c+s));
dlpf_tustin = tf(c2d(lpf,1/fs,'tustin'));
dfilt_coeff.b0 = dlpf_tustin.Numerator{1,1}(1);
dfilt_coeff.b1 = dlpf_tustin.Numerator{1,1}(2);
dfilt_coeff.a1 = dlpf_tustin.Denominator{1,1}(2);
dlpf_zoh = c2d(lpf,1/fs);
alpha = dlpf_zoh.K;
%% Zero-order hold
dist_bottom_filt_zoh = [];
dist_bottom_filt_zoh(1) = 0;
for i=2:length(distance_sensor)
    dist_bottom_filt_zoh(i) = alpha*distance_sensor(i) + (1-alpha)*dist_bottom_filt_zoh(i-1);
end   
%% Tustin-bilnear
dist_bottom_filt = [];
dist_bottom_filt(1) = 0;
for i=2:length(distance_sensor)
    dist_bottom_filt(i) = dfilt_coeff.b0*distance_sensor(i) +...
                            dfilt_coeff.b1*distance_sensor(i-1) - ...
                            dfilt_coeff.a1*dist_bottom_filt(i-1);
end  

%% Moving average
dist_bottom_filt_ma = [];
dist_bottom_filt_ma(1) = 0;
L=5;
for i=L:length(distance_sensor)
    sum = distance_sensor(i);
    for j=1:L-1
        sum = sum + distance_sensor(i-j);
    end
    dist_bottom_filt_ma(i) = sum/L;
end  

%% Second-order filter design 
theta_c = 2*pi*fpass/fs;
Q = 0.5;
K = tan(theta_c/2);
W = K^2;
a = 1+K/Q+W;
a0 = 1;
a1 = 2*(W-1)/a;
a2 = (1-K/Q+W)/a;
b0 = W/a;
b1 = 2*W/a;
b2 = b0;

dist_bottom_filt_bq = [];
dist_bottom_filt_bq(1:2) = [0,0];
for i=3:length(distance_sensor)
    dist_bottom_filt_bq(i) = b0*distance_sensor(i) +...
                             b1*distance_sensor(i-1) + ...
                             b2*distance_sensor(i-2) - ...
                             a1*dist_bottom_filt_bq(i-1) - ...
                             a2*dist_bottom_filt_bq(i-2);
end  

%%
% figure
[y,d]=lowpass(distance_sensor,fpass,fs);
% lowpass(distance_sensor,fpass,fs);
% lowpass(distance_sensor,fpass,fs);
% figure
% lowpass(d_dist,fpass,fs);
% distance_sensor_f = lowpass(distance_sensor,fpass,fs);
%%
% dist_bottom_rate = (y(2:end) - y(1:end-1))/(1/20);
% figure
% % plot(time_distance_sensor(1:end-1),dist_bottom_rate)
% hold on
% plot(time_distance_sensor,y)
% hold on
% plot(time_distance_sensor,distance_sensor)
%%
figure
hold on
plot(time_distance_sensor,distance_sensor,'LineWidth',1)
plot(time_distance_sensor,y,'LineWidth',1)
plot(time_distance_sensor,dist_bottom_filt_zoh,'LineWidth',1)
% plot(time_distance_sensor,dist_bottom_filt,'LineWidth',1)
plot(time_distance_sensor,dist_bottom_filt_bq,'LineWidth',1)
line([0 time_distance_sensor(end)],[0 0],'LineWidth',1,'Color','k')
title('Distance from free-surface of water')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$h$} \textbf{[m]}','interpreter','latex')
xlim([0, time_distance_sensor(end)])    
grid minor
legend('Raw','Discrete filter - FIR','Discrete filter - ZOH','Discrete filter - BQ')

%%
diff1 = y - distance_sensor;
diff2 = dist_bottom_filt' - distance_sensor;
figure
hold on
plot(time_distance_sensor,diff2)
plot(time_distance_sensor,diff1)
legend('Tustin','IIR');
