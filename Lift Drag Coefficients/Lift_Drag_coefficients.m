clc
clear all
close all

CD.data_deg = csvread('Experimental_data_CD.csv',1,0);
CL.data_deg = csvread('Experimental_data_CL.csv',1,0);
%% For radians
CD.data_rad = [deg2rad(CD.data_deg(:,1)), CD.data_deg(:,2)];
CL.data_rad = [deg2rad(CL.data_deg(:,1)), CL.data_deg(:,2)];
CD.alpha_range = deg2rad([-5 10]);
CL.alpha_range = deg2rad([-10 10]);

CD.data_lin_region = CD.data_rad(find(CD.data_rad(:,1)>=CD.alpha_range(1) &...
                             CD.data_rad(:,1)<=CD.alpha_range(2)),:);
CL.data_lin_region = CL.data_rad(find(CL.data_rad(:,1)>=CL.alpha_range(1) &...
                             CL.data_rad(:,1)<=CL.alpha_range(2)),:);

CD.coeff = polyfit(CD.data_lin_region(:,1),CD.data_lin_region(:,2),1);
CL.coeff = polyfit(CL.data_lin_region(:,1),CL.data_lin_region(:,2),1);


CD.lin_val = polyval(CD.coeff,CD.alpha_range);
CL.lin_val = polyval(CL.coeff,CL.alpha_range);


figure
subplot(1,2,1)
hold on
plot(CL.data_rad(:,1),CL.data_rad(:,2),'ro-','LineWidth',1)
plot(CL.alpha_range,CL.lin_val,'b--','LineWidth',3)
line([0, 0], [min(CL.data_rad(:,2)), max(CL.data_rad(:,2))],'Color','k','LineWidth',0.8)
line([min(CL.data_rad(:,1)), max(CL.data_rad(:,1))], [0, 0],'Color','k','LineWidth',0.8)
ylabel('\boldmath{$C_L$}',Interpreter='latex',FontSize=13)
xlabel('\boldmath{$\alpha$} \textbf{[rad]}',Interpreter='latex',FontSize=13)
ylim([min(CL.data_rad(:,2)), max(CL.data_rad(:,2))])
title('Lift coefficient',FontSize=13)
legend('Experimental Data','Linear approximation','Location','best',FontSize=10)
grid minor

subplot(1,2,2)
hold on
plot(CD.data_rad(:,1),CD.data_rad(:,2),'ro-','LineWidth',1)
plot(CD.alpha_range,CD.lin_val,'b--','LineWidth',3)
line([0, 0], [0, max(CD.data_rad(:,2))],'Color','k','LineWidth',0.8)
line([min(CD.data_rad(:,1)), max(CD.data_rad(:,1))], [0, 0],'Color','k','LineWidth',0.8)
ylabel('\boldmath{$C_D$}',Interpreter='latex',FontSize=13)
xlabel('\boldmath{$\alpha$} \textbf{[rad]}',Interpreter='latex',FontSize=13)
title('Drag coefficient',FontSize=13)
legend('Experimental Data','Linear approximation','Location','best',FontSize=10)
grid minor

%% For degrees
CD.data_deg = [CD.data_deg(:,1), CD.data_deg(:,2)];
CL.data_deg = [CL.data_deg(:,1), CL.data_deg(:,2)];

CD.alpha_range = [-5 10];
CL.alpha_range = [-10 10];

CD.data_lin_region = CD.data_deg(find(CD.data_deg(:,1)>=CD.alpha_range(1) &...
                             CD.data_deg(:,1)<=CD.alpha_range(2)),:);
CL.data_lin_region = CL.data_deg(find(CL.data_deg(:,1)>=CL.alpha_range(1) &...
                             CL.data_deg(:,1)<=CL.alpha_range(2)),:);

CD.coeff = polyfit(CD.data_lin_region(:,1),CD.data_lin_region(:,2),1);
CL.coeff = polyfit(CL.data_lin_region(:,1),CL.data_lin_region(:,2),1);


CD.lin_val = polyval(CD.coeff,CD.alpha_range);
CL.lin_val = polyval(CL.coeff,CL.alpha_range);


figure
subplot(1,2,1)
hold on
plot(CL.data_deg(:,1),CL.data_deg(:,2),'ro-','LineWidth',1)
plot(CL.alpha_range,CL.lin_val,'b--','LineWidth',3)
line([0, 0], [min(CL.data_deg(:,2)), max(CL.data_deg(:,2))],'Color','k','LineWidth',0.8)
line([min(CL.data_deg(:,1)), max(CL.data_deg(:,1))], [0, 0],'Color','k','LineWidth',0.8)
ylabel('\boldmath{$C_L$}',Interpreter='latex',FontSize=13)
xlabel('\boldmath{$\alpha_e$} \textbf{[deg]}',Interpreter='latex',FontSize=13)
xlim([min(CL.data_deg(:,1)), max(CL.data_deg(:,1))])
ylim([min(CL.data_deg(:,2)), max(CL.data_deg(:,2))])
title('Lift coefficient',FontSize=13)
legend('Experimental Data','Linear approximation','Location','best',FontSize=10)
grid minor

subplot(1,2,2)
hold on
plot(CD.data_deg(:,1),CD.data_deg(:,2),'ro-','LineWidth',1)
plot(CD.alpha_range,CD.lin_val,'b--','LineWidth',3)
line([0, 0], [0, max(CD.data_deg(:,2))],'Color','k','LineWidth',0.8)
line([min(CD.data_deg(:,1)), max(CD.data_deg(:,1))], [0, 0],'Color','k','LineWidth',0.8)
xlim([min(CL.data_deg(:,1)), max(CL.data_deg(:,1))])
ylabel('\boldmath{$C_D$}',Interpreter='latex',FontSize=13)
xlabel('\boldmath{$\alpha_e$} \textbf{[deg]}',Interpreter='latex',FontSize=13)
title('Drag coefficient',FontSize=13)
legend('Experimental Data','Linear approximation','Location','best',FontSize=10)
grid minor

%% CD error calculation
error_CD = [];
CD.lin_approx = CD.coeff(1)*CD.data_deg(:,1) + CD.coeff(2);
for i=1:size(CD.data_deg(:,2),1)
    error_CD(i) = (CD.data_deg(i,2) - CD.lin_approx(i))/CD.data_deg(i,2)*100;
end
figure
hold on
line([-5, -5], [min(error_CD), max(error_CD)],'LineStyle','--','Color','k','LineWidth',0.8)
line([10, 10], [min(error_CD), max(error_CD)],'LineStyle','--','Color','k','LineWidth',0.8)
plot(CD.data_deg(:,1),error_CD,'*-','LineWidth',2)
ylabel('\boldmath{$Error=\frac{CD_{nl}-CD_{l}}{CD_{nl}}\cdot 100 [\%]$}',Interpreter='latex',FontSize=13)
xlabel('\boldmath{$\alpha_e$} \textbf{[deg]}',Interpreter='latex',FontSize=13)
% title('Drag coefficient error due to linear approximation',FontSize=13)
ylim([min(error_CD), max(error_CD)])
grid minor