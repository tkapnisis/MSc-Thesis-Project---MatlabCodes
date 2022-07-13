function plot_uss_states(t,y,samples,eq)

figure
subplot(3,1,1)
hold on
for i=1:samples
    plot(t,y(:,1,i) + eq(1),'LineWidth',0.5,'Color','b')
end
title('Heave')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$z_n$} \textbf{[m]}','interpreter','latex')
grid minor
subplot(3,1,2)
hold on
for i=1:samples
    plot(t,rad2deg(y(:,2,i) + eq(2)),'LineWidth',0.5,'Color','b')
end
title('Roll')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\phi$} \textbf{[deg]}','interpreter','latex')
grid minor
subplot(3,1,3)
hold on
for i=1:samples
    plot(t,rad2deg(y(:,3,i) + eq(3)),'LineWidth',0.5,'Color','b')
end
title('Pitch')
xlabel('\textbf{time [s]}','interpreter','latex')
ylabel('\boldmath{$\theta$} \textbf{[deg]}','interpreter','latex')
grid minor

end