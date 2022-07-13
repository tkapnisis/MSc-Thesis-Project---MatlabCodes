function fig = plot_ss_states(t,y,eq_z,LineWidth,LineStyle,color)

subplot(3,1,1)
hold on
fig = plot(t,y(:,1) + eq_z,'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',color);
title('Heave','FontSize',12)
xlabel('\textbf{time [s]}','interpreter','latex','FontSize',12)
ylabel('\boldmath{$z_n$} \textbf{[m]}','interpreter','latex','FontSize',12)
grid minor

subplot(3,1,2)
hold on
plot(t,rad2deg(y(:,2)),'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',color)
title('Roll','FontSize',12)
xlabel('\textbf{time [s]}','interpreter','latex','FontSize',12)
ylabel('\boldmath{$\phi$} \textbf{[deg]}','interpreter','latex','FontSize',12)
grid minor

subplot(3,1,3)
hold on
plot(t,rad2deg(y(:,3)),'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',color)
title('Pitch','FontSize',12)
xlabel('\textbf{time [s]}','interpreter','latex','FontSize',12)
ylabel('\boldmath{$\theta$} \textbf{[deg]}','interpreter','latex','FontSize',12)
grid minor

end