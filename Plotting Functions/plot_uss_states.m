function fig = plot_uss_states(t,y,samples,eq_z,LineWidth,LineStyle,Color)

subplot(3,1,1)
hold on
for i=1:samples
    fig = plot(t,y(:,1,i) + eq_z,'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',Color);
end
ax = gca;
ax.FontSize = 10; 
title('Heave','FontSize',12)
xlabel('\textbf{time (s)}','interpreter','latex','FontSize',12)
ylabel('\boldmath{$z_n$} \textbf{(m)}','interpreter','latex','FontSize',12)
grid minor
subplot(3,1,2)
hold on
for i=1:samples
    plot(t,rad2deg(y(:,2,i)),'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',Color)
end
ax = gca;
ax.FontSize = 10; 
title('Roll','FontSize',12)
xlabel('\textbf{time (s)}','interpreter','latex','FontSize',12)
ylabel('\boldmath{$\phi$} \textbf{(deg)}','interpreter','latex','FontSize',12)
grid minor
subplot(3,1,3)
hold on
for i=1:samples
    plot(t,rad2deg(y(:,3,i)),'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',Color)
end
ax = gca;
ax.FontSize = 10; 
title('Pitch','FontSize',12)
xlabel('\textbf{time (s)}','interpreter','latex','FontSize',12)
ylabel('\boldmath{$\theta$} \textbf{(deg)}','interpreter','latex','FontSize',12)
grid minor

end