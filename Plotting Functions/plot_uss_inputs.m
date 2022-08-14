function fig = plot_uss_inputs(t,u_in,u_eq,samples,LineWidth,LineStyle,Color)

subplot(3,1,1)
hold on
for i=1:samples
    fig = plot(t,rad2deg(u_in(:,1,i) + u_eq(1)),'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',Color);
end
title('Fore Servo Motor','FontSize',12)
xlabel('\textbf{time (s)}','interpreter','latex','FontSize',12)
ylabel('\boldmath{$\delta_s^f$} \textbf{(deg)}','interpreter','latex','FontSize',12)
grid minor

subplot(3,1,2)
hold on
for i=1:samples
    plot(t,rad2deg(u_in(:,2,i) + u_eq(2)),'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',Color);
end
title('Aft Port Servo Motor','FontSize',12)
xlabel('\textbf{time (s)}','interpreter','latex','FontSize',12)
ylabel('\boldmath{$\delta_s^{ap}$} \textbf{(deg)}','interpreter','latex','FontSize',12)
grid minor

subplot(3,1,3)
hold on
for i=1:samples
    plot(t,rad2deg(u_in(:,2,i) + u_eq(3)),'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',Color);
end
title('Aft Starboard Servo Motor','FontSize',12)
xlabel('\textbf{time (s)}','interpreter','latex','FontSize',12)
ylabel('\boldmath{$\delta_s^{as}$} \textbf{(deg)}','interpreter','latex','FontSize',12)
grid minor

end