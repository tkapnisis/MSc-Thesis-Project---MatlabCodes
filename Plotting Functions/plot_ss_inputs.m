function plot_ss_inputs(t,u_in,u_eq)

plot(t,rad2deg(u_in + u_eq),'LineWidth', 1.5)
grid minor
ylabel('\boldmath{$\theta_{s,i}$} \textbf{[deg]}','interpreter','latex','FontSize',12)
xlabel('\textbf{time [s]}','interpreter','latex','FontSize',12)
legend('Fore', 'Aft port', 'Aft starboard','FontSize',9)
title('Servo motor angles - Control inputs')
end