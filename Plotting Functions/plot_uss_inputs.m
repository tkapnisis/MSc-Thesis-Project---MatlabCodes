function plot_uss_inputs(t,u_in,u_eq,samples)

figure
hold on
for i=1:samples
    plot(t,rad2deg(u_in(:,1,i) + u_eq(1)),'LineWidth', 1,'Color','#0072BD')
    plot(t,rad2deg(u_in(:,2,i) + u_eq(2)),'LineWidth', 1,'Color','#D95319')
    plot(t,rad2deg(u_in(:,3,i) + u_eq(3)),'LineWidth', 1,'Color','#EDB120')
end    
title('Servo motor angles - Control inputs')
grid minor
ylabel('\boldmath{$\theta_s$} \textbf{[deg]}','interpreter','latex')
xlabel('\textbf{time [s]}','interpreter','latex')
legend('Fore hydrofoil', 'Aft port hydrofoil', 'Aft starboard hydrofoil')




end