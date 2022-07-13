function plot_uss_inputs(t,u_in,u_eq,samples)

hold on
for i=1:samples
    plot(t,rad2deg(u_in(:,1,i) + u_eq(1)),'LineWidth', 0.5,'Color','#0072BD')
    plot(t,rad2deg(u_in(:,2,i) + u_eq(2)),'LineWidth', 0.5,'Color','#D95319')
    plot(t,rad2deg(u_in(:,3,i) + u_eq(3)),'LineWidth', 0.5,'Color','#EDB120')
end    
grid minor
ylabel('\boldmath{$\theta_{s,i}$} \textbf{[deg]}','interpreter','latex','FontSize',12)
xlabel('\textbf{time [s]}','interpreter','latex','FontSize',12)
legend('Fore', 'Aft port', 'Aft starboard','FontSize',9)


end