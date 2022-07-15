function fig = plot_ss_states(t,y,ref,eq_z,LineWidth,LineStyle,color,type)

if type=="ref"
    subplot(3,1,1)
    hold on
    fig = plot(t,y(:,1) + eq_z,'LineWidth',1.5);
    plot(t,ref(1,:) + eq_z,'r--','LineWidth',1.5)
    title('Heave','FontSize',12)
    xlabel('\textbf{time [s]}','interpreter','latex','FontSize',12)
    ylabel('\boldmath{$z_n$} \textbf{[m]}','interpreter','latex','FontSize',12)
    grid minor
    
    subplot(3,1,2)
    hold on
    plot(t,rad2deg(y(:,2)),'LineWidth',1.5)
    plot(t,ref(2,:),'r--','LineWidth',1.5)
    title('Roll','FontSize',12)
    xlabel('\textbf{time [s]}','interpreter','latex','FontSize',12)
    ylabel('\boldmath{$\phi$} \textbf{[deg]}','interpreter','latex','FontSize',12)
    grid minor
    
    subplot(3,1,3)
    hold on
    plot(t,rad2deg(y(:,3)),'LineWidth',1.5)
    plot(t,ref(3,:),'r--','LineWidth',1.5)
    title('Pitch','FontSize',12)
    xlabel('\textbf{time [s]}','interpreter','latex','FontSize',12)
    ylabel('\boldmath{$\theta$} \textbf{[deg]}','interpreter','latex','FontSize',12)
    legend('Response','Reference Signal','Location','best')
    grid minor

elseif type=="ref_no_leg"
    subplot(3,1,1)
    hold on
    fig = plot(t,y(:,1) + eq_z,'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',color);
    title('Heave','FontSize',12)
    xlabel('\textbf{time [s]}','interpreter','latex','FontSize',12)
    ylabel('\boldmath{$z_n$} \textbf{[m]}','interpreter','latex','FontSize',12)
    grid minor
    
    subplot(3,1,2)
    hold on
    plot(t,rad2deg(y(:,2)),'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',color);
    title('Roll','FontSize',12)
    xlabel('\textbf{time [s]}','interpreter','latex','FontSize',12)
    ylabel('\boldmath{$\phi$} \textbf{[deg]}','interpreter','latex','FontSize',12)
    grid minor
    
    subplot(3,1,3)
    hold on
    plot(t,rad2deg(y(:,3)),'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',color);
    title('Pitch','FontSize',12)
    xlabel('\textbf{time [s]}','interpreter','latex','FontSize',12)
    ylabel('\boldmath{$\theta$} \textbf{[deg]}','interpreter','latex','FontSize',12)
    grid minor

elseif type=="dist"
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

end