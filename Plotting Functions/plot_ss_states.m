function [fig1,fig2] = plot_ss_states(t,y,ref,eq_z,LineWidth,LineStyle,Color,type)

if type == "ref"
    subplot(3,1,1)
    hold on
    fig1 = plot(t,y(:,1) + eq_z,'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',Color);
    fig2 = plot(t,ref(1,:) + eq_z,'LineWidth',0.8,'LineStyle','--','Color','k');
    ax = gca;
    ax.FontSize = 10; 
    title('Heave','FontSize',12)
    xlabel('\textbf{time (s)}','interpreter','latex','FontSize',12)
    ylabel('\boldmath{$z_n$} \textbf{(m)}','interpreter','latex','FontSize',12)
    grid minor
    
    subplot(3,1,2)
    hold on
    plot(t,rad2deg(y(:,2)),'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',Color);
    plot(t,rad2deg(ref(2,:)),'LineWidth',0.8,'LineStyle','--','Color','k');
    ax = gca;
    ax.FontSize = 10; 
    title('Roll','FontSize',12)
    xlabel('\textbf{time (s)}','interpreter','latex','FontSize',12)
    ylabel('\boldmath{$\phi$} \textbf{(deg)}','interpreter','latex','FontSize',12)
    grid minor
    
    subplot(3,1,3)
    hold on
    plot(t,rad2deg(y(:,3)),'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',Color);
    plot(t,rad2deg(ref(3,:)),'LineWidth',0.8,'LineStyle','--','Color','k');
    ax = gca;
    ax.FontSize = 10; 
    title('Pitch','FontSize',12)
    xlabel('\textbf{time (s)}','interpreter','latex','FontSize',12)
    ylabel('\boldmath{$\theta$} \textbf{(deg)}','interpreter','latex','FontSize',12)
    grid minor

elseif type == "dist"
    subplot(3,1,1)
    hold on
    fig1 = plot(t,y(:,1) + eq_z,'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',Color);
    ax = gca;
    ax.FontSize = 10; 
    title('Heave','FontSize',12)
    xlabel('\textbf{time (s)}','interpreter','latex','FontSize',12)
    ylabel('\boldmath{$z_n$} \textbf{(m)}','interpreter','latex','FontSize',12)
    grid minor
    
    subplot(3,1,2)
    hold on
    plot(t,rad2deg(y(:,2)),'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',Color)
    ax = gca;
    ax.FontSize = 10; 
    title('Roll','FontSize',12)
    xlabel('\textbf{time (s)}','interpreter','latex','FontSize',12)
    ylabel('\boldmath{$\phi$} \textbf{(deg)}','interpreter','latex','FontSize',12)
    grid minor
    
    subplot(3,1,3)
    hold on
    plot(t,rad2deg(y(:,3)),'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',Color)
    ax = gca;
    ax.FontSize = 10; 
    title('Pitch','FontSize',12)
    xlabel('\textbf{time (s)}','interpreter','latex','FontSize',12)
    ylabel('\boldmath{$\theta$} \textbf{(deg)}','interpreter','latex','FontSize',12)
    grid minor

    fig2 = [];

elseif type == "dist_ref"
    subplot(3,1,1)
    hold on
    fig1 = plot(t,y(:,1) + eq_z,'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',Color);
    fig2 = plot(t,ref(1,:) + eq_z,'LineWidth',0.8,'LineStyle','--','Color','k');
    ax = gca;
    ax.FontSize = 10; 
    title('Heave','FontSize',12)
    xlabel('\textbf{time (s)}','interpreter','latex','FontSize',12)
    ylabel('\boldmath{$z_n$} \textbf{(m)}','interpreter','latex','FontSize',12)
    grid minor
    
    subplot(3,1,2)
    hold on
    plot(t,rad2deg(y(:,2)),'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',Color);
    plot(t,rad2deg(ref(2,:)),'LineWidth',0.8,'LineStyle','--','Color','k');
    ax = gca;
    ax.FontSize = 10; 
    title('Roll','FontSize',12)
    xlabel('\textbf{time (s)}','interpreter','latex','FontSize',12)
    ylabel('\boldmath{$\phi$} \textbf{(deg)}','interpreter','latex','FontSize',12)
    grid minor
    
    subplot(3,1,3)
    hold on
    plot(t,rad2deg(y(:,3)),'LineWidth',LineWidth,'LineStyle',LineStyle,'Color',Color);
    plot(t,rad2deg(ref(3,:)),'LineWidth',0.8,'LineStyle','--','Color','k');
    ax = gca;
    ax.FontSize = 10; 
    title('Pitch','FontSize',12)
    xlabel('\textbf{time (s)}','interpreter','latex','FontSize',12)
    ylabel('\boldmath{$\theta$} \textbf{(deg)}','interpreter','latex','FontSize',12)
    grid minor
end

end