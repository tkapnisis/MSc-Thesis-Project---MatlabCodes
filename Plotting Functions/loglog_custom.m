function [fig1,fig2,fig3] = loglog_custom(sv1,wout1,sv2,wout2,sv3,wout3,num)

% Select num 2 for plotting 2 sets of singular values and 3 for 3 sets
figure

if num==2  
    loglog(wout1,sv1,'Color','red','LineStyle','-','LineWidth',1.5)
    hold on
    fig1 = loglog(wout1(1,1),sv1(1,1),'Color','red','LineStyle','-','LineWidth',1.5);
    loglog(wout2,sv2,'Color','blue','LineStyle','--','LineWidth',1)
    fig2 = loglog(wout2(1,1),sv2(1,1),'Color','blue','LineStyle','--','LineWidth',1);
    fig3 = [];
    grid on
    xlim([min(wout1),max(wout1)])
    xlabel('Frequency (rad/s)')
    ylabel('Singular Values (abs)')
    title('Singular Values')
    
elseif num==3
    loglog(wout3,sv3,'Color','#0072BD','LineStyle','-','LineWidth',1)
    hold on
    fig3 = loglog(wout3(1,1),sv3(1,1),'Color','#0072BD','LineStyle','-','LineWidth',1);
    loglog(wout1,sv1,'Color','#D95319','LineStyle','-.','LineWidth',1.5)
    fig1 = loglog(wout1(1,1),sv1(1,1),'Color','#D95319','LineStyle','-.','LineWidth',1.5);
    loglog(wout2,sv2,'Color','#EDB120','LineStyle','--','LineWidth',1.5)
    fig2 = loglog(wout2(1,1),sv2(1,1),'Color','#EDB120','LineStyle','--','LineWidth',1.5);
    grid on
    xlim([min(wout1),max(wout1)])
    xlabel('Frequency (rad/s)')
    ylabel('Singular Values (abs)')
    title('Singular Values')
end