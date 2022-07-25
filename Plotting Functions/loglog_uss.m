function [fig1,fig2,fig3] = loglog_uss(sv1,wout1,sv2,wout2,sv3,wout3,samples,num)

% Select num 2 for plotting 2 sets of singular values and 3 for 3 sets
figure

if num==1  
    for i=1:samples
        loglog(wout1,sv1(:,:,i),'Color','red','LineStyle','-.','LineWidth',0.5)
        hold on
        loglog(wout2,sv2(:,:,i),'Color','blue','LineStyle','-.','LineWidth',0.5)
    end    
    fig1 = loglog(wout1(1,1),sv1(1,1,1),'Color','red','LineStyle','-.','LineWidth',1);
    fig2 = loglog(wout2(1,1),sv2(1,1,1),'Color','blue','LineStyle','--','LineWidth',1);
    fig3 = [];
    grid on
    xlim([min(wout1),max(wout1)])
    xlabel('Frequency (rad/s)','FontSize',12)
    ylabel('Singular Values (abs)','FontSize',12)
    title('Singular Values','FontSize',12)
    
elseif num==2
    for i=1:samples
        loglog(wout1,sv1(:,:,i),'Color','#EDB120','LineStyle','--','LineWidth',0.75)
        hold on
        loglog(wout2,sv2(:,:,i),'Color','red','LineStyle','-.','LineWidth',0.75)
        loglog(wout3,sv3,'Color','blue','LineStyle','--','LineWidth',1)
    end  
    fig1 = loglog(wout1(1,1),sv1(1,1,1),'Color','#EDB120','LineStyle','--','LineWidth',0.75);
    fig2 = loglog(wout2(1,1),sv2(1,1,1),'Color','red','LineStyle','-.','LineWidth',0.75);
    fig3 = loglog(wout3(1,1),sv3(1,1),'Color','blue','LineStyle','--','LineWidth',1);
    grid on
    xlim([min(wout1),max(wout1)])
    xlabel('Frequency (rad/s)','FontSize',11)
    ylabel('Singular Values (abs)','FontSize',11)
    title('Singular Values','FontSize',12)
elseif num==3
    for i=1:samples
        loglog(wout1,sv1(:,:,i),'Color','red','LineStyle','--','LineWidth',0.5)
        hold on
        loglog(wout2,sv2(:,:,i),'Color','blue','LineStyle','-.','LineWidth',0.5)
        loglog(wout3,sv3,'Color','black','LineStyle','-','LineWidth',0.75)
    end  
    fig1 = loglog(wout1(1,1),sv1(1,1,1),'Color','red','LineStyle','-.','LineWidth',0.5);
    fig2 = loglog(wout2(1,1),sv2(1,1,1),'Color','blue','LineStyle','--','LineWidth',0.5);
    fig3 = loglog(wout3(1,1),sv3(1,1),'Color','black','LineStyle','-','LineWidth',0.75);
    grid on
    xlim([min(wout1),max(wout1)])
    xlabel('Frequency (rad/s)','FontSize',11)
    ylabel('Singular Values (abs)','FontSize',11)
    title('Singular Values','FontSize',12)    
end