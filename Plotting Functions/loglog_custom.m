function [fig1,fig2,fig3] = loglog_custom(mu_syn_sv,mu_syn_wout,...
                            hinf_sv,hinf_wout,weight_sv,weight_wout,type)

if type=="weight"
    figure
    loglog(weight_wout,weight_sv,'Color','#0072BD','LineStyle','-','LineWidth',1)
    hold on
    fig1 = loglog(weight_wout(1,1),weight_sv(1,1),'Color','#0072BD','LineStyle','-','LineWidth',1);
    loglog(mu_syn_wout,mu_syn_sv,'Color','#D95319','LineStyle','-.','LineWidth',1.5)
    fig2 = loglog(mu_syn_wout(1,1),mu_syn_sv(1,1),'Color','#D95319','LineStyle','-.','LineWidth',1.5);
    loglog(hinf_wout,hinf_sv,'Color','#EDB120','LineStyle','--','LineWidth',1.5)
    fig3 = loglog(hinf_wout(1,1),hinf_sv(1,1),'Color','#EDB120','LineStyle','--','LineWidth',1.5);
    grid on
    xlim([min(mu_syn_wout),max(mu_syn_wout)])
    xlabel('Frequency (rad/s)')
    ylabel('Singular Values (abs)')
    title('Singular Values')
elseif type=="contr"   
    fig1 = [];
    figure
    loglog(mu_syn_wout,mu_syn_sv,'Color','#0072BD','LineStyle','--','LineWidth',1)
    hold on
    fig2 = loglog(mu_syn_wout(1,1),mu_syn_sv(1,1),'Color','#0072BD','LineStyle','--','LineWidth',1);
    loglog(hinf_wout,hinf_sv,'Color','#D95319','LineStyle','-','LineWidth',1)
    fig3 = loglog(hinf_wout(1,1),hinf_sv(1,1),'Color','#D95319','LineStyle','-','LineWidth',1);
    grid on
    xlim([min(mu_syn_wout),max(mu_syn_wout)])
    xlabel('Frequency (rad/s)')
    ylabel('Singular Values (abs)')
    title('Singular Values')
end