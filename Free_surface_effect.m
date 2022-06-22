clc
clear all
close all


AR = [4,6,8];
% A = 4;
c = 0.1;
f = 0.05:0.01:0.35;
ratio = f/c;
att_fact = zeros(length(AR),length(f));
for i=1:length(ratio)
    for j=1:length(AR)
        A = AR(j);
        eq1 = A/(4*sqrt(1/4+4*ratio(i)^2+A^2/4));
        eq2 = 1/(1/4+4*ratio(i)^2) + 1/(A^2/4+4*ratio(i)^2);
        eq3 = 1/(8/A*ratio(i)^2+A/2);
        eq4 = 2/A*(sqrt(A^2+1)+1);
        att_fact(j,i) = 1/(1 + (eq1*eq2+eq3)/eq4);
    end
end

%%
figure
plot(ratio,att_fact)
legend('AR=4','AR=6','AR=8')
ylabel('CL/CLmax')
xlabel('h/c')
title('Attenuating factor of CL depending on depth')
hold on
ylim([0,1])
%%
att_fact_lin_coeff = polyfit(ratio,att_fact(1,:),1);
att_fact_lin_val = polyval(att_fact_lin_coeff,[min(ratio),max(ratio)]);

plot([min(ratio),max(ratio)],att_fact_lin_val)

%%
syms A c f

ratio = f/c;
eq1 = A/(4*sqrt(1/4+4*ratio^2+A^2/4));
eq2 = 1/(1/4+4*ratio^2) + 1/(A^2/4+4*ratio^2);
eq3 = 1/(8/A*ratio^2+A/2);
eq4 = 2/A*(sqrt(A^2+1)+1);
att_fact = 1/(1 + (eq1*eq2+eq3)/eq4);

%%
att_fact_lin = jacobian(att_fact,f);
c = 0.1;
A = 4;
f = 0.2;
att_fact_lin = double(subs(att_fact_lin))