function [Gnom_frd,Gp_frd,Gp_samples,W_I,W_I_frd] = Unc_Approx(omega,order,samples,G,Gp)

% Approximation of the uncertainty
Gnom_frd = frd(G,omega);
Gp_samples = usample(Gp,samples);
Gp_frd = frd(Gp_samples,omega);

W_I = ss([]);

for i=1:3
    for j=1:3
        [~,Info] = ucover(Gp_frd(i,j),Gnom_frd(i,j),order,'InputMult');
        W_I(i,j) = Info.W1;
        j
    end
    i
end  
W_I_frd = frd(W_I,omega);

end