function sv = sigma_uss(sys,omega,samples)

sys_un = usample(sys,samples);
sv = zeros(size(sys,1),length(omega),samples);
for i=1:samples
    [sv(:,:,i),~] = sigma(sys_un(:,:,i,1),omega);
end

end