function y = lsim_uss(sys,u,t,samples)

sys_un = usample(sys,samples);
% y = zeros(length(t),size(sys_un(:,:,1,1),1),samples);
y = [];
for i=1:samples
    y(:,:,i) = lsim(sys_un(:,:,i,1),u,t);
end

end