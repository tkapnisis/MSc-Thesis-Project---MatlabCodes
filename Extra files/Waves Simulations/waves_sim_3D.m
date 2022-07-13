function waves_sim_3D(x,X,Y,t,k,L,A,omega,xlim,zlim)
    
    k_eq = (2*pi/L);
    % Calculate wave height along x at time t: 
    z = A*sin(omega*t(k) - k_eq*x); 

    Z = ones(length(x),1)*z;
    mesh(X,Y,Z,'EdgeColor','none','FaceColor','b','FaceAlpha','0.2')  
    hold on
    % Plot wave: 
    fill3([x,xlim(2),xlim(1)],[zeros(1,length(x)),0,0],[z,zlim(1),zlim(1)],[0.3010 0.7450 0.9330],...
          'FaceAlpha',0.2,'EdgeColor','b','LineWidth',2);

    hold off
    
end