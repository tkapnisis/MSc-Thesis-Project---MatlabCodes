function [u_x,u_z] = waves_sim_2D(x,t,k,L,A,omega,xlim,ylim,d,x_bar,z_bar)
    
    k_eq = (2*pi/L);
    % Calculate wave height along x at time t: 
    z = A*sin(omega*t(k) - k_eq*x); 
%     z = A*sin(k_eq*x-omega*t(k)); 
    for i=1:length(x_bar)
        x_w(i) = x_bar(i) - A*cosh(k_eq*(d+z_bar(i)))/sinh(k_eq*d)*...
                          cos(omega*t(k) - k_eq*x_bar(i));
        z_w(i) = z_bar(i) + A*sinh(k_eq*(d+z_bar(i)))/sinh(k_eq*d)*...
                          sin(omega*t(k) - k_eq*x_bar(i));
%         x_w(i) = x_bar(i) - A*exp(k_eq*z_bar(i))*cos(k_eq*x_bar(i)-omega*t(k));
%         z_w(i) = -z_bar(i) + A*exp(k_eq*z_bar(i))*sin(k_eq*x_bar(i)-omega*t(k));
%         major axis of elliptical trajectory of water particles
        a_traj = A*cosh(k_eq*(d + z_bar(i)))/sinh(k_eq*d);  
        % minor axis of elliptical trajectory of water particles
        b_traj = A*sinh(k_eq*(d + z_bar(i)))/sinh(k_eq*d);
%         water particles trajectory in x-axis
        th = linspace(0,2*pi);
        x_w_traj(i,:) = x_bar(i) + a_traj*cos(th);
        z_w_traj(i,:) = z_bar(i) + b_traj*sin(th);
        u_x(i) = omega*A*cosh(k_eq*(d + z_bar(i)))/sinh(k_eq*d)*...
                 cos(omega*t(k) - k_eq*x_bar(i));
        u_z(i) = omega*A*sinh(k_eq*(d+z_bar(i)))/sinh(k_eq*d)*...
                 sin(omega*t(k) - k_eq*x_bar(i));     
%         u_x(i) = -omega*A*exp(k_eq*z_bar(i))*sin(k_eq*x_bar(i)-omega*t(k));
%         u_z(i) = -omega*A*exp(k_eq*z_bar(i))*cos(k_eq*x_bar(i)-omega*t(k));   
    end

    % Plot wave: 
    fill([x,xlim(2),xlim(1)],[z,ylim(1),ylim(1)],[0.3010 0.7450 0.9330],...
          'FaceAlpha',0.2,'EdgeColor','b','LineWidth',2);
 
    hold on

    plot(x_w,z_w,'ko','markersize',8);
    for i=1:length(x_bar)
        plot(x_w_traj(i,:),z_w_traj(i,:),'b')
        plot(x_w_traj(i,:),z_w_traj(i,:),'b')
    end
    hold off
    
end