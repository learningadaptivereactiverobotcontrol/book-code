function [h_data, h_vel] = plot_reference_trajectories(Data, vel_sample, vel_size)

figure('Color',[1 1 1]);
M = size(Data,1)/2;
if M == 2    
    % Plot the position trajectories
    h_data = plot(Data(1,:),Data(2,:),'r.','markersize',10); hold on;
    
    % Plot Velocities of Reference Trajectories
    vel_points = Data(:,1:vel_sample:end);
    U = zeros(size(vel_points,2),1);
    V = zeros(size(vel_points,2),1);
    for i = 1:size(vel_points, 2)
        dir_    = vel_points(3:end,i)/norm(vel_points(3:end,i));
        U(i,1)   = dir_(1);
        V(i,1)   = dir_(2);
    end
    h_vel = quiver(vel_points(1,:)',vel_points(2,:)', U, V, vel_size, 'Color', 'k', 'LineWidth',2); hold on;
    
    grid on;
    box on;
    title('Reference Trajectories','Interpreter','LaTex','FontSize',20);
    xlabel('$\xi_1$','Interpreter','LaTex','FontSize',20);
    ylabel('$\xi_2$','Interpreter','LaTex','FontSize',20);
    
elseif M == 3

    % Plot the position trajectories
    h_data = plot3(Data(1,:),Data(2,:),Data(3,:),'r.','markersize',10); hold on;
    % Plot Velocities of Reference Trajectories
    vel_points = Data(:,vel_sample:vel_sample:end-vel_sample);
    U = zeros(size(vel_points,2),1);
    V = zeros(size(vel_points,2),1);
    W = zeros(size(vel_points,2),1);
    for i = 1:size(vel_points, 2)
        dir_    = vel_points(4:end,i)/norm(vel_points(4:end,i));
        U(i,1)   = dir_(1);
        V(i,1)   = dir_(2);
        W(i,1)   = dir_(3);
    end
    h_vel = quiver3(vel_points(1,:)',vel_points(2,:)',vel_points(3,:)', U, V, W, vel_size, 'Color', 'k', 'LineWidth',1.5); hold on;
    
    grid on;
    box on;
    title('Reference Trajectories','Interpreter','LaTex','FontSize',20);
    xlabel('$\xi_1$','Interpreter','LaTex','FontSize',20);
    ylabel('$\xi_2$','Interpreter','LaTex','FontSize',20);
    zlabel('$\xi_3$','Interpreter','LaTex','FontSize',20);
    axis equal;
else
    warning('Dimensionality not supported');
end

end