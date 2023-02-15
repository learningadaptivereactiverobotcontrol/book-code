function[h_vel] = visualizeEstimatedVelocities(Data, ds_fun)

[M, N] = size(Data);
M = M/2;
h_vel = figure('Color',[1 1 1]);
xd_dot = [];

% Simulate velocities from same reference trajectory
for i=1:N
    xd_dot_ = ds_fun(Data(1:M,i));
    % Record Trajectories
    xd_dot = [xd_dot xd_dot_];
end
if isempty(xd_dot)
    xd_dot = ds_fun(Data(1:M,:));
end
% Plot Demonstrated Velocities vs Generated Velocities
if M == 2
    plot(Data(3,:)', '.-','Color',[0 0 1], 'LineWidth',1); hold on;
    plot(Data(4,:)', '.-','Color',[1 0 0], 'LineWidth',1); hold on;
    plot(xd_dot(1,:)','--','Color',[0 0 1], 'LineWidth', 1); hold on;
    plot(xd_dot(2,:)','--','Color',[1 0 0], 'LineWidth', 1); hold on;
    legend({'$\dot{x}^{ref}_{1}$','$\dot{x}^{ref}_{2}$','$\dot{x}^{d}_{1}$','$\dot{x}^{d}_{2}$'}, 'Interpreter', 'LaTex', 'FontSize', 15)    
    grid on;
    
elseif M == 3
    plot(Data(4,:)', '.-','Color',[0 0 1], 'LineWidth',1); hold on;
    plot(Data(5,:)', '.-','Color',[1 0 0], 'LineWidth',1); hold on;
    plot(Data(6,:)', '.-','Color',[0 1 0], 'LineWidth',1); hold on;
    plot(xd_dot(1,:)','--','Color',[0 0 1], 'LineWidth', 1); hold on;
    plot(xd_dot(2,:)','--','Color',[1 0 0], 'LineWidth', 1); hold on;
    plot(xd_dot(3,:)','--','Color',[0 1 0], 'LineWidth', 1); hold on;
    legend({'$\dot{x}^{ref}_{1}$','$\dot{x}^{ref}_{2}$','$\dot{x}^{ref}_{3}$',...,
        '$\dot{x}^{d}_{1}$','$\dot{x}^{d}_{2}$', '$\dot{x}^{d}_{3}$'}, 'Interpreter', 'LaTex', 'FontSize', 15)            
    grid on;
    
elseif M == 6 
    subplot(2,1,1)
    plot(Data(7,:)', '.-','Color',[0 0 1], 'LineWidth',1); hold on;
    plot(Data(8,:)', '.-','Color',[1 0 0], 'LineWidth',1); hold on;
    plot(Data(9,:)', '.-','Color',[0 1 0], 'LineWidth',1); hold on;
    plot(xd_dot(1,:)','--','Color',[0 0 1], 'LineWidth', 1); hold on;
    plot(xd_dot(2,:)','--','Color',[1 0 0], 'LineWidth', 1); hold on;
    plot(xd_dot(3,:)','--','Color',[0 1 0], 'LineWidth', 1); hold on;
    legend({'$\dot{x}^{ref}_{1}$','$\dot{x}^{ref}_{2}$','$\dot{x}^{ref}_{3}$',...,
        '$\dot{x}^{d}_{1}$','$\dot{x}^{d}_{2}$', '$\dot{x}^{d}_{3}$'}, 'Interpreter', 'LaTex', 'FontSize', 15)            

    grid on;
    
    
    subplot(2,1,2)
    plot(Data(10,:)', '.-','Color',[0 0 1], 'LineWidth',1); hold on;
    plot(Data(11,:)', '.-','Color',[1 0 0], 'LineWidth',1); hold on;
    plot(Data(12,:)', '.-','Color',[0 1 0], 'LineWidth',1); hold on;    
    plot(xd_dot(4,:)','--','Color',[0 0 1], 'LineWidth', 1); hold on;
    plot(xd_dot(5,:)','--','Color',[1 0 0], 'LineWidth', 1); hold on;
    plot(xd_dot(6,:)','--','Color',[0 1 0], 'LineWidth', 1); hold on;
    legend({'$\dot{x}^{ref}_{1}$','$\dot{x}^{ref}_{2}$','$\dot{x}^{ref}_{3}$',...,
        '$\dot{x}^{d}_{1}$','$\dot{x}^{d}_{2}$', '$\dot{x}^{d}_{3}$'}, 'Interpreter', 'LaTex', 'FontSize', 15)            
    grid on;
end

title('Real vs Estimated Velocities', 'Interpreter', 'LaTex', 'FontSize', 15)
end