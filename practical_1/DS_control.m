% Initialization
close all; clear; clc
filepath = fileparts(which('DS_control.m'));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-thirdparty')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-ds-opt')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-phys-gmm')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-robot-simulation')));
load('ds_control.mat');


% Create robot and controller structure
pandaRobot = PandaWrapper();
ctrl = LimitedJointVelocityController(500, pandaRobot.robot);
disturbance = CartesianForceDisturbance(5e3, 2);

% Initial state: joint velocity and position
initial_position = [-0.2; -0.4; 0.4];
q = pandaRobot.computeInverseKinematics(initial_position, pandaRobot.robot.randomConfiguration, ...
                false, diag([1, -1, -1]));
q_dot = zeros(length(q), 1);

% Display robot
f = figure('Name', 'Simulation', 'WindowKeyPressFcn', {@KeyboardCb, disturbance});
screensize = get(groot, 'Screensize');
f.Position = [0.5  * screensize(3), 0.1  * screensize(4), 0.7 * screensize(3), 0.8 * screensize(4)];

ax = show(pandaRobot.robot, q, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on', 'FastUpdate', 1);
view([45, 45])
axisLimit = [-0.5, 0.7, -0.6, 0.6, -0.1, 1];
axis(axisLimit);

% Plot target point and DS open-loop path
hold on;
plot3(attractor(1), attractor(2), attractor(3), 'go', 'LineWidth', 3, 'MarkerSize', 20);

opt_sim = []; 
opt_sim.plot  = 0;
[x_sim, ~] = Simulation(initial_position , [], ds_control, opt_sim);
plot3(x_sim(1,:), x_sim(2,:), x_sim(3,:), 'r', 'LineWidth', 2);

% Plot initial velocity and position
desired_vel = ds_control(initial_position);
plot3(initial_position(1), initial_position(2), initial_position(3), '.k', 'MarkerSize', 10)
quiver3(initial_position(1), initial_position(2), initial_position(3), ...
        desired_vel(1), desired_vel(2), desired_vel(3), 0.1, 'b'); 

% Plot DS open-loop path starting at random points
randomPosition = rand(3, 20);
randomPosition(1, :) = randomPosition(1, :)*(axisLimit(2)-axisLimit(1)) + axisLimit(1);
randomPosition(2, :) = randomPosition(2, :)*(axisLimit(4)-axisLimit(3)) + axisLimit(3);
randomPosition(3, :) = randomPosition(3, :)*(axisLimit(6)-axisLimit(5)) + axisLimit(5);
[x_sim, ~] = Simulation(randomPosition , [], ds_control, opt_sim);
for iTraj = 1:size(randomPosition, 2)
    plot3(x_sim(1,:,iTraj), x_sim(2,:,iTraj), x_sim(3,:,iTraj), 'Color', [1,0.6,0.2,0.6]);
end

drawnow;

legend('','','','','','','','','', ...
        'Target','Open-loop trajectory', 'Current position', 'Desired velocity', 'DS trajectories', 'AutoUpdate','off')

text(axisLimit(2), axisLimit(4), 0.5 * axisLimit(6),{'\bf Perturbations: ', '\rm A/D keys : X axis ', 'Q/E keys : Y axis ', 'W/S keys : Z axis ', ...
                                                    '', '\bfStop simulation : \rm Space bar'}, 'FontSize', 12);
% Simulate closed-loop trajectory
delta_t = 25e-3;
start = tic;
i = 0;
position = pandaRobot.computeForwardKinematics(q);
global stopFlag;
stopFlag = false;
while vecnorm(position - attractor) > 0.01 && ~stopFlag
    
    % Display disturbance
    if norm(disturbance.values) > 0 && disturbance.iteration < disturbance.nb_active_iterations
        if disturbance.iteration == 0
            if exist('hline', 'var') && exist('hhead', 'var')
                delete(hline);
                delete(hhead);
            end
            [hline, hhead] = arrow3d(position', position' + 0.5 * disturbance.values'/norm(disturbance.values), ...
                15, 'cylinder', [0.2, 0.2], [20, 10], [1, 0 ,0; 1, 0, 0]);
            drawnow;
        end
        disturbance.iteration = disturbance.iteration + 1;
    else
        disturbance.values = zeros(3,1);
        disturbance.iteration = inf;
    end

    % Integrate forward given current state and control
    [t, y] = ode15s(@(t,y) forwardStateDynamic(t, y, pandaRobot.robot, ds_control, disturbance.values, ctrl), ...
        [i * delta_t, (i+1) * delta_t], [q; q_dot]);
    
    q = y(end,1:7)';
    q_dot = y(end,8:14)';
    
    show(pandaRobot.robot, q, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on', 'FastUpdate', 1);    
    plot3(position(1), position(2), position(3), '.k', 'MarkerSize', 10)
    drawnow;
    
    position = pandaRobot.computeForwardKinematics(q);
    
    % Plot desired velocity from DS
    if mod(i, 4)==0
        desired_vel = ds_control(position);
        quiver3(position(1), position(2), position(3), desired_vel(1), desired_vel(2), desired_vel(3), 0.2, 'Color', [0.2,0,1,0.9]);   
    end

    i = i + 1;
end

pandaRobot.robot.getTransform(q, 'panda_link8')
time_elapsed = toc(start);
disp("Average computation time: " + time_elapsed / i + " [s] | Total time: " + time_elapsed + " [s]")

%% Simulation function
function dydt = forwardStateDynamic(t, y, robot, ds_control, force, ctrl)

    % Current state
    q = y(1:7);
    q_dot = y(8:14);

    % Desired state
    current_position = tform2trvec(robot.getTransform(q, 'panda_link8'))';
    desired_velocity = ds_control(current_position);   

    % Compute corresponding control torque (PD controller)
    full_jac = robot.geometricJacobian(q, 'panda_link8');
    controlAcceleration = ctrl.computeCommand(desired_velocity, q_dot, full_jac);

    % Compute disturbance
    torque_disturbance = full_jac(4:end,:)' * force;
    
    % Compute state derivative from current state and control
    dydt = [q_dot; ...
            controlAcceleration + torque_disturbance];

end


function KeyboardCb(~, event, force_disturbance)
    global stopFlag;
    if strcmp(event.Key, 'space') % space bar
        stopFlag = true;
        disp("Stopped Simulation")
    end
    force_disturbance.handleKeyEvent(event.Key);
    
end