%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2024 Learning Algorithms and Systems Laboratory, EPFL,
%    Switzerland
%   Author: Aude Billard
%   email: aude.billard@epfl.ch
%   website: lasa.epfl.ch
%    
%   Permission is granted to copy, distribute, and/or modify this program
%   under the terms of the GNU General Public License, version 3 or any
%   later version published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful, but
%   WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%   Public License for more details
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Section 1: Create world and define obstacles
clc; clear; close all;
folder = fileparts(which('practical2_main.m'));
addpath(genpath(fullfile(folder, 'library')));

initialPosition = [-0.3; 0.4; 0.7];
myWorld = Environment(initialPosition);

%% Section 2: Simulate initial conditions

% Initial state: joint velocity and position
q = myWorld.robot.computeInverseKinematics(initialPosition, myWorld.robot.robot.homeConfiguration, ...
                false, diag([1, -1, -1]));
q_dot = zeros(length(q), 1);

% Create robot and controller structure
usingMPC_ds = false;
if usingMPC_ds
    load("ds_control.mat")
    dsModulated = @(x) modulatedDS(x, attractor, myWorld, ds_control);
else
    attractor = [0.3; 0; 0.3];
    dsModulated = @(x) modulatedDS(x, attractor, myWorld);
end

% Display initial robot pose
ax = show(myWorld.robot.robot, q, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on', 'FastUpdate', 1);

% Plot target point and DS open-loop path
hold on;
plot3(attractor(1), attractor(2), attractor(3), 'go', 'LineWidth', 3, 'MarkerSize', 20);

opt_sim = []; 
opt_sim.plot  = 0;
opt_sim.dt = 3e-3;
opt_sim.i_max = 1500;
echo off;
[x_sim, ~] = Simulation(initialPosition , [], dsModulated, opt_sim);
plot3(x_sim(1,:), x_sim(2,:), x_sim(3,:), 'r', 'LineWidth', 2);

% Plot initial velocity and position
desired_vel = dsModulated(initialPosition);
plot3(initialPosition(1), initialPosition(2), initialPosition(3), '.k', 'MarkerSize', 10)
quiver3(initialPosition(1), initialPosition(2), initialPosition(3), ...
        desired_vel(1), desired_vel(2), desired_vel(3), 0.1, 'b'); 


% Plot DS open-loop path starting at random points
axisLimit = 0.9*myWorld.axisLimit;
for iTraj = 1:10
    randomPosition = rand(3, 1);
    randomPosition(1) = randomPosition(1)*(axisLimit(2)-axisLimit(1)) + axisLimit(1);
    randomPosition(2) = randomPosition(2)*(axisLimit(4)-axisLimit(3)) + axisLimit(3);
    randomPosition(3) = randomPosition(3)*(axisLimit(6)-axisLimit(5)) + axisLimit(5);
    scatter3(randomPosition(1), randomPosition(2), randomPosition(3))
    [~, x_sim, ~] = evalc('Simulation(randomPosition , [], dsModulated, opt_sim)');

    plot3(x_sim(1,:), x_sim(2,:), x_sim(3,:), 'Color', [1,0.6,0.2,0.6]);
end

drawnow;

legend('','','','','','','','','', ...
        'Target','Open-loop trajectory', 'Current position', 'Desired velocity', 'DS trajectories', 'AutoUpdate','off')

%% Section 3: Simulate closed-loop trajectory

% Setup controller and disturbance
ctrl = LimitedJointVelocityController(500, myWorld.robot.robot);
disturbance = CartesianForceDisturbance(5e3, 2);

% Simulation setup and parameters
text(axisLimit(2), axisLimit(4), 0.5 * axisLimit(6),{'\bf Perturbations: ', '\rm A/D keys : X axis ', 'Q/E keys : Y axis ', 'W/S keys : Z axis ', ...
                                                    '', '\bfStop simulation : \rm Space bar'}, 'FontSize', 12);

set(myWorld.figure,'WindowKeyPressFcn', {@KeyboardCb, disturbance});

delta_t = 25e-3;
start = tic;
i = 0;
position = myWorld.robot.computeForwardKinematics(q);
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
    [t, y] = ode15s(@(t,y) forwardStateDynamic(t, y, myWorld.robot.robot, dsModulated, disturbance.values, ctrl), ...
        [i * delta_t, (i+1) * delta_t], [q; q_dot]);
    
    q = y(end,1:7)';
    q_dot = y(end,8:14)';
    
    show(myWorld.robot.robot, q, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on', 'FastUpdate', 1);    
    plot3(position(1), position(2), position(3), '.k', 'MarkerSize', 10)
    drawnow;
    
    position = myWorld.robot.computeForwardKinematics(q);
    myWorld.endEffectorPosition = position;
    
    % Plot desired velocity from DS
    if mod(i, 4)==0
        desired_vel = dsModulated(position);
        quiver3(position(1), position(2), position(3), desired_vel(1), desired_vel(2), desired_vel(3), 0.2, 'Color', [0.2,0,1,0.9]);   
    end

    % Move obstacles if desired
    for k =1:length(myWorld.listOfObstacles)
        myWorld.listOfObstacles(k).moveShape(i*delta_t)
    end

    i = i + 1;
end

myWorld.robot.robot.getTransform(q, 'panda_link8')
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

function xdot = modulatedDS(x, attractor, world, nominalDS)

    if nargin < 4
        xdot_nominal = -4*eye(3)*(x-attractor);
    else
        xdot_nominal = nominalDS(x);
    end

    % Initialize modulation matrix and average obstacle velocity
    M_tot = eye(3);
    meanObstacleVelocity = zeros(3, 1);

    %% ------ Write your code below ------ %%
    %  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %
    
    % Create the array of gamma values from the list world.listOfObstacles
    % for convenience
    kObst = length(world.listOfObstacles);
    gamma = inf(1, kObst);
    for k = 1:kObst
        gamma(k) % = ;
    end
  
    % Go over each obstacle, compute the modulation matrix and apply it to
    % the initial ds velocity xdot_modulated
    for k = 1:kObst   

        % Task 2: Compute weight function 
%         if kObst > 1
%             weight  = ;
%         else
%             weight = 1;
%         end

        % Compute modulation matrix 
        D % = ;
    
        % Compute normal and tangent on ellipse
        normal % = ;
        tangent1 % = ;
        tangent2 % = ;
    
        E % = ;

        M % = ;
    end    

    xdot = M_tot * xdot_nominal;
end