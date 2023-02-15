%% Initialization
clear; close all; clc;
filepath = fileparts(which('MPC_tracking.m'));
addpath(genpath(fullfile(filepath, '..', '..', 'book-robot-simulation')));
cd(filepath);

pandaRobot = PandaWrapper();
optimal_control = MPC7DOF(pandaRobot);
optimal_control.nlSolver.Optimization.CustomCostFcn = @minimumTime;

target = [0.5; 0; 0.3; 0; 0; 0];

%% Solve optimal trajectory from random configuration
% Find random configuration with specific orientation
q0 = pandaRobot.robot.randomConfiguration;
%initPosition = pandaRobot.computeForwardKinematics(q0);
initPosition = [-0.4; 0.4; 0.2];
q0 = pandaRobot.computeInverseKinematics(initPosition, q0, false, diag([1, -1, -1]));

% Show initial configuration
f = figure('Name', 'Simulation'); 
f.Position(3:4) = [800, 800];
show(pandaRobot.robot, q0, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
hold on;
plot3(initPosition(1), initPosition(2), initPosition(3), 'ro')
view([45, 45]);
axis([-0.7, 0.9, -0.8, 0.8, -0.3, 1.2])
%% Solve problem

start = tic;
[xGuess, optimalSolution] = optimal_control.solveOptimalTrajectory(target, q0, 5, true, false);
toc(start);  

optimal_control.showResults(optimalSolution, target, 'Minimal Time');
%% Simulating tracking of MPC solution
[simulatedTime, simulatedJoint, simulatedSpeed] = pandaRobot.simulateRobot(optimal_control, target);

% Plot results
f = figure(2);
f.Position = [0, 0, 400 800];
subplot(2, 1, 1);
plot(optimalSolution.Topt, optimalSolution.Xopt(1:7,:), '-');
hold on;
plot(simulatedTime, simulatedJoint, ':');
title('Joint position [rad]');
subplot(2, 1, 2);
plot(optimalSolution.Topt, optimalSolution.Xopt(8:14,:), '-');
hold on;
plot(simulatedTime, simulatedSpeed, ':');
title('Joint speed [rad/s]');

f = figure(3);
f.Position = [1000, 500, 400, 800];
subplot(2, 1, 1);
plot(optimalSolution.Topt, 1000 * pandaRobot.computeForwardKinematics(optimalSolution.Xopt(1:7,:)), '-');
hold on;
plot(simulatedTime, 1000 * pandaRobot.computeForwardKinematics(simulatedJoint), ':');
title('Cartesian position [mm]');
subplot(2, 1, 2);
plot(optimalSolution.Topt, ...
    pandaRobot.computeForwardVelocity(optimalSolution.Xopt(1:7,:), optimalSolution.Xopt(8:14,:)), '-');
hold on;
plot(simulatedTime, pandaRobot.computeForwardVelocity(simulatedJoint, simulatedSpeed), ':');
title('Cartesian velocity [m/s]');

%% Cost function %%

% Task 1: Minimum time
% This function integrates the time scaling parameter u(5) to minimize
% trajectory time
function cost = minimumTime(X, U, e, data, robot, target)

    cost = sum(U(1:end-1,end)-1) + sum(U(end-1,:).^2);% ...
           %+ 50*sum((geometricJacobian(robot.robot, X(end, 1:7), 'panda_link8')*X(end, 8:14)' - [0; 0; 0; target(4:6)']).^2);
    for i = 1:data.PredictionHorizon
        T = robot.robot.getTransform(X(i,1:7)', 'panda_link8', 'panda_link0');
        cost = cost + 5*sum(sum((T(1:3, 1:3) - [1, 0, 0; 0, -1, 0; 0, 0, -1]).^2));
    end
end
