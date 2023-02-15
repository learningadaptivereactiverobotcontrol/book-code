%% Initialization
clear; close all; clc;
filepath = fileparts(which('generate_MPC_trajectories.m'));
addpath(genpath(fullfile(filepath, '..', '..', 'book-robot-simulation')));
cd(filepath);
datasetPath = fullfile(filepath, '..', '..', '..', 'lecture6', 'dataset', 'MPC_test_dataset.mat');

pandaRobot = PandaWrapper();
optimal_control = MPC7DOF(pandaRobot);
optimal_control.nlSolver.Optimization.CustomCostFcn = @minimumTime;

target = [0.5; 0; 0.3; 0; 0; 0];

%% Solve optimal trajectory from random configuration
nTraj = 1;
for i = 1:nTraj

    % Find random configuration with specific orientation
    q0 = pandaRobot.robot.randomConfiguration;
    initPosition = pandaRobot.computeForwardKinematics(q0);
    q0 = pandaRobot.computeInverseKinematics(initPosition, q0, false, [1, 0, 0; 0, -1, 0; 0, 0, -1]);

    % Solve problem
    [xGuess, optimalSolution] = optimal_control.solveOptimalTrajectory(target, q0, 5, true, true);

    % Save solution
    load(datasetPath)

    [optJointPosition, optJointSpeed, optJointTorque] = ...
        optimal_control.getSolution(linspace(0, optimalSolution.Topt(end), 200));
    trajectories = cat(3, trajectories, ...
        [pandaRobot.computeForwardKinematics(optJointPosition); ...
        pandaRobot.computeForwardVelocity(optJointPosition, optJointSpeed)]);
    
    save(datasetPath, 'trajectories');

    disp(size(trajectories, 3));

end

%% Review generated trajectories
for i = 1:size(trajectories, 3)
    disp("Showing trajectory number "+ int2str(i));

    optimal_control.showTaskVolume(trajectories(:,:,i));
    
    disp("Press space to continue...");
    pause();
    close all;
end
%% Cost function %%

% Minimize time to reach target and final control action
% Also minimize distance to target orientation
function cost = minimumTime(X, U, e, data, robot, target)

    cost = sum(U(1:end-1,end)-1) + sum(U(end-1,:).^2);% ...
           %+ 50*sum((geometricJacobian(robot.robot, X(end, 1:7), 'panda_link8')*X(end, 8:14)' - [0; 0; 0; target(4:6)']).^2);
    for i = 1:data.PredictionHorizon
        T = robot.robot.getTransform(X(i,1:7)', 'panda_link8', 'panda_link0');
        cost = cost + 5*sum(sum((T(1:3,1:3) - [1, 0, 0; 0, -1, 0; 0, 0, -1]).^2));
    end
end
