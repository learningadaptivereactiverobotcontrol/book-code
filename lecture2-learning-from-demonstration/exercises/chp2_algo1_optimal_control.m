%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Exercise Script for  Chapter 2 of:                                      %
% "Robots that can learn and adapt" by Billard, Mirrazavi and Figueroa.   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2020 Learning Algorithms and Systems Laboratory,          %
% EPFL, Switzerland                                                       %
% Author:  Alberic de Lajarte                                             %
% email:   alberic.lajarte@epfl.ch                                        %
% website: http://lasa.epfl.ch                                            %
%                                                                         %
% Permission is granted to copy, distribute, and/or modify this program   %
% under the terms of the GNU General Public License, version 2 or any     %
% later version published by the Free Software Foundation.                %
%                                                                         %
% This program is distributed in the hope that it will be useful, but     %
% WITHOUT ANY WARRANTY; without even the implied warranty of              %
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General%
% Public License for more details                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Create solver
clear; close all; clc;
filepath = fileparts(which('chp2_algo1_optimal_control.m'));
addpath(genpath(fullfile(filepath, '..', '..', 'libraries', 'book-robot-simulation')));

robot = RobotisWrapper();
optimalControl = MPC4DOF(robot);
optimalControl.nlSolver.Optimization.CustomCostFcn = % ... 

target_position = [0.25; 0; 0];
toleranceDistance = 10e-3;
maxTime = 5;
%% Generate batch of minimum time trajectories
nTraj = 10;
nPoints = optimalControl.nlSolver.PredictionHorizon + 1;
optimalTrajectories = nan(3, nPoints, nTraj);

h = waitbar(0,'Computing trajectories...');
for iTraj=1:nTraj
    
    % Find solution starting at random configuration
    q0 = robot.robot.randomConfiguration;
    % Last two parameters are used to speed up computation
    optimalSolution = optimalControl.solveOptimalTrajectory(target_position, q0, maxTime, true, true);
    
    % If the target is reached, append it to the dataset
    if norm(optimalSolution.Yopt(:, end) - target_position) < toleranceDistance
        optimalTrajectories(:,:,iTraj) = optimalSolution.Yopt;
    end
    
    % Visualize progression
    waitbar(iTraj/nTraj)
end
close(h)

% Display all successful trajectories
optimalControl.showTaskVolume(optimalTrajectories)

%% %%%%%%%%%%%%% User defined cost functions %%%%%%%%%%%%% %%



