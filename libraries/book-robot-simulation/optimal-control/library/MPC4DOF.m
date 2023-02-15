%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper class for programming exercises in:                              %
% 'Learning for Adaptive and Reactive Robot Control' by Billard,          %
% Mirrazavi and Figueroa.                                                 %
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
classdef MPC4DOF
   properties
      nlSolver 
      robot
      
   end
   methods
    
    function self = MPC4DOF(robot)
        % Create non-linear solver for 4 DoF robotic arm

        % Create non-linear MPN object with four states x, input u and output y
        nu = 5; nx = 4; ny = 4;
        nlobj = nlmpc(nx, ny, nu);
        nlobj.Model.NumberOfParameters = 2;

        % Define sampling (Ts [s]), control and prediction (Tf) time [s]
        Ts = 0.1;
        nlobj.Model.IsContinuousTime = true;
        
        % The optimal control problem is solved with a normalized time: [0;1]
        % The real time is then obtained by multiplying it with the constant time
        % scaling parameter u(5) -> t: [0; u(5)]
        nlobj.Ts = Ts;
        nlobj.PredictionHorizon = round(1/Ts);
        nlobj.ControlHorizon = round(1/Ts);

        % Define system dynamics x_dot = f(x, u) = q_dot = u(1:4)
        % The time scaling effect is obtained by scaling the state dynamics 
        % by u(5) -> x_dot = u(5)*q_dot
        nlobj.Model.StateFcn = @(x, u, robot, target) u(5) * u(1:4);
        nlobj.Model.OutputFcn = @(x, u, robot, target) x(1:4);
        nlobj.Jacobian.StateFcn = @myStateJacobian;
        
        % Define custom equality constraints
        nlobj.Optimization.CustomEqConFcn = @myEqConFunction;

        % Store robot model 
        self.robot = robot;

        % Define state and input limits
        jointRange = 0.8;
        for i = 1:nx
            % Limit joint range to avoid singularities
            self.robot.robot.Bodies{1, 1+i}.Joint.PositionLimits(1) = ...
                jointRange*self.robot.robot.Bodies{1, 1+i}.Joint.PositionLimits(1); 
            self.robot.robot.Bodies{1, 1+i}.Joint.PositionLimits(2) = ...
                jointRange*self.robot.robot.Bodies{1, 1+i}.Joint.PositionLimits(2);
            
            nlobj.States(i).Min = self.robot.robot.Bodies{1, 1+i}.Joint.PositionLimits(1); 
            nlobj.States(i).Max = self.robot.robot.Bodies{1, 1+i}.Joint.PositionLimits(2); 
            
            % Limit joint speed and velocities
            nlobj.MV(i).Min = -5;
            nlobj.MV(i).Max = 5;
            nlobj.MV(i).RateMin = -0.25;
            nlobj.MV(i).RateMax = 0.25;

            % Soften rate of input change constraints
            nlobj.MV(i).RateMinECR = 1.0;
            nlobj.MV(i).RateMaxECR = 1.0;
        end
        % Constrain final time
        nlobj.MV(5).Min = 0.1;
        nlobj.MV(5).Max = 5;

        % Tune solver parameters for better performances
        nlobj.Optimization.SolverOptions.ConstraintTolerance = 1e-5;
        nlobj.Optimization.SolverOptions.MaxIterations = 100;
      
        % Store non-linear MPC
        self.nlSolver = nlobj;
    end
    
    function optimalSolution = solveOptimalTrajectory(self, cartesianTarget, initialPose, maxTime, addInitialGuess, avoidCheck)
        % Compute optimal trajectory to reach a cartesian target position
        
        % Compute initial and target joint configuration
        q0 = self.robot.robot.homeConfiguration;

        if nargin >= 3
            q0 = initialPose;            
        end   
        if nargin >= 4
            if maxTime > self.nlSolver.MV(5).Min
                self.nlSolver.MV(5).Max = maxTime;  
            else
                error('Requested time deadline is too short')
            end
        end
        x0 = q0(1:4);
        u0 = [0; 0; 0; 0; 1.5];
        
        if nargin == 6 && ~avoidCheck
            % Check problem formulation, then solve it
            validateFcns(self.nlSolver, x0, u0, [], {self.robot, cartesianTarget});
        end
        
        nloptions = nlmpcmoveopt;

        if nargin >= 5 && addInitialGuess
            xTarget = self.robot.computeInverseKinematics(cartesianTarget, initialPose, false); 
            nPoints = self.nlSolver.PredictionHorizon+1;
            
            xGuess = [linspace(q0(1), xTarget(1), nPoints)', ...
                      linspace(q0(2), xTarget(2), nPoints)', ...
                      linspace(q0(3), xTarget(3), nPoints)', ...
                      linspace(q0(4), xTarget(4), nPoints)'];
            nloptions.X0 = xGuess;
        end

        nloptions.Parameters = {self.robot, cartesianTarget};
        [~, ~, optimalSolution] = nlmpcmove(self.nlSolver, x0, u0, [], [], nloptions); 
        
        % Scale normalized time by time factor
        optimalSolution.MVopt = optimalSolution.MVopt';
        Tf = optimalSolution.MVopt(5, 1);
        optimalSolution.Topt = Tf .* optimalSolution.Topt';
        
        % Overwrite output variable (Y) to store cartesian trajectory
        optimalSolution.Xopt = optimalSolution.Xopt';
        optimalSolution.Yopt = self.robot.computeForwardKinematics(optimalSolution.Xopt);
            
        if optimalSolution.ExitFlag < 0
            warning('Solver failed. Try relaxing some of your constraints')
        else
            disp('OPTIMIZATION SUCCESSFUL')
        end
        
        disp("--------- Reached target in "+ Tf + " seconds ---------");
        
    end

    function showResults(self, optimalSolution, cartesianTarget, figTitle)
        % Show the results of the optimization
        
        cartesianTrajectory = optimalSolution.Yopt;
        
        % Animate the robot arm
        self.robot.animateTrajectory(optimalSolution.Xopt, cartesianTrajectory, cartesianTarget, figTitle);
        
        % Plot trajectories
        f = figure('Name', figTitle);
        screensize = get(groot, 'Screensize');
        f.Position = [0, 0.1  * screensize(4), screensize(3) / 4, 0.8 * screensize(4)];
        
        subplot(3,1,1);
        plot(optimalSolution.Topt, rad2deg(optimalSolution.Xopt)); 
        xlabel('Time [s]')
        ylabel('Joint position [degree]')

        subplot(3,1,2); 
        plot(optimalSolution.Topt, rad2deg(optimalSolution.MVopt(1:4,:))); 
        xlabel('Time [s]')
        ylabel('Joint velocity [degree/s]')
        
        subplot(3,1,3); 
        plot(optimalSolution.Topt, cartesianTrajectory); 
        xlabel('Time [s]')
        ylabel('Cartesian position [m]') 
    end

    function showTaskVolume(self, trajectories)
        
        nPoints = size(trajectories, 2);
        nTraj = size(trajectories, 3);
        
        trajX = reshape(trajectories(1,:,:), nPoints, nTraj);
        trajY = reshape(trajectories(2,:,:), nPoints, nTraj);
        trajZ = reshape(trajectories(3,:,:), nPoints, nTraj);
        
        show(self.robot.robot, [self.robot.computeInverseKinematics([trajX(end,1), trajY(end,1), trajZ(end,1)], [0; 0; 0; 0; 0; 0], false); 0; 0], ...
            'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
        
        hold on;
        % Plot trajectory and target in task space
        plot3(trajX, trajY, trajZ, 'k'); 
        plot3(trajX(1,:), trajY(1,:), trajZ(1,:), 'bo'); 
        plot3(trajX(end,:), trajY(end,:), trajZ(end,:), 'go'); 

        view([45, 45])
        axis([-0.3 0.35 -0.35 0.4 -0.2 0.45])
    end
    
   end
end

% Analyticaly computes the Jacobian of the state dynamics
function [A, Bmv] = myStateJacobian(X, U, robot, target)
    A = zeros(4);
    Bmv = cat(2, U(5)*eye(4), U(1:4));
end

function ceq = myEqConFunction(X, U, data, robot, target)
    % This function enforces the constraint to reach a specific target at the
    % end of the horizon
    ceq = [robot.fastForwardKinematic(X(end,1:4)) - target; diff(U(1:end-1,5))];
end
