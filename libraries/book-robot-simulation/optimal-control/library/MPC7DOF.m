%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper class for programming exercises in:                              %
% "Learning for Adaptive and Reactive Robot Control" by Billard,          %
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
classdef MPC7DOF < handle
   properties
      nlSolver 
      currentSolution
      robot

      perturbations
      currentTime
   end
   methods
    
    function self = MPC7DOF(robot)
        % Create non-linear solver for 4 DoF robotic arm

        % Create non-linear MPN object with four states x, input u and output y
        nu = 8; nx = 14; ny = 14;
        nlobj = nlmpc(nx, ny, nu);
        nlobj.Model.NumberOfParameters = 2;

        % Define sampling (Ts [s]), control and prediction (Tf) time [s]
        Ts = 0.08;
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
        nlobj.Model.StateFcn = @myStateFunction;
        
        % Define custom equality constraints
        nlobj.Optimization.CustomEqConFcn = @myEqConFunction;

        % Store robot model 
        self.robot = robot;

        % Define state and input limits
        jointRange = 0.9;
        for i = 1:nx/2
            % Limit joint range to avoid singularities
            self.robot.robot.Bodies{1, i}.Joint.PositionLimits(1) = ...
            jointRange*self.robot.robot.Bodies{1, i}.Joint.PositionLimits(1); 
            self.robot.robot.Bodies{1, i}.Joint.PositionLimits(2) = ...
            jointRange*self.robot.robot.Bodies{1, i}.Joint.PositionLimits(2);
            
            nlobj.States(i).Min = self.robot.robot.Bodies{1, i}.Joint.PositionLimits(1); 
            nlobj.States(i).Max = self.robot.robot.Bodies{1, i}.Joint.PositionLimits(2); 
            
            nlobj.States(i+nx/2).Min = -2.6; 
            nlobj.States(i+nx/2).Max = 2.6; 
            
            % Limit joint speed and velocities
            nlobj.MV(i).Min = -10;
            nlobj.MV(i).Max = 10;
            nlobj.MV(i).RateMin = -5;
            nlobj.MV(i).RateMax = 5;

            % Soften rate of input change constraints
            nlobj.MV(i).RateMinECR = 1.0;
            nlobj.MV(i).RateMaxECR = 1.0;
        end
        % Constrain final time
        nlobj.MV(8).Min = 0.01;
        nlobj.MV(8).Max = 5;

        % Tune solver parameters for better performances
        nlobj.Optimization.SolverOptions.ConstraintTolerance = 1e-7;
        nlobj.Optimization.SolverOptions.MaxIterations = 400;
      
        % Store non-linear MPC
        self.nlSolver = nlobj;
    end
    
    function [xGuess, optimalSolution] = solveOptimalTrajectory(self, cartesianTarget, initialPose, maxTime, addInitialGuess, avoidCheck)
        % Compute optimal trajectory to reach a cartesian target position
        
        % Compute initial and target joint configuration
        q0 = self.robot.robot.homeConfiguration;

        if nargin >= 3
            q0 = initialPose;            
        end   
        if nargin >= 4
            if maxTime > self.nlSolver.MV(8).Min
                self.nlSolver.MV(8).Max = maxTime;  
            else
                error('Requested time deadline is too short')
            end
        end
        x0 = [q0; zeros(7, 1)];
        u0 = [zeros(7, 1); 2.5];
        
        if nargin == 6 && ~avoidCheck
            % Check problem formulation, then solve it
            validateFcns(self.nlSolver, x0, u0, [], {self.robot, cartesianTarget});
        end
        
        nloptions = nlmpcmoveopt;

        if nargin >= 5 && addInitialGuess
            disp('Using linear initial guess');
            xTarget = self.robot.computeInverseKinematics(cartesianTarget(1:3), initialPose, false); 
            nPoints = self.nlSolver.PredictionHorizon+1;
            
            xGuess = zeros(nPoints, 14);  
            tolerance = 1e-1;
            for iJoint = 1:7
                xGuess(:, iJoint) = linspace(q0(iJoint), xTarget(iJoint), nPoints)';
                if xTarget(iJoint) > q0(iJoint) + tolerance
                    xGuess(:, iJoint+7) = ones(nPoints, 1)*self.nlSolver.States(iJoint+7).Max;
                elseif xTarget(iJoint) < q0(iJoint) - tolerance
                    xGuess(:, iJoint+7) = ones(nPoints, 1)*self.nlSolver.States(iJoint+7).Min;
                end
                
            end
            
            nloptions.X0 = xGuess;
        end

        nloptions.Parameters = {self.robot, cartesianTarget};
        [~, ~, optimalSolution] = nlmpcmove(self.nlSolver, x0, u0, [],[], nloptions);

        % Post-processing solution
        optimalSolution.Xopt = optimalSolution.Xopt';
        
        % Scale normalized time by time factor
        Tf = optimalSolution.MVopt(1,8);
        optimalSolution.Topt = Tf .* optimalSolution.Topt';
        
        % Overwrite output variable (Y) to store cartesian trajectory
        optimalSolution.Yopt = self.robot.computeForwardKinematics(optimalSolution.Xopt(1:7,:));
        
        % Overwrite mannipulated variable (U) to store joint torque
        optimalSolution.MVopt = self.robot.computeInverseDynamic(...
            optimalSolution.Xopt(1:7,:), optimalSolution.Xopt(8:14,:), optimalSolution.MVopt(:,1:7));

        self.currentSolution = optimalSolution;
        
        if optimalSolution.ExitFlag < 0
            warning('Solver failed. Try relaxing some of your constraints')
        else
            disp('OPTIMIZATION SUCCESSFUL')
        end
        
        disp("--------- Reached target in "+ Tf + " seconds ---------");
        
    end
    
    function [jointPosition, jointSpeed, jointTorque] = getSolution(self, time)
        % Interpolate MPC solution at requested time
        
        X_interp = interp1(self.currentSolution.Topt, self.currentSolution.Xopt', time)';
        jointPosition = X_interp(1:7,:);
        jointSpeed = X_interp(8:14,:);
        jointTorque = interp1(self.currentSolution.Topt, self.currentSolution.MVopt', time)';
    end

    function showResults(self, optimalSolution, cartesianTarget, figTitle)
        % Show the results of the optimization
        
        cartesianTrajectory = optimalSolution.Yopt;
        cartesianSpeed = self.robot.computeForwardVelocity(optimalSolution.Xopt(1:7,:), optimalSolution.Xopt(8:14,:));
        
        % Animate the robot arm
        self.robot.animateTrajectory(optimalSolution.Xopt(1:7,:), cartesianTrajectory, cartesianSpeed, cartesianTarget, figTitle);
        
        % Plot trajectories
        f = figure('Name', figTitle);
        f.Position = [0, 0, 400, 700];
        
        subplot(4,1,1);
        plot(optimalSolution.Topt, rad2deg(optimalSolution.Xopt(1:7,:))); 
        xlabel('Time [s]')
        ylabel('Joint position [degree]')

        subplot(4,1,2); 
        plot(optimalSolution.Topt, rad2deg(optimalSolution.Xopt(8:14,:)));
        xlabel('Time [s]')
        ylabel('Joint velocity [degree/s]')
        
        subplot(4,1,3); 
        plot(optimalSolution.Topt, optimalSolution.MVopt); 
        xlabel('Time [s]')
        ylabel('Joint torque [N.m]')
        
        subplot(4,1,4); 
        plot(optimalSolution.Topt, cartesianTrajectory); 
        xlabel('Time [s]')
        ylabel('Cartesian position [m]') 
    end

    function showTaskVolume(self, trajectories)
        % Display a batch of trajectories in one plot.
        
        nPoints = size(trajectories, 2);
        nTraj = size(trajectories, 3);
        
        trajX = reshape(trajectories(1,:,:), nPoints, nTraj);
        trajY = reshape(trajectories(2,:,:), nPoints, nTraj);
        trajZ = reshape(trajectories(3,:,:), nPoints, nTraj);
        
        show(self.robot.robot, self.robot.computeInverseKinematics([trajX(end, 1), trajY(end, 1), trajZ(end, 1)]), ...
            'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
        
        hold on;
        % Plot trajectory and target in task space
        plot3(trajX, trajY, trajZ, 'k'); 
        plot3(trajX(1, :), trajY(1, :), trajZ(1, :), 'bo'); 
        plot3(trajX(end, :), trajY(end, :), trajZ(end, :), 'go'); 

        view([45, 45])
        axis([-0.7, 0.9, -0.8, 0.8, -0.3, 1.2])
    end
    
   end
end

function z = myStateFunction(x, u, robot, target)
    %z = u(8)*[x(8:14); robot.robot.forwardDynamics(x(1:7)', x(8:14)', u(1:7)')' ];
    z = u(8)*[x(8:14); u(1:7)];
end

% Analyticaly computes the Jacobian of the state dynamics
% function [A, Bmv] = myStateJacobian(X, U, robot, target)
%     A = zeros(4);
%     Bmv = cat(2, U(5)*eye(4), U(1:4));
% end

function ceq = myEqConFunction(X, U, data, robot, target)
    % This function enforces the constraint to reach a specific target at the
    % end of the horizon
    finalPose = robot.robot.getTransform(X(end, 1:7)', 'panda_link8', 'panda_link0');
    ceq = [(finalPose(1:3, 4) - target(1:3)); 
            X(end, 8:14)';
            diff(U(1:end-1, end))];
end
