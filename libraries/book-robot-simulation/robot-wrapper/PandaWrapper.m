%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper class for programming exercises in:                              %
% "Learning for Adaptive and Reactive Robot Control" by Billard,          %
% Mirrazavi and Figueroa.                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2020 Learning Algorithms and Systems Laboratory,          %
% EPFL, Switzerland                                                       %
% Author:  Alberic de Lajarte                                             %
% email:   alberic.lajarte@epfl.ch                                          %
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
classdef PandaWrapper
   properties
       robot
   end

   methods
       function self = PandaWrapper()
           % Create and store 'panda_arm' robot model
           % with the Matlab Robotics Toolbox. This class provides helper
           % functions for the robot object.
            
            warning('off','robotics:robotmanip:joint:ResettingHomePosition')
            self.robot = importrobot(fullfile('panda-model', 'panda_arm.urdf'), ...
                'MeshPath', fullfile('panda-model', 'meshes', 'collision'));
            self.robot.DataFormat = 'column';
            self.robot.Gravity = [0; 0; -9.81];
       end
    
       function randomTaskPosition = sampleRandomPosition(self)
           % Sample random joint configuration and compute forward kinematics
           q = self.robot.randomConfiguration;
           randomTaskPosition = self.computeForwardKinematics(q);
       end

       function cartesianTrajectory = computeForwardKinematics(self, jointTrajectory)
           % Compute cartesian trajectory from joint space trajectory
           % jointTrajectory is a 7xN matrix with the 7 joints in each column
           % cartesianTrajectory is a 3xN matrix with the 3D position in each column
           
           jointTrajectory = self.checkDimensions(jointTrajectory, 7);
           
           nTrajectory =  size(jointTrajectory, 2);

           cartesianTrajectory = nan(3, nTrajectory);
           for iTraj = 1:nTrajectory
               transform = self.robot.getTransform(jointTrajectory(:,iTraj), 'panda_link8', 'panda_link0');
               cartesianTrajectory(:,iTraj) = transform(1:3,4);
           end
       end

       function jointTrajectory = computeInverseKinematics(self, cartesianTrajectory, initialJoint, showBar, targetRotation)
           % Compute joint trajectory from cartesian space trajectory
           % cartesianTrajectory is a Nx3 or 3xN matrix
           % jointTrajectory is a 7xN matrix with the 7 joints in each column
           
           if nargin <= 2
               initialJoint = self.robot.homeConfiguration; % home configuration as initial guess;
           end

           if nargin <= 3
               showBar = true;
           end

           if nargin == 5
               R0 = targetRotation;
               weights = [1, 1, 1, 0.8, 0.8, 0.8]; % Wx, Wy, Wz, X, Y, Z
           else
               R0 = eye(3);
               weights = [0, 0, 0, 1, 1, 1]; % Wx, Wy, Wz, X, Y, Z
           end
           
           cartesianTrajectory = self.checkDimensions(cartesianTrajectory, 3);
          
           nTrajectory =  size(cartesianTrajectory, 2);
           jointTrajectory = nan(7, nTrajectory);
           
           inverseKinematic = inverseKinematics('RigidBodyTree', self.robot);
            
           if showBar
               h = waitbar(0, 'Computing inverse kinematics...');
               h.Position(3:4) = [300, 100];
               for iTraj = 1:nTrajectory
                   jointTrajectory(:,iTraj) = inverseKinematic('panda_link8', ...
                       [[R0, cartesianTrajectory(:,iTraj)]; [0, 0, 0, 1]] , weights, initialJoint);
                   initialJoint = jointTrajectory(:,iTraj);
                   waitbar(iTraj/nTrajectory, h)
               end
               close(h)
           else
               for iTraj = 1:nTrajectory
                   jointTrajectory(:, iTraj) = inverseKinematic('panda_link8', ...
                       [[R0, cartesianTrajectory(:, iTraj)]; [0, 0, 0, 1]] , weights, initialJoint);
                   initialJoint = jointTrajectory(:, iTraj);
               end
           end
       end
       
       function cartesianVelocity = computeForwardVelocity(self, jointTrajectory, jointSpeed)
           % Compute cartesian velocity from joint space trajectory and
           % speed
           % jointTrajectory is a 7xN matrix with the 7 joints in each column
           % jointSpeed is a 7xN matrix with the 7 joints in each column
           % cartesianVelocity is a 3xN matrix with the 3D speed in each column
           
           jointTrajectory = self.checkDimensions(jointTrajectory, 7);
           jointSpeed = self.checkDimensions(jointSpeed, 7);
           
           nTrajectory =  size(jointTrajectory, 2);

           cartesianVelocity = nan(3, nTrajectory);
           for iTraj = 1:nTrajectory
               jacobian = self.robot.geometricJacobian(jointTrajectory(:,iTraj), 'panda_link8');
               cartesianVelocity(:,iTraj) = jacobian(4:6,:) * jointSpeed(:,iTraj);
           end
       end
       
       function jointTorque = computeInverseDynamic(self, jointTrajectory, jointSpeed, jointAcceleration)
            % Compute joint torque from joint posiiton, speed an acceleration trajectory
            % jointTrajectory is a Nx7 or 7xN matrix
            % jointSpeed is a Nx7 or 7xN matrix
            % jointAcceleration is a Nx7 or 7xN matrix
            % jointTorque is a 7xN matrix with the 7 joints in each column     
           
            % Check dimensions of arguments
            jointTrajectory = self.checkDimensions(jointTrajectory, 7);
            jointSpeed = self.checkDimensions(jointSpeed, 7);
            jointAcceleration = self.checkDimensions(jointAcceleration, 7);
            
            nTrajectory =  size(jointTrajectory, 2);

            jointTorque = nan(7, nTrajectory);
            for iTraj = 1:nTrajectory
                jointTorque(:,iTraj) = self.robot.inverseDynamics(jointTrajectory(:,iTraj), ...
                    jointSpeed(:,iTraj), jointAcceleration(:,iTraj));
            end
       end
       
       function [simulatedTime, simulatedJoint, simulatedSpeed] = simulateRobot(self, optimalController, cartesianTarget)
           % Simulate tracking of MPC trakectory with a linear controller
           % and external disturbances.

           f = figure('Name', 'Simulation', 'WindowKeyPressFcn', {@keyPressCallback, optimalController}); 
           f.Position(3:4) = [800, 800];
           
           % Show at beginning else robot arm doesn't display properly
           show(self.robot, optimalController.currentSolution.Xopt(1:7, 1), ...
               'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on', 'FastUpdate', 1);
           hold on;

           % Plot trajectory and target in task space
           cartesianTrajectory = optimalController.currentSolution.Yopt;
           
           plot3(cartesianTrajectory(1,:), cartesianTrajectory(2,:), ...
               cartesianTrajectory(3,:), 'k', 'LineWidth', 1.5);
           plot3(cartesianTrajectory(1,1), cartesianTrajectory(2,1), ...
               cartesianTrajectory(3,1), 'bo', 'LineWidth', 1.5, 'MarkerSize', 15);
           plot3(cartesianTrajectory(1,end), cartesianTrajectory(2,end), ...
               cartesianTrajectory(3,end), 'go', 'LineWidth', 1.5, 'MarkerSize', 15);
           
           view([45, 45])
           axis([-0.7, 0.9, -0.8, 0.8, -0.3, 1.2])

           % Simulate and show robotic arm
           time = optimalController.currentSolution.Topt;      
           nTrajectory = length(time); 
           
           overSampling = 20;
           simulatedJoint = nan(7, overSampling*(nTrajectory-1));
           currentJoint = optimalController.currentSolution.Xopt(1:7, 1);
           simulatedSpeed = nan(7, overSampling*(nTrajectory-1));
           currentSpeed = optimalController.currentSolution.Xopt(8:14, 1);
           simulatedTime = nan(1, overSampling*(nTrajectory-1));

           optimalController.perturbations = zeros(7, 1);

           for iTraj=2:nTrajectory
               % Integrate forward given current state and control
               [t,y] = ode15s(@(t,y) forwardStateDynamic(t,y, optimalController), [time(iTraj-1) time(iTraj)], [currentJoint; currentSpeed]);
               
               currentJoint = y(end,1:7)';
               currentSpeed = y(end,8:14)';
               
               % Extract some points from the solution to log data
               sampletime = interp1(linspace(1, max(t), length(t)), t, linspace(1, max(t), overSampling));
               
               simulatedTime(:,(iTraj-2)*overSampling+1:(iTraj-1)*overSampling) = sampletime;
               simulatedJoint(:,(iTraj-2)*overSampling+1:(iTraj-1)*overSampling) = interp1(t, y(:,1:7), sampletime)';
               simulatedSpeed(:,(iTraj-2)*overSampling+1:(iTraj-1)*overSampling) = interp1(t, y(:,8:14), sampletime)'; 
               
               pos = self.computeForwardKinematics(currentJoint);
               plot3(pos(1), pos(2), pos(3), 'r*', 'LineWidth', 1.5, 'MarkerSize', 15);
               
               show(self.robot, currentJoint, 'PreservePlot', false, 'Frames', 'on', 'Visuals', 'on', 'FastUpdate', 1);
               rotate3d off;
               drawnow
           end
           rotate3d on;

           cartesianTrajectory = self.computeForwardKinematics(simulatedJoint);
           plot3(cartesianTrajectory(1,:), cartesianTrajectory(2,:), cartesianTrajectory(3,:), 'r', 'LineWidth', 0.5);
           
           % Display final error
           disp("Offset at target is " + num2str(1000 * norm(pos - cartesianTarget(1:3)),'%.2f') + "mm");
           disp("Velocity error is " + num2str(100 * norm( ...
               self.computeForwardVelocity(simulatedJoint(:,end), simulatedSpeed(:,end)) - cartesianTarget(4:6)),'%.1f') + "cm/s");

       end

       function animateTrajectory(self, jointTrajectory, cartesianTrajectory, cartesianSpeed, cartesianTarget, figTitle, figHandle)
           % Show motion of robot from joint trajectory.

           % Check dimensions of arguments
           cartesianTrajectory = self.checkDimensions(cartesianTrajectory, 3);
           
           if nargin == 7
               f = figHandle;
           else
               f = figure('Name', figTitle); 
           end
           f.Position(3:4) = [800, 800];

           show(self.robot, jointTrajectory(:,1), 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
           hold on;
           % Plot trajectory and target in task space
           plot3(cartesianTrajectory(1,:), cartesianTrajectory(2,:), cartesianTrajectory(3,:), ...
               'k', 'LineWidth', 1.5);
           plot3(cartesianTrajectory(1,1), cartesianTrajectory(2,1), cartesianTrajectory(3,1), ...
               'bo', 'LineWidth', 1.5, 'MarkerSize', 15);
           plot3(cartesianTarget(1), cartesianTarget(2), cartesianTarget(3), ...
               'ro', 'LineWidth', 1.5, 'MarkerSize', 15);
           plot3(cartesianTrajectory(1,end), cartesianTrajectory(2,end), cartesianTrajectory(3,end), ...
               'go', 'LineWidth', 1.5, 'MarkerSize', 15);
            
           quiver3(cartesianTrajectory(1,:), cartesianTrajectory(2,:), cartesianTrajectory(3,:), ...
                   cartesianSpeed(1,:), cartesianSpeed(2,:), cartesianSpeed(3,:), 0.5, 'b', 'LineWidth', 1)

           view([45, 45])
           axis([-0.7 0.9 -0.8 0.8 -0.3 1.2])
           
           for iTraj = 1:size(jointTrajectory, 2)
               pos = self.computeForwardKinematics(jointTrajectory(:,iTraj));
               plot3(pos(1), pos(2), pos(3), 'kx', 'LineWidth', 1.5, 'MarkerSize', 15);
               show(self.robot, jointTrajectory(:,iTraj), 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
               drawnow
           end
       end
       
       % Check dimensions of matrix, and transpose it if necessary
       % Convention is that each row is a vector of dimension = nColumn
       function correctedMatrix = checkDimensions(self, inputMatrix, dimension)
           
           correctedMatrix = inputMatrix;
           
           % Check is matrix is in wrong order
           if size(correctedMatrix, 1) ~= dimension && size(inputMatrix, 2) == dimension
               correctedMatrix = correctedMatrix';
           end
           % Check if input matrix or its transpose have the right vector
           % dimension   
           assert(size(correctedMatrix, 1) == dimension, ...
               "Input trajectory must be Nx" +int2str(dimension) + " or " + int2str(dimension) + "xN matrix")
       end
   end
end

function dydt = forwardStateDynamic(t, y, optimalController)

    optimalController.currentTime = t;
    
    % Current state    
    jointPosition = y(1:7);
    jointSpeed = y(8:14);
    
    % Compute corresponding controlled torque (feed-forward PD controller)
    [targetPosition, targetSpeed, targetTorque] = optimalController.getSolution(t);
    controlTorque = targetTorque + 20 * (targetSpeed - jointSpeed) + ...
        1000 * (targetPosition - jointPosition);

    % Compute state derivative from current state and control
    dydt = zeros(14, 1);
    dydt(1:7) = jointSpeed;
    dydt(8:14) = optimalController.robot.robot.forwardDynamics(jointPosition, jointSpeed, controlTorque) ...
                                        + optimalController.perturbations;
end

function keyPressCallback(source, eventdata, control, currentJoint)
      % determine the key that was pressed
      keyPressed = eventdata.Key;

      [targetPosition, ~, ~] = control.getSolution(control.currentTime);
      
      jacobian = control.robot.robot.geometricJacobian(targetPosition, 'panda_link8');
      newPerturbation = zeros(7, 1);
      forcePerturbation = 40;

      if strcmp(keyPressed,'uparrow')==1
          newPerturbation = jacobian'*[0; 0; 0; 0; 0; forcePerturbation];

      elseif strcmp(keyPressed,'downarrow')==1
          newPerturbation = jacobian'*[0; 0; 0; 0; 0; -forcePerturbation];

      elseif strcmp(keyPressed,'leftarrow')==1
          newPerturbation = jacobian'*[0; 0; 0; 0; -forcePerturbation; 0];

      elseif strcmp(keyPressed,'rightarrow')==1
          newPerturbation = jacobian'*[0; 0; 0; 0; forcePerturbation; 0];
      end   

      control.perturbations = control.perturbations + newPerturbation;
end
