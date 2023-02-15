%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper class for programming exercises in:                              %
% 'Learning for Adaptive and Reactive Robot Control' by Billard,          %
% Mirrazavi and Figueroa.                                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2020 Learning Algorithms and Systems Laboratory,          %
% EPFL, Switzerland                                                       %
% Author:  Dominic Reber                                                  %
% email:   dominic.reber@epfl.ch                                          %
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
classdef RobotisWrapper
   properties
       robot
   end

   methods
       function self = RobotisWrapper()
           % Create and store 'robotisOpenManipulator' robot model
           % with the Matlab Robotics Toolbox. This class provides helper
           % functions for the robot object.
           self.robot = loadrobot('robotisOpenManipulator', 'DataFormat', 'column');
       end
    
       function randomTaskPosition = sampleRandomPosition(self)
           % Sample random joint configuration and compute forward kinematics
           q = self.robot.randomConfiguration;
           randomTaskPosition = self.fastForwardKinematic(q(1:4));
       end

       function randomJointPosition = sampleRandomConfiguration(self)
           % Sample random joint configuration
           q = self.robot.randomConfiguration;
           randomJointPosition = q(1:4);
       end

       function cartesianTrajectory = computeForwardKinematics(self, jointTrajectory)
           % Compute cartesian trajectory from joint space trajectory
           % jointTrajectory is a 4xN or Nx4 matrix with the 4 joints in each row
           % cartesianTraj is a 3xN matrix with the 3D position in each row
           if size(jointTrajectory, 1) ~= 4 && size(jointTrajectory, 2) == 4
               jointTrajectory = jointTrajectory';
           end
           if size(jointTrajectory, 1) ~= 4
               error('Input trajectory must be Nx4 or 4xN matrix')
           end
           trajectorySize =  size(jointTrajectory, 2);

           cartesianTrajectory = zeros(3, trajectorySize);
           for i = 1:trajectorySize
               cartesianTrajectory(:,i) = self.fastForwardKinematic(jointTrajectory(:,i));
           end
       end

       function jointTrajectory = computeInverseKinematics(self, cartesianTrajectory, initialJoint, showBar)
           % Compute joint trajectory from cartesian space trajectory
           % cartesianTrajectory is a 3xN or Nx3 matrix
           % jointTrajectory is a 4xN matrix with the 4 joints in each row
           
           if nargin <= 2
               initialJoint = self.robot.homeConfiguration; % home configuration as initial guess;
           end

           if nargin <= 3
               showBar = true;
           end
           
           if size(cartesianTrajectory, 1) ~= 3 && size(cartesianTrajectory, 2) == 3
               cartesianTrajectory = cartesianTrajectory';
           end
           if size(cartesianTrajectory, 1) ~= 3
               error('Input trajectory must be Nx3 or 3xN matrix')
           end
           trajectorySize =  size(cartesianTrajectory, 2);

           jointTrajectory = zeros(4, trajectorySize);
           
           inverseKinematic = inverseKinematics('RigidBodyTree', self.robot);
           weights = [0, 0, 0, 1, 1, 1]; % Wx, Wy, Wz, X, Y, Z
            
           if showBar
               h = waitbar(0, 'Computing inverse kinematics...');
               h.Position(3:4) = [300, 100];
               for i = 1:trajectorySize
                   qs = inverseKinematic('end_effector_link', ...
                       trvec2tform(cartesianTrajectory(:,i)'), weights, initialJoint);
                   initialJoint = qs;
                   jointTrajectory(:,i) = qs(1:4);
                   waitbar(i/trajectorySize, h)
               end
               close(h)
           else
               for i = 1:trajectorySize
                   qs = inverseKinematic('end_effector_link', ...
                       trvec2tform(cartesianTrajectory(:,i)'), weights, initialJoint);
                   initialJoint = qs;
                   jointTrajectory(:,i) = qs(1:4);
               end
           end
       end

       function cartesianVelocity = computeForwardVelocity(self, jointTrajectory, jointSpeed)
           % Compute cartesian velocity from joint space trajectory and
           % speed
           % jointTrajectory is a 4xN matrix with the 4 joints in each column
           % jointSpeed is a 4xN matrix with the 4 joints in each column
           % cartesianVelocity is a 3xN matrix with the 3D speed in each column
           if size(jointTrajectory, 1) ~= 4 && size(jointTrajectory, 2) == 4
               jointTrajectory = jointTrajectory';
           end
           if size(jointTrajectory, 1) ~= 4
               error('Input trajectory must be Nx4 or 4xN matrix')
           end

           if size(jointSpeed, 1) ~= 4 && size(jointSpeed, 2) == 4
               jointSpeed = jointSpeed';
           end
           if size(jointSpeed, 1) ~= 4
               error('Input speed must be Nx4 or 4xN matrix')
           end
           
           nTrajectory =  size(jointTrajectory, 2);

           cartesianVelocity = nan(3, nTrajectory);
           for iTraj = 1:nTrajectory
               jacobian = self.fastJacobian(jointTrajectory(:,iTraj));
               cartesianVelocity(:,iTraj) = jacobian * jointSpeed(:,iTraj);
           end
       end

       function animateTrajectory(self, jointTrajectory, cartesianTrajectory, cartesianTarget, figTitle, figHandle)
           % Animate the robot on a trajectory
           if size(jointTrajectory, 1) ~= 4  && size(jointTrajectory, 2) == 4
               jointTrajectory = jointTrajectory';
           end
           if size(jointTrajectory, 1) ~= 4
               error('Input trajectory must be Nx4 or 4xN matrix')
           end
           if size(cartesianTrajectory, 1) ~= 3 && size(cartesianTrajectory, 2) == 3
               cartesianTrajectory = cartesianTrajectory';
           end
           if size(cartesianTrajectory, 1) ~= 3
               error('Input trajectory must be Nx3 or 3xN matrix')
           end

           if nargin == 6
               f = figHandle;
           else
               f = figure('Name', figTitle); 
           end
           
           screensize = get(groot, 'Screensize');
           f.Position = [screensize(3) / 2, 0.1 * screensize(4), screensize(3) / 2, 0.8 * screensize(4)];

           show(self.robot, [jointTrajectory(:,1); 0; 0], 'PreservePlot', false, ...
               'Frames', 'off', 'Visuals', 'on');
           hold on;
           % Plot trajectory and target in task space
           plot3(cartesianTrajectory(1,:), cartesianTrajectory(2,:), ...
               cartesianTrajectory(3,:), 'k', 'LineWidth', 1.5);
           plot3(cartesianTrajectory(1,1), cartesianTrajectory(2,1), ...
               cartesianTrajectory(3,1), 'bo', 'LineWidth', 1.5, 'MarkerSize', 15);
           plot3(cartesianTrajectory(1,end), cartesianTrajectory(2, end), ...
               cartesianTrajectory(3,end), 'go', 'LineWidth', 1.5, 'MarkerSize', 15);
           plot3(cartesianTarget(1), cartesianTarget(2), cartesianTarget(3), ...
               'ro', 'LineWidth', 1.5, 'MarkerSize', 15);

           view([45, 45])
           axis([-0.2 0.4 -0.35 0.35 -0.1 0.5])

           for i = 1:size(jointTrajectory, 2)
               pos = self.fastForwardKinematic(jointTrajectory(:,i));
               plot3(pos(1), pos(2), pos(3), 'kx', 'LineWidth', 1.5, 'MarkerSize', 15);
               show(self.robot, [jointTrajectory(:,i); 0; 0], ...
                   'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
               drawnow
           end
       end
       
       function cartesianPosition = fastForwardKinematic(self, joint_configuration)
           % Calculate the forward kinematics of the robotisOpenManipulator. For a
           % given joint configuration q, this returns the position of the end
           % effector in cartesian coordinates.
           if length(joint_configuration) ~= 4
               error('Input configuration must be vector of size 4');
           end
           
           t2 = cos(joint_configuration(1));
           t3 = cos(joint_configuration(2));
           t4 = cos(joint_configuration(3));
           t5 = cos(joint_configuration(4));
           t6 = sin(joint_configuration(1));
           t7 = sin(joint_configuration(2));
           t8 = sin(joint_configuration(3));
           t9 = sin(joint_configuration(4));
           t10 = t3.*t4;
           t11 = t7.*t8;
           t12 = t4.*t7;
           t14 = t3.*t8;
           cartesianPosition = [t2.*t3.*0.0240+t2.*t7.*0.1280+t5.*(t2.*t10-t2.*t11).*0.1260-t9.*(t2.*t14+t2.*t12).*0.1260+t2.*t10.*0.1240-t2.*t11.*0.1240+0.0120; ...
                                t3.*t6.*0.0240+t6.*t7.*0.1280+t5.*(t10.*t6-t6.*t11).*0.1260-t9.*(t14*t6+t12*t6).*0.1260+t10.*t6.*0.1240-t6.*t11.*0.1240; ...
                                t3.*0.1280-t7.*0.0240-t14.*0.1240-t12.*0.1240-t5.*(t14+t12).*0.1260-t9.*(t10-t11).*0.1260+7.65e-2];
       end

       function jacobian = fastJacobian(self, joint_configuration)
           % Calculate the jacobian of the robotisOpenManipulator for a given 
           % joint configuration.
           t2 = cos(joint_configuration(1));
           t3 = cos(joint_configuration(2));
           t4 = cos(joint_configuration(3));
           t5 = cos(joint_configuration(4));
           t6 = sin(joint_configuration(1));
           t7 = sin(joint_configuration(2));
           t8 = sin(joint_configuration(3));
           t9 = sin(joint_configuration(4));
           t10 = t3.*t4;
           t11 = t3.*t8;
           t12 = t4.*t7;
           t13 = t7.*t8;
           t14 = t6.*t13;
           t16 = t2.*t10;
           t17 = t2.*t11;
           t18 = t2.*t12;
           t19 = t6.*t10;
           t20 = t2.*t13;
           t21 = t6.*t11;
           t22 = t6.*t12;
           t25 = t10.*0.124;
           t26 = t13.*0.124;
           t27 = t11+t12;
           t24 = -t2.*t13;
           t29 = t10-t13;
           t30 = t17.*0.124;
           t31 = t18.*0.124;
           t32 = t21.*0.124;
           t33 = t22.*0.124;
           t38 = t17+t18;
           t39 = t21+t22;
           t42 = t9.*t27.*0.126;
           t40 = t16+t24;
           t41 = t14-t19;
           t43 = t5.*t29.*0.126;
           t45 = t5.*t38.*0.126;
           t46 = t5.*t39.*0.126;
           t49 = t9.*t40.*0.126;
           t50 = t9.*t41.*0.126;

           jacobian = [(t14*0.124-t19*0.124-t3*t6*0.024-t6*t7*0.128+t5*t41*0.126+t9*t39*0.126), (-t30-t31-t45-t49+t2.*t3.*0.128-t2.*t7.*0.024), -t30-t31-t45-t49, (-t45-t49);  
                       (t16.*0.124-t20.*0.124+t2.*t3.*0.024+t2.*t7.*0.128+t5.*t40.*0.126-t9.*t38.*0.126), (-t32-t33-t46+t50+t3.*t6.*0.128-t6.*t7.*0.024), (-t32-t33-t46+t50), (-t46+t50);
                       (0.0), (t3.*(-0.024)-t7.*0.128+t26-t25+t42-t43), (t26-t25+t42-t43), (t42-t43)];
       end
   end
end
