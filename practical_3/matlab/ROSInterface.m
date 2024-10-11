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
classdef ROSInterface < handle
    properties
      interfaceNode
      poseSub
      optitrackSub
      twistPub
      pathPub
      demoPub
      obstacleSub
      attractorPub
      countObsUpdate

      main_handle % function handle to main loop
      obs_handle % function handle to obstacle loop (updtaes their live position in DS and Rviz )
    end

    methods
       
        function self = ROSInterface(main_handle, obs_handle)

            setenv("ROS_DOMAIN_ID", "14");
        
            self.interfaceNode = ros2node("/interfaceNode", 14);

            pause(1)
            self.poseSub = ros2subscriber(self.interfaceNode,"/robot_state", "std_msgs/Float64MultiArray", @self.poseCallback);
            self.twistPub = ros2publisher(self.interfaceNode, "/command_twist","geometry_msgs/TwistStamped");
            self.pathPub = ros2publisher(self.interfaceNode, "/ds_trajectory","nav_msgs/Path");
            self.demoPub = ros2publisher(self.interfaceNode, "/demo_trajectory","nav_msgs/Path");
            self.obstacleSub = ros2subscriber(self.interfaceNode, "/visualization_marker","visualization_msgs/Marker", @self.obstacleCallback);
            self.attractorPub = ros2publisher(self.interfaceNode, "/attractor_pos","geometry_msgs/Vector3");
            % Empty message to initialize Rviz
            self.sendOpenLoopTrajectory(@(x) zeros(3,1), zeros(3,1));
            %self.sendObstacleInfo(8);
            self.countObsUpdate = 0;

            self.main_handle = main_handle;
            self.obs_handle = obs_handle; 
            
        end
        
        % Callback to handle ROS robot pose messages
        function poseCallback(self, message)
    
            currentTime = message.data(1);
            currentPosition =  message.data(2:4);
            currentVelocity = message.data(5:7);

            % Update Robot position
            self.main_handle(currentTime, currentPosition, currentVelocity);

        end
        
        function obstacleCallback(self, message)

            % Extract data from the ROS message
            id = message.id; % Assuming 'Id' is the field in the ROS message
            pose = message.pose; % Pose information (position and orientation
            position = [pose.position.x, pose.position.y, pose.position.z];

            radius = message.scale.x;
            
            rho = message.scale.z;

            color = message.color;
  
            % send info to mainHub
            self.obs_handle(id, position, radius, rho, color)
        end

        function sendTwistCommand(self, velocityCommand)

            % Create ROS message
            twistMsg = ros2message(self.twistPub);
            twistMsg.twist.linear.x = velocityCommand(1);
            twistMsg.twist.linear.y = velocityCommand(2);
            twistMsg.twist.linear.z = velocityCommand(3);

            twistMsg.header.frame_id = 'panda_base';
    
            self.twistPub.send(twistMsg)
        end

        function sendOpenLoopTrajectory(self, dsControl, initialPosition)
            
            pathMsg = ros2message(self.pathPub);

            if initialPosition ~=  zeros(3,1) 

                opt_sim = []; 
                opt_sim.plot  = 0;
                opt_sim.dt = 3e-3;
                opt_sim.i_max = 10000;
                echo off;
                [~, x_sim, ~, t, ~] = evalc('Simulation(initialPosition , [], dsControl, opt_sim)');
    
                nPoints = 100;
                x_sim = interp1(t, x_sim', linspace(t(1), t(end), nPoints))';
    
                pathMsg.header.frame_id = 'world';
    
                for i=1:nPoints
    
                    pose = ros2message("geometry_msgs/PoseStamped");
                    pose.header.frame_id = 'world';

                    pose.pose.position.x = x_sim(1, i);
                    pose.pose.position.y = x_sim(2, i);
                    pose.pose.position.z = x_sim(3, i);
    
                    pathMsg.poses = [pathMsg.poses, pose];
                end
    
                pathMsg.poses = pathMsg.poses(2:end);
            end
            
            self.pathPub.send(pathMsg);
        end


        function sendDemonstration(self, demonstration)
            
            demoMsg = ros2message(self.demoPub);

            columns_size = size(demonstration, 2);

            if columns_size == 200
                demoMsg.header.frame_id = 'world';

            elseif columns_size == 1
                demoMsg.header.frame_id = 'remove_last';

            elseif columns_size == 2
                demoMsg.header.frame_id = 'remove_all';

            end

            for i=1:size(demonstration, 2)

                pose = ros2message("geometry_msgs/PoseStamped");
                pose.header.frame_id = 'world';
                
                pose.pose.position.x = demonstration(1, i);
                pose.pose.position.y = demonstration(2, i);
                pose.pose.position.z = demonstration(3, i);

                demoMsg.poses = [demoMsg.poses, pose];
            end

            demoMsg.poses = demoMsg.poses(2:end);

            self.demoPub.send(demoMsg);
        end


        function sendObstacleInfo(self,k, obstacle)
            
            % to send empty first message or when deleting
            if exist('obstacle', 'var') == 0
                type = int32(2);
                ns = 'world';
                axes = [0.01;0.01;0.01];
                obstacle.position = [10;10;10];
                q = [0;0;0;1];

                color = [0;0;0;0]; % alpha in 1
            end
            
            % identify type of obstacle
            if isprop(obstacle, 'ellipseAxes') %is ellipse
                type = int32(2);
                ns = 'world';
                axes = 2*obstacle.ellipseAxes;
                q = [0;0;0;1];

                color = [1.0;0;0;1.0]; % alpha in 1
            elseif isprop(obstacle, 'radius')% is cylinder
                type = int32(3);
                ns = 'world';
                axes = 2*[obstacle.radius ; obstacle.radius ; 2.5];
                q = [0;0;0;1];

                color = [0.6;0;1.0;0]; % alpha in 1
            elseif isprop(obstacle, 'normal')
                type = int32(1);
                ns = 'world';
                axes = [2.5; 2.5 ; 0.02];
                color = [0.4;1.0;0;0]; % alpha in 1
                % Convert normal vector to quaternion
                vertical = [0;0;1];
                % check to avoid NaN q
                if dot(obstacle.normal, vertical) == -1 
                    % if normal parallel & opposite to vertical, rotate 180 deg
                    obstacle.normal =  [1, 0, 0 ; 0, 1, 0; 0, 0, -1]* obstacle.normal;
                end

                a = cross(vertical, obstacle.normal);
                w = sqrt((norm(obstacle.normal) ^ 2) * (norm(vertical) ^ 2)) + dot(obstacle.normal, vertical);
                q = [a ; w];
                q = q/norm(q);
            elseif isprop(obstacle, 'obstacleType')
                type = int32(3);
                ns = obstacle.obstacleType; % used to send info to python code
                q = [0;0;0;1];
                axes = [0.0; 0.0 ; 0.0];
                color = [1.0;0.0;0;0]; % alpha in 1
            end
        
            obstacleMsg = ros2message(self.obstaclePub);
            obstacleMsg.ns = ns;
            obstacleMsg.header.frame_id = 'world';

            obstacleMsg.id = int32(k); % name of object, based on its position in listofObstacles
            obstacleMsg.type = type; % cylinder=3, cube=1, sphere=2
            obstacleMsg.action = int32(0); % 0= ADD/Modify
            
            obstacleMsg.pose.position.x = obstacle.position(1);
            obstacleMsg.pose.position.y = obstacle.position(2);
            if isprop(obstacle, 'normal') % adjust height for plane z-dimension (2cm)
                obstacleMsg.pose.position.z = obstacle.position(3)-0.01;
            else
                obstacleMsg.pose.position.z = obstacle.position(3);
            end

            obstacleMsg.pose.orientation.x = q(1);
            obstacleMsg.pose.orientation.y = q(2);
            obstacleMsg.pose.orientation.z = q(3);
            obstacleMsg.pose.orientation.w = q(4);
               
            obstacleMsg.scale.x = axes(1);
            obstacleMsg.scale.y = axes(2);
            obstacleMsg.scale.z = axes(3);
            obstacleMsg.color.a = single(color(1)); % Don't forget to set the alpha!
            obstacleMsg.color.r = single(color(2));
            obstacleMsg.color.g = single(color(3));
            obstacleMsg.color.b = single(color(4));

            obstacleMsg.lifetime.sec = int32(0); % Set to 0 to last forever

            self.obstaclePub.send(obstacleMsg);

        end

        function sendRemoveObstacle(self, k, obstacle)
            
            % Send color=0 when deleting
       
            if isprop(obstacle, 'obstacleType')
                ns = obstacle.obstacleType; % used to send info to python code
            else 
                ns = 'world';
            end
        
            obstacleMsg = ros2message(self.obstaclePub);
            obstacleMsg.ns = ns;
            obstacleMsg.header.frame_id = 'world';

            obstacleMsg.id = int32(k); % name of object, based on its position in listofObstacles
            obstacleMsg.type = int32(2); % cylinder=3, cube=1, sphere=2
            obstacleMsg.action = int32(0); % 0= ADD/Modify
            
            obstacleMsg.pose.position.x = 10;
            obstacleMsg.pose.position.y = 10;
            obstacleMsg.pose.position.z = 10;

            obstacleMsg.pose.orientation.x = 0;
            obstacleMsg.pose.orientation.y = 0;
            obstacleMsg.pose.orientation.z = 0;
            obstacleMsg.pose.orientation.w = 1;
               
            obstacleMsg.scale.x = 0.01;
            obstacleMsg.scale.y = 0.01;
            obstacleMsg.scale.z = 0.01;
            obstacleMsg.color.a = single(0); % Don't forget to set the alpha!
            obstacleMsg.color.r = single(0);
            obstacleMsg.color.g = single(0);
            obstacleMsg.color.b = single(0);

            obstacleMsg.lifetime.sec = int32(0); % Set to 0 to last forever
            
            % Send multiple times to bypass traffic with updateSim()
            self.obstaclePub.send(obstacleMsg);
            self.obstaclePub.send(obstacleMsg);
            % self.obstaclePub.send(obstacleMsg);
            % self.obstaclePub.send(obstacleMsg);

        end

        function sendAttractorPos(self, attractor)
                    
            attMsg = ros2message(self.attractorPub);
            
            attMsg.x = attractor(1);
            attMsg.y = attractor(2);
            attMsg.z = attractor(3);

            self.attractorPub.send(attMsg);
        end


        function setFreezeMode(self, value)
             
            nodeParams = ros2param("/matlab_bridge");
            % Check existence of parameter
            % nameExists = has(nodeParams,"freeze_mode");
            set(nodeParams,'freeze_mode',value);
        end

        function sendResetRobot(self)
            nodeParams = ros2param("/matlab_bridge");
            %nameExists = has(nodeParams,"reset_robot");
            set(nodeParams,'reset_robot', true);
        end

        function setAttractorParam(self, attractor)
            % disp("Setting attractor param")
            % nodeParams = ros2param("/matlab_bridge");
            % %nameExists = has(nodeParams,"reset_robot");
            % set(nodeParams,'attractor', attractor);
            if isnumeric(attractor) && isvector(attractor) && numel(attractor) == 3
                nodeParams = ros2param("/matlab_bridge");
                set(nodeParams, 'attractor', attractor);
            else
                error('Attractor must be a numeric vector with 3 elements.');
            end
        end
        
        % Destructor to shut down ROS
        function delete(self)
            clear self.interfaceNode
            clear self.poseSub
            clear self.twistPub 

            disp("Cleared communications. Over.")
        end


    end
end
