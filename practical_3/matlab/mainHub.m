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

classdef mainHub < handle
    % Class to centralize all communication
    
    properties
        figHandle1
        figHandle2
        btnStartRecording
        btnStopRecording
        btnLearn
        btnPlay
        btnStop
        btnDeleteTrajectory
        btnClearDataset
        btnFreezeMode
        inputAttractor
        dropdownAlgo
        btnToggle
        robot
        axisLimit

        appState
        rosInterface

        demoTrajectories % Cell array to record states into trajectories
        trajectoryDataset % dataset of formatted trajectories
        iTraj % index of current trajectory

        myDS % associated DS object

        predefExists
        removeObs
        t
        delta_t

    end
    
    methods
        function self = mainHub(myDS)
            
            % link to DS
            self.myDS = myDS;

            % Create figure and initial GUI setup
            screensize = get(groot, 'Screensize');    

            self.figHandle1 = figure('Name', 'Control', 'NumberTitle', 'off', 'MenuBar', 'none', 'ToolBar', 'none', 'DeleteFcn', {@deleteCallback, self});
            self.figHandle1.Position = [0, 0, 200, 480]; 

            self.figHandle2 = figure('Name', 'Learning', 'NumberTitle', 'off', 'MenuBar', 'none', 'ToolBar', 'none', 'DeleteFcn', {@deleteCallback, self});
            self.figHandle2.Position = [200, 0, 200, 450]; 

            % Create buttons
            self.createButtonsFigure1();
            self.createButtonsFigure2();
          
            % Initial variables
            self.iTraj = 1;
            self.appState = 'Idle';
            
            % Set these to avoid errors
            self.axisLimit = [-0.7, 0.9, -0.8, 0.8, -0.3, 1.2];
            self.myDS.myWorld.axisLimit = self.axisLimit; % link to Env for obstacles

            % Object to interface with ROS
            % % self.rosInterface = ROSInterface(@self.main_loop, @self.updateObsfromOptitrack);
            self.rosInterface = ROSInterface(@self.main_loop, @self.updateObstacle);

            self.predefExists = false; 
            self.removeObs = 0;
            self.t = 0;
            self.delta_t = 1e-4;

        end
        
        function self = main_loop(self, currentTime, currentPosition, currentVelocity)
            
            % update Sim constantly
            % self.updateSim()

            switch self.appState
                case 'Idle'
                    % do nothing

                case 'Record'
                    % Append new state to current trajectory
                    self.demoTrajectories{self.iTraj} = [self.demoTrajectories{self.iTraj}, [currentTime; currentPosition; currentVelocity]];
                
                case 'Stop'
                    % Stop filling array, increment index to prepare next
                    % trajectory
                    % Stop robot from receiving twist from learned DS
                    velocityCommand = [0;0;0];
                    self.rosInterface.sendTwistCommand(velocityCommand);

                    % Remove ds traj from display 
                    self.rosInterface.sendOpenLoopTrajectory(@(x) zeros(3,1), zeros(3,1));

                    self.appState = 'Idle'; 

                case 'Learn'
                    % Change state now to avoid mutliple calls to learn functions
                    self.appState = 'Idle';
                    
                    % Learn DS if there is data
                    if isempty(self.trajectoryDataset)
                        disp('Using trajectories in provided dataset.')
                        
                        %%% ---------- CHOOSE DATASET HERE -------- %%%
                        % traj2load = load('datasets/demonstration_dataset.mat');
                        traj2load = load('datasets/MPC_dataset.mat');
                        % traj2load = load('datasets/theoretical_DS_dataset.mat');

                        self.trajectoryDataset = traj2load.trajectories;
                    end

                    % Run desired learning algorithm
                    if strcmp(self.myDS.algoName,'LPVDS')
                        self.myDS.learnLPVDS(self.trajectoryDataset);
                    elseif strcmp(self.myDS.algoName,'SEDS')
                        self.myDS.learnSEDS(self.trajectoryDataset);
                    else
                        fprintf("Must specify algorithm name when instantiating DS")
                    end

                case 'Play'
                    self.rosInterface.sendOpenLoopTrajectory(self.myDS.dsControl, currentPosition)
                    self.rosInterface.sendAttractorPos(self.myDS.attractor);
                    self.appState = 'Control'; 

                case 'Control'
                    % Run DS to command robot
                    velocityCommand = self.myDS.dsControl(currentPosition);
                    if all(~isnan(velocityCommand))
                        self.rosInterface.sendTwistCommand(velocityCommand);
                    else
                        self.rosInterface.sendTwistCommand(1e-6*ones(3, 1));
                    end
            
                otherwise
                    error("Unrecognized state in FSM")
            end
        end

        function updateRviz(self)
            % Update Rviz with all obstacles

            % first call to empty RViz
%             for k =1:10
%                 %disp('Cleaning Rviz')
%                 self.rosInterface.sendObstacleInfo(k);
%             end
      

            % call to refresh visuals
            kObst = length(self.myDS.myWorld.listOfObstacles);
            if kObst > 1
                for k = 2:kObst
%                     disp('Updating Rviz')
                    self.rosInterface.sendObstacleInfo(k,self.myDS.myWorld.listOfObstacles(k));
                end
            end
        end

        function updateSim(self)
            % Update Sim with all obstacles
            
            if self.removeObs ~= 0 
                % remove from my world list of obstacles            
                self.rosInterface.sendRemoveObstacle(self.removeObs,self.myDS.myWorld.listOfObstacles(self.removeObs));
                self.myDS.myWorld.listOfObstacles(self.removeObs) = [];
                self.removeObs = 0; % reset flag
            else 
    
                kObst = length(self.myDS.myWorld.listOfObstacles);
                for k = 1:kObst
                    self.myDS.myWorld.listOfObstacles(k).moveShape(self.t);
                    self.t = self.t + self.delta_t;
                    self.rosInterface.sendObstacleInfo(k,self.myDS.myWorld.listOfObstacles(k));
                end
            end
        end

        function updateObstacle(self, obstacleID, newPosition, newRadius, newRho, color)

            % check obstacle exists
            nObs = length(self.myDS.myWorld.listOfObstacles);
            obs_index = 0;
            for k = 1:nObs
                obstacle = self.myDS.myWorld.listOfObstacles(k);
                if obstacle.id == obstacleID
                    obs_index = k;
                end
            end

            if obs_index == 0  % add new obstacle
                self.addSphere(newPosition, newRadius, newRho, obstacleID);
                disp("Adding obstacle")
            elseif color.a == 0.0 % remove obstacle
               
               self.myDS.myWorld.listOfObstacles(obs_index) = [];
               disp("Removing obstacle")
            else %update existing obstacle
            
                obstacle = self.myDS.myWorld.listOfObstacles(obs_index);

                % update obstacle alpha/rho
                obstacle.rho = newRho;
    
                % update obstacle properties
                newAxes = [newRadius; newRadius; newRadius];
                obstacle.ellipseAxes = newAxes;
                obstacle.shapeParameters.ellipseAxes = newAxes;

                % update object position
                obstacle.position = newPosition ;
            end
        end
        
        % Function to update obstacle position live 
        function updateObsfromOptitrack(self, ObsPosition)
               
            % Update Obstacle in myWorld -> Tracked obstacle is always in
            % 1st position in listOfObstacles
            obstacle = self.myDS.myWorld.listOfObstacles(1);
               
            % update object position
            obstacle.position = ObsPosition;
   
        end
    

        function deleteObstacle(self, obstacleNumber)
            % Remove one obstacle from myWorld and Rviz
            
            % Remove form plot via vertices
            obstacle = self.myDS.myWorld.listOfObstacles(obstacleNumber);
            vertices = get(obstacle.patchVal, 'Vertices');
            empty_vertices = zeros(size(vertices));
            set(obstacle.patchVal, 'Vertices', empty_vertices);

            if obstacleNumber == 0 
                disp('Cannot remove non existing obstacle. Index starts at 1!');
    
            elseif obstacleNumber > length(self.myDS.myWorld.listOfObstacles)
                fprintf('Index too high! there are only %s obstacles \n', string(length(self.myDS.myWorld.listOfObstacles)));

            else
                if isprop(obstacle, 'obstacleType')
                      self.predefExists = false;
                end 
                
                % Set flag to remove obs at next updateSim() call
                self.removeObs = obstacleNumber;

                disp('Finished deleting obstacle');
            end
        end

        function self = formatTrajectories(self)
        
            nSamplePoint = 200;
            self.demoTrajectories = self.demoTrajectories(~cellfun("isempty", self.demoTrajectories));
            nTraj = length(self.demoTrajectories);
            self.trajectoryDataset = nan(6, nSamplePoint, nTraj);
          
            for i = 1:nTraj
        
                time = double(self.demoTrajectories{i}(1, :));
                position = double(self.demoTrajectories{i}(2:4, :));
                velocity = double(self.demoTrajectories{i}(5:7, :));
        
                self.trajectoryDataset(:, :, i) = [interp1(time, position', linspace(time(1), time(end), nSamplePoint)), ...
                                                   interp1(time, velocity', linspace(time(1), time(end), nSamplePoint))]';
            end
            
            % Save dataset in case of app restart
            traj2save= self.trajectoryDataset ; 
            save('DemoDataset.mat', 'traj2save');
        end

        % Wrappers to the addShapes() methods
        function addSphere(self, position, radius,  rho, id)
    
            axes = [radius; radius ; radius];

            self.myDS.myWorld.addEllipsoid(axes, position, rho, id);

        end

        function addCylinder(self, position, radius, rho, velocity_function)

            if exist('rho', 'var') == false
              rho = 1;
            end
    
            if exist('velocity_function', 'var') == false
              velocity_function = @(x,t) [ 0; 0; 0 ];
            end 

            self.myDS.myWorld.addCylinder(radius, position, rho, velocity_function)
            obstacleNumber = size(self.myDS.myWorld.listOfObstacles, 2);
            obstacle = self.myDS.myWorld.listOfObstacles(obstacleNumber);
            self.rosInterface.sendObstacleInfo(obstacleNumber, obstacle);
        end

        function addPlane(self, position, normal,  rho, velocity_function)

            if exist('rho', 'var') == false
              rho = 1;
            end
    
            if exist('velocity_function', 'var') == false
              velocity_function = @(x,t) [ 0; 0; 0 ];
            end 
            
            self.myDS.myWorld.addPlane(normal, position, rho, velocity_function)
            obstacleNumber = size(self.myDS.myWorld.listOfObstacles, 2);
            obstacle = self.myDS.myWorld.listOfObstacles(obstacleNumber);
            self.rosInterface.sendObstacleInfo(obstacleNumber, obstacle);
        end

        function addPredefinedObstacle(self, type, position, rho, velocity_function)

            % Add position if undefined 
            if exist('position', 'var') == false
                position = [0,0,0];
            end

            if exist('rho', 'var') == false
              rho = 1;
            end

            if exist('velocity_function', 'var') == false
              velocity_function = @(x,t) [ 0; 0; 0 ];
            end 
            
            % There can only be one
            if self.predefExists
                kObst = length(self.myDS.myWorld.listOfObstacles);
                for k = 1:kObst
                    obstacle = self.myDS.myWorld.listOfObstacles(k);
                    if isprop(obstacle, 'obstacleType')
                        disp("Found predeef obstacle !!! " + string(k))
                        predef_index = k;
                    end
                end
                
                self.deleteObstacle(predef_index)
                fprintf("There can only be one predefined obstacle at a time, replacing previous with current one. \n")
            end
            % avoid issues when deleting
            if ~self.predefExists
                self.predefExists = true;
            end

            % Check correct type
            possible_types = ['line', 'ring', 'tshape', 'wall'];
            if ~ismember(type, possible_types)
                error('Invalid type: %s. Valid types are: line, ring, tshape, wall.', input_type);
            end
            
            self.myDS.myWorld.addPredefinedObstacle(type, position, rho, velocity_function);
            obstacleNumber = size(self.myDS.myWorld.listOfObstacles, 2);
            obstacle = self.myDS.myWorld.listOfObstacles(obstacleNumber);
            self.rosInterface.sendObstacleInfo(obstacleNumber, obstacle);

        end

        function createButtonsFigure1(self)
            % Create buttons arranged horizontally
            buttonWidth = 0.85; % Each button takes 12% of the figure width
            buttonHeight = 0.12; % Button height relative to figure
            buttonSpacing = 0.022; % Spacing between buttons
            buttonPlacement = (1-buttonWidth) /2; % left-side spacing of buttons

            inputWidth = 0.6;
            inputHeight = 0.10;
            inputSpacing = 0.03;
            inputPlacement = (1-inputWidth) /2; % left-side spacing of buttons

            toggleHeight = 0.7*buttonHeight;
            
            % Create Freeze Mode button
            self.btnFreezeMode = uicontrol(self.figHandle1,'Style', 'pushbutton', 'String', 'Freeze Mode',...
                'Units', 'normalized', 'Position', [buttonPlacement, buttonSpacing + 6 * (buttonHeight + buttonSpacing) - 0.3* buttonHeight, buttonWidth, buttonHeight],...
                'Callback', @(src, event) self.keyPressCallback('f'),...
                'BackgroundColor', [0.5 0.5 1], 'ForegroundColor', [0 0 0]);
            
            % Create Play button
            self.btnPlay = uicontrol(self.figHandle1,'Style', 'pushbutton', 'String', 'Start DS Control',...
                'Units', 'normalized', 'Position', [buttonPlacement, buttonSpacing + 5 * (buttonHeight + buttonSpacing) - 0.3* buttonHeight, buttonWidth, buttonHeight],...
                'Callback', @(src, event) self.keyPressCallback('p'),...
                'BackgroundColor', [0.4 1 0.4], 'ForegroundColor', [0 0 0]);
            
            % Create Stop button
            self.btnStop = uicontrol(self.figHandle1,'Style', 'pushbutton', 'String', 'Stop DS Control',...
                'Units', 'normalized', 'Position', [buttonPlacement, buttonSpacing + 4 * (buttonHeight + buttonSpacing) - 0.3* buttonHeight, buttonWidth, buttonHeight],...
                'Callback', @(src, event) self.keyPressCallback('s'),...
                'BackgroundColor', [1 0.4 0.4], 'ForegroundColor', [0 0 0]);
           
            % Create Reset button
            self.btnStop = uicontrol(self.figHandle1,'Style', 'pushbutton', 'String', 'Reset Robot',...
                'Units', 'normalized', 'Position', [buttonPlacement, buttonSpacing + 3 * (buttonHeight + buttonSpacing) - 0.3* buttonHeight, buttonWidth, buttonHeight],...
                'Callback', @(src, event) self.keyPressCallback('k'),...
                'BackgroundColor', [1 0.6 0.2], 'ForegroundColor', [0 0 0]);

            % Create Set Linear DS button
            self.btnStop = uicontrol(self.figHandle1,'Style', 'pushbutton', 'String', 'Set Linear DS Attractor',...
                'Units', 'normalized', 'Position', [buttonPlacement, buttonSpacing + (toggleHeight + buttonSpacing)+ (inputHeight + inputSpacing), buttonWidth, buttonHeight],...
                'Callback', @(src, event) self.keyPressCallback('a'),...
                'BackgroundColor', [1 1 0.4], 'ForegroundColor', [0 0 0]);

            % Read Linear DS attractor
            self.inputAttractor = uicontrol(self.figHandle1, 'Style', 'edit', 'String', '[0.0; 0.0; 0.0]',...
                'Units', 'normalized', 'Position', [inputPlacement, inputSpacing + (toggleHeight + buttonSpacing), inputWidth, inputHeight],...
                'BackgroundColor', [1 1 0.9], 'ForegroundColor', [0 0 0]); % Input field with default text

             % Create Set Linear DS button
            self.btnToggle = uicontrol(self.figHandle1,'Style', 'pushbutton', 'String', 'Activate modulation',...
                'Units', 'normalized', 'Position', [buttonPlacement, buttonSpacing, buttonWidth, toggleHeight],...
                'Callback', @(src, event) self.toggleButtonCallback,...
                'BackgroundColor', [0.2 0.8 0.2], 'ForegroundColor', [0 0 0]);
            
        end

        function createButtonsFigure2(self)
            % Create buttons arranged horizontally
            buttonWidth = 0.85; % Each button takes 12% of the figure width
            buttonHeight = 0.14; % Button height relative to figure
            buttonSpacing = 0.02; % Spacing between buttons
            buttonPlacement = (1-buttonWidth) /2; % left-side spacing of buttons

            dropdownWidth = 0.4;
            dropdownPlacement = (1-dropdownWidth) /2; % 
            
            % Create Start Recording button
            self.btnStartRecording = uicontrol(self.figHandle2,'Style', 'pushbutton', 'String', 'Start Recording',...
                'Units', 'normalized', 'Position', [buttonPlacement, buttonSpacing + 5 * (buttonHeight + buttonSpacing), buttonWidth, buttonHeight],...
                'Callback', @(src, event) self.keyPressCallback('r'),...
                'BackgroundColor', [0.6 1 0.6], 'ForegroundColor', [0 0 0]);
            
            % Create Stop Recording button
            self.btnStopRecording = uicontrol(self.figHandle2,'Style', 'pushbutton', 'String', 'Stop Recording',...
                'Units', 'normalized', 'Position', [buttonPlacement, buttonSpacing + 4 * (buttonHeight + buttonSpacing), buttonWidth, buttonHeight],...
                'Callback', @(src, event) self.keyPressCallback('s'),...
                'BackgroundColor', [1 0.6 0.6], 'ForegroundColor', [0 0 0]);
            
            % Create Learn button
            self.btnLearn = uicontrol(self.figHandle2,'Style', 'pushbutton', 'String', 'Learn DS',...
                'Units', 'normalized', 'Position', [buttonPlacement, buttonSpacing + 3 * (buttonHeight + buttonSpacing), buttonWidth, buttonHeight],...
                'Callback', @(src, event) self.keyPressCallback('l'),...
                'BackgroundColor', [1 1 0.6], 'ForegroundColor', [0 0 0]);

            % Create Delete Trajectory button
            self.btnDeleteTrajectory = uicontrol(self.figHandle2,'Style', 'pushbutton', 'String', 'Delete Last Trajectory',...
                'Units', 'normalized', 'Position', [buttonPlacement, buttonSpacing + 1 * (buttonHeight + buttonSpacing), buttonWidth, buttonHeight],...
                'Callback', @(src, event) self.keyPressCallback('d'),...
                'BackgroundColor', [1 0.8 0.4], 'ForegroundColor', [0 0 0]);
            
            % Create Clear Dataset button
            self.btnClearDataset = uicontrol(self.figHandle2, 'Style', 'pushbutton', 'String', 'Clear Dataset',...
                'Units', 'normalized', 'Position', [buttonPlacement, buttonSpacing + 0 * (buttonHeight + buttonSpacing), buttonWidth, buttonHeight],...
                'Callback', @(src, event) self.keyPressCallback('x'),...
                'BackgroundColor', [0.9 0.7 0.7], 'ForegroundColor', [0 0 0]);

            % Create a text label above the dropdown
            textAlgo = uicontrol(self.figHandle2, 'Style', 'text', 'String', 'Select Algorithm :', 'Units', 'normalized', ...
                'Position', [buttonPlacement, buttonSpacing + 2 * (buttonHeight + buttonSpacing) + 0.4*buttonHeight, buttonWidth, buttonHeight * 0.5], ...
                'HorizontalAlignment', 'center', 'BackgroundColor', self.figHandle2.Color, 'FontSize', 10);
            
                         % Create a dropdown menu (popup menu) in the figure
            self.dropdownAlgo = uicontrol(self.figHandle2, 'Style', 'popupmenu', 'String', {'SEDS', 'LPVDS'}, ...
                 'Units', 'normalized','Position', [dropdownPlacement, buttonSpacing + 2 * (buttonHeight + buttonSpacing), dropdownWidth, buttonHeight* 0.5], ...
                         'Callback', @(src, event) self.processDropdownAlgo); % Assign the callback
            
        end

        function processInputAttractor(self)
            % Function to read and process input from the inputField
            inputText = get(self.inputAttractor, 'String');
            try
                % Convert the input string to a numeric array
                inputArray = str2num(inputText); %#ok<ST2NM> 
                
                % Validate if the input is a column vector
                if isempty(inputArray) || size(inputArray, 2) ~= 1
                    error('Input must be a column vector in the format [0.0; 0.0; 0.0].');
                else
                    self.myDS.setLinearDS(inputArray) 
                    disp('New Attractor:');
                    disp(inputArray);
                end
            catch
                disp('Invalid input format. Please use the format [0.0; 0.0; 0.0].');
            end
        end

        function processDropdownAlgo(self)
            val = self.dropdownAlgo.Value;  % Get the selected value
            switch val
                case 1
                    disp('You selected SEDS algorthim');
                    self.myDS.algoName = 'SEDS';
                    % Add the code you want to execute for Option 1 here
                case 2
                    disp('You selected LPVDS algorithm');
                    self.myDS.algoName = 'LPVDS';
                    % Add the code you want to execute for Option 2 here
            end
        end

        function toggleButtonCallback(self, ~, ~)
            % Check the current state and switch between the two
            if strcmp(self.btnToggle.String, 'Activate modulation')
                set(self.btnToggle, 'String', 'Deactivate modulation', 'BackgroundColor', [0.8 0.2 0.2]);
                self.myDS.activateModulation;
            else
                set(self.btnToggle, 'String', 'Activate modulation', 'BackgroundColor', [0.2 0.8 0.2]);
                self.myDS.deactivateModulation;
            end
        end

        function keyPressCallback(self, keyPressed)
            % Same functionality as before, but triggered by buttons
            if strcmp(keyPressed,'r')==1
                disp('Started recording.')
                self.demoTrajectories{self.iTraj} = [];
                self.appState = 'Record'; 

            elseif strcmp(keyPressed,'s')==1 
                if strcmp(self.appState, 'Record')         
                    fprintf('Stopped recording. Number of trajectories : %i \n', self.iTraj)
                    self.formatTrajectories();
                    self.rosInterface.sendDemonstration(self.trajectoryDataset(1:3, : , end));
                    self.iTraj = self.iTraj + 1;
                else
                    disp('Stopped sending commands. Ready for next input.')
                end
                self.appState = 'Stop';   

            elseif strcmp(keyPressed,'d')==1 
                if strcmp(self.appState, 'Record')
                    warning('Cannot delete while recording.')
                elseif self.iTraj == 1
                    warning('No trajectory to delete.')
                else
                    self.iTraj = self.iTraj - 1;
                    self.demoTrajectories{self.iTraj} = [];
                    self.formatTrajectories();
                    self.rosInterface.sendDemonstration(zeros(3,1));
                    fprintf('Deleted last trajectory. Number of trajectories : %i \n', self.iTraj-1)
                end
                self.appState = 'Idle';   

            elseif strcmp(keyPressed,'l')==1
                disp('Initializing learning, please wait.')
                self.appState = 'Learn';

            elseif strcmp(keyPressed,'p')==1
                disp('Controlling robot using myDS.')
                self.appState = 'Play';

            elseif strcmp(keyPressed,'x')==1
                disp('Cleaning dataset.')
                self.appState = 'Stop';
                self.iTraj = 1;
                self.demoTrajectories = {};            
                self.trajectoryDataset = nan(6, 200, 0);
                self.rosInterface.sendDemonstration(zeros(3,2));
                disp('Finished cleaning dataset.')

            elseif strcmp(keyPressed,'f')==1
                disp('Activating Freeze mode.')
                disp('Only available when not commanding robot. Will automatically deactivate when pressing "P".')
                self.rosInterface.setFreezeMode(true);

            elseif strcmp(keyPressed,'k')==1
                disp('Resetting Robot.')
                self.rosInterface.sendResetRobot();

            elseif strcmp(keyPressed,'a')==1
                disp('Updating Linear DS attractor.')
                self.processInputAttractor();
            end  
        end
    end
end


function deleteCallback(~, ~, Hub)
      % Close everything when main figure is closed
        disp('Closing application.')
        Hub.appState = 'Stop';
        delete(Hub.figHandle1);
        delete(Hub.figHandle2);
        close all force;
        Hub.iTraj = 1;
        Hub.demoTrajectories = {};
        Hub.trajectoryDataset = [];
        Hub.myDS.myWorld.listOfObstacles = [];
        Hub.rosInterface.delete()   
end
