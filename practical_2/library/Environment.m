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

classdef Environment < handle
   properties
      listOfObstacles
      
      figure
      axis
      axisLimit

      robot
      endEffectorRadius = 0.1
      endEffectorPosition
   end

   methods
    
       function self = Environment(robotPosition)

        filepath = fileparts(which('Environment.m'));

        addpath(genpath(fullfile(filepath, '..', '..', 'libraries', 'book-robot-simulation')));
        addpath(genpath(fullfile(filepath, '..', '..', 'libraries', 'book-thirdparty')));
        addpath(genpath(fullfile(filepath, '..', '..', 'libraries', 'book-ds-opt')));
        addpath(genpath(fullfile(filepath, '..', '..', 'libraries', 'book-phys-gmm')));

        self.listOfObstacles = [];
        
        self.figure = figure(1);
        screensize = get(groot, 'Screensize');
        self.figure.Position = [0.4 * screensize(3), 0.1 * screensize(4), 0.6 * screensize(3), 0.8 * screensize(4)];
        self.axis = axes;

        self.robot = PandaWrapper();
        show(self.robot.robot, zeros(7, 1), 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');

        view([45, 45])
        axis manual; hold on;
        self.axisLimit = [-0.7, 0.9, -0.8, 0.8, -0.3, 1.2];
        axis(self.axisLimit)

        self.endEffectorPosition = robotPosition;
    end

    function self = showGammaIsosurfaces(self, isoValues, useDistance, color)

        if exist('color', 'var') == false
            color = [0.2, 0, 0.8];
        end

        if exist('useDistance', 'var') == false
            useDistance = false;
        end

        nPoints = 50;
        [x,y,z] = meshgrid(linspace(self.axisLimit(1), self.axisLimit(2), nPoints), ...
                           linspace(self.axisLimit(3), self.axisLimit(4), nPoints), ...
                           linspace(self.axisLimit(5), self.axisLimit(6), nPoints)); 
        
        for obstacle = self.listOfObstacles

            for i=1:length(isoValues)
                value = isoValues(i);
                if useDistance
                    s = isosurface(x, y, z, obstacle.gammaDistance(x, y, z), value);
                else
                    s = isosurface(x, y, z, obstacle.gamma(x, y, z), value);
                end
                alpha = 0.1 * (1.1-i/length(isoValues));
                patch(s, 'FaceColor', color, 'FaceAlpha', alpha, 'EdgeColor', 'none')
            end
        end
    
    end

    function self = addEllipsoid(self, axes, position, rho, velocity_function)

        if exist('rho', 'var') == false
          rho = 1;
        end

        if exist('velocity_function', 'var') == false
          velocity_function = @(x,t) [ 0; 0; 0 ];
        end

        newEllipse = VerticalEllipsoid(axes, position, velocity_function, rho);

        self.listOfObstacles = [self.listOfObstacles, newEllipse];
        
        % If position is inside obstacle, we count it as a boundary
        if newEllipse.gamma(self.endEffectorPosition(1), self.endEffectorPosition(2), self.endEffectorPosition(3)) > 1 
            alpha = 0.7;
        else 
            alpha = 0.1;
        end

        newEllipse.showShape(self.axisLimit, alpha);

    end

    function self = addCylinder(self, radius, position, rho, velocity_function)

        if exist('rho', 'var') == false
          rho = 1;
        end

        if exist('velocity_function', 'var') == false
          velocity_function = @(x,t) [ 0; 0 ;0 ];
        end

        newCylinder = VerticalCylinder(radius, position, velocity_function, rho);

        self.listOfObstacles = [self.listOfObstacles, newCylinder];

        % If position is inside obstacle, we count it as a boundary
        if newCylinder.gamma(self.endEffectorPosition(1), self.endEffectorPosition(2), self.endEffectorPosition(3)) > 1             
            alpha = 0.5;
        else 
            alpha = 0.1;
        end

        newCylinder.showShape(self.axisLimit, alpha);

    end

    function self = addPlane(self, normal, position, rho, velocity_function)

        if exist('rho', 'var') == false
          rho = 1;
        end

        if exist('velocity_function', 'var') == false
          velocity_function = @(x,t) [ 0; 0; 0 ];
        end

        newPlane = Plane(normal, position, velocity_function, rho);

        self.listOfObstacles = [self.listOfObstacles, newPlane];

        newPlane.showShape(self.axisLimit, 0.2);

    end

    function self = addShape(self, gamma, gammaDistance, gradientGamma, parameters, position, velocity_function)

        if exist('rho', 'var') == false
          rho = 1;
        end

        if exist('velocity_function', 'var') == false
          velocity_function = @(x,t) [ 0; 0; 0 ];
        end

        newShape = Shapes(gamma, gammaDistance, gradientGamma, parameters, position, velocity_function, rho);

        self.listOfObstacles = [self.listOfObstacles, newShape];

        % If position is inside obstacle, we count it as a boundary
        if newShape.gamma(self.endEffectorPosition(1), self.endEffectorPosition(2), self.endEffectorPosition(3)) > 1    
            alpha = 0.5;
        else 
            alpha = 0.1;
        end

        newShape.showShape(self.axisLimit, alpha);

    end

   end

end
