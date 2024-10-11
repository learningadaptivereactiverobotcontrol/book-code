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
      
      ax
      axisLimit

      endEffectorRadius = 0.0
   end

   methods
    
       function self = Environment()

        filepath = fileparts(which('Environment.m'));

        addpath(genpath(fullfile(filepath, '..', '..', 'libraries', 'book-robot-simulation')));
        addpath(genpath(fullfile(filepath, '..', '..', 'libraries', 'book-thirdparty')));
        addpath(genpath(fullfile(filepath, '..', '..', 'libraries', 'book-ds-opt')));
        addpath(genpath(fullfile(filepath, '..', '..', 'libraries', 'book-phys-gmm')));

        self.listOfObstacles = [];

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

    function self = addEllipsoid(self, axes, position, rho, id, velocity_function)

        if exist('rho', 'var') == false
          rho = 1;
        end

        if exist('velocity_function', 'var') == false
          velocity_function = @(x,t) [ 0; 0; 0 ];
        end

        newEllipse = VerticalEllipsoid(axes, position, velocity_function, rho, id);

        self.listOfObstacles = [self.listOfObstacles, newEllipse];

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

        newCylinder.showShape(self.axisLimit,  self.ax);

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

        newPlane.showShape(self.axisLimit, self.ax);

    end

    function self = addPredefinedObstacle(self, type, position, rho, velocity_function)

        if exist('rho', 'var') == false
          rho = 1;
        end

        if exist('velocity_function', 'var') == false
          velocity_function = @(x,t) [ 0; 0; 0 ];
        end

        if exist('position', 'var') == false
          position = [ 0; 0; 0 ];
        end

        newPredefinedObstacle = PredefinedObstacle(type, position, rho, velocity_function);

        self.listOfObstacles = [self.listOfObstacles, newPredefinedObstacle];

        % newPredefinedObstacle.showShape(self.axisLimit, self.ax);

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

        newShape.showShape(self.axisLimit,  self.ax);

    end

   end

end
