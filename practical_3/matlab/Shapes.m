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

classdef Shapes < matlab.mixin.Heterogeneous & handle
   % HierarchyRoot is a direct subclass of matlab.mixin.Heterogeneous.
   % HierarchyRoot is the root of this heterogeneous hierarchy.

   properties

       id
       position
       velocity
       velocityFunction

       gamma
       gammaDistance
       gradientGamma
       shapeParameters

       rho
       patchVal
       prevTime
   end

   methods

       function self = Shapes(gamma, gammaDistance, gradientGamma, parameters, position, velocityFunction, rho, id)
        
           if nargin > 0
              
              % Enforce column vector notation
              if size(position, 2) == 3
                position = position';
              end
                
              self.id = id;

              self.position = position;
              self.velocityFunction = velocityFunction;
              self.velocity = [0; 0; 0];

              self.shapeParameters = parameters;
              self.prevTime = 0;

              self.gamma = @(x, y, z) gamma(x-self.position(1), ...
                                            y-self.position(2), ...
                                            z-self.position(3), ...
                                            self.shapeParameters);
              self.gammaDistance = @(x, y, z) gammaDistance(x-self.position(1), ...
                                                            y-self.position(2), ...
                                                            z-self.position(3), ...
                                                            self.shapeParameters);

              self.gradientGamma = @(x, y, z) gradientGamma(x-self.position(1), ...
                                                            y-self.position(2), ...
                                                            z-self.position(3), ...
                                                            self.shapeParameters);

              if exist('rho', 'var') == true
                  self.rho = rho;
              end


           end
        end

        
        function showShape(self, axisLimit, ax)
            
            alpha = 0.2;


            nPoints = 50;
            [x,y,z] = meshgrid(linspace(axisLimit(1), axisLimit(2), nPoints), ...
                               linspace(axisLimit(3), axisLimit(4), nPoints), ...
                               linspace(axisLimit(5), axisLimit(6), nPoints)); 
            
            s = isosurface(x, y, z, self.gamma(x, y, z), 1);
            self.patchVal = patch(ax, s, 'FaceColor', rand(1, 3), 'FaceAlpha', alpha, 'EdgeColor', 'none');

        end

        function moveShape(self, t)
              
            % Compute delta_t and update prevTime
            delta_t = t - self.prevTime;
            self.prevTime = t;
            
            % % Update position and velocity
            new_velocity = self.velocityFunction(self.position, t);
            % new_position = self.position + new_velocity * delta_t ;
            % self.position = new_position;
            % self.velocity = new_velocity;
            
            % Straigh up pos function
            self.position = self.position + self.velocityFunction(self.position, t);


            % Update display
            vertices = get(self.patchVal, 'Vertices');
            delta_position = new_velocity *delta_t;
            delta_p_reshaped = repmat(delta_position', size(vertices,1), 1 );
            vertices = vertices + delta_p_reshaped;
            set(self.patchVal, 'Vertices', vertices);
        end
   
   end
end