classdef Shapes < matlab.mixin.Heterogeneous & handle
   % HierarchyRoot is a direct subclass of matlab.mixin.Heterogeneous.
   % HierarchyRoot is the root of this heterogeneous hierarchy.

   properties

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

       function self = Shapes(gamma, gammaDistance, gradientGamma, parameters, position, velocityFunction, rho)
        
           if nargin > 0
              
              % Enforce column vector notation
              if size(position, 2) == 3
                position = position';
              end

              self.position = position;
              self.velocityFunction = velocityFunction;
              self.velocity = [0; 0; 0];

              self.shapeParameters = parameters;
              self.prevTime = 0;

              self.gamma = @(x, y, z) gamma(x-self.position(1), ...
                                            y-self.position(2), ...
                                            z-self.position(3), ...
                                            parameters);
              self.gammaDistance = @(x, y, z) gammaDistance(x-self.position(1), ...
                                                            y-self.position(2), ...
                                                            z-self.position(3), ...
                                                            parameters);

              self.gradientGamma = @(x, y, z) gradientGamma(x-self.position(1), ...
                                                            y-self.position(2), ...
                                                            z-self.position(3), ...
                                                            parameters);

              if exist('rho', 'var') == true
                  self.rho = rho;
              end


           end
        end

        
        function showShape(self, axisLimit, alpha)
            
            if exist('alpha', 'var') == false
                  alpha = 0.2;
            end

            nPoints = 50;
            [x,y,z] = meshgrid(linspace(axisLimit(1), axisLimit(2), nPoints), ...
                               linspace(axisLimit(3), axisLimit(4), nPoints), ...
                               linspace(axisLimit(5), axisLimit(6), nPoints)); 
            
            s = isosurface(x, y, z, self.gamma(x, y, z), 1);
            self.patchVal = patch(s, 'FaceColor', rand(1, 3), 'FaceAlpha', alpha, 'EdgeColor', 'none');

        end

        function moveShape(self, t)
              
            % Compute delta_t and update prevTime
            delta_t = t - self.prevTime;
            self.prevTime = t;
            
            % Update position and velocity
            new_velocity = self.velocityFunction(self.position, t);
            new_position = self.position + new_velocity * delta_t ;
            self.position = new_position;
            self.velocity = new_velocity;

            % Update display
            vertices = get(self.patchVal, 'Vertices');
            delta_position = new_velocity *delta_t;
            delta_p_reshaped = repmat(delta_position', size(vertices,1), 1 );
            vertices = vertices + delta_p_reshaped;
            set(self.patchVal, 'Vertices', vertices);
        end
   
   end
end