classdef Plane < Shapes
   properties
      normal

   end

   methods
    
       function self = Plane(normal, position, velocity_function, rho)

           param = [];
           param.normal = normal/norm(normal);
           
           % abs() is needed to have Gamma value > 1: allow working with
           % obstacles and boundaries
           gammaDistance = @(x, y, z, param) abs(param.normal(1)*x + param.normal(2)*y + param.normal(3)*z) + 1;

           gradientGamma = @(x, y, z, param) [ones(size(x))*param.normal(1); 
                                              ones(size(y))*param.normal(2);  
                                              ones(size(z))*param.normal(3)];
           
           % Classic formula: gamma between 0 and infinity, works better
           % for display
           gamma = @(x, y, z, param) param.normal(1)*x + param.normal(2)*y + param.normal(3)*z + 1;

           self = self@Shapes(gamma, gammaDistance, gradientGamma, param, position, velocity_function, rho)

           self.normal = normal;
       end
      
         % Overwrite parent showShape() function for better perfomance.
         % !! Doesn't Work for vertical planes !!
%        function showShape(self, axisLimit, alpha)
%             
%             % Generate vertex position from workspace limits
%             x = [axisLimit(1) axisLimit(2) axisLimit(2) axisLimit(1)]; 
%             y = [axisLimit(4) axisLimit(4) axisLimit(3) axisLimit(3)]; 
%             z = (-1/self.normal(3))*(self.normal(1)*(x - self.position(1)) + self.normal(2)*(y - self.position(2)) - self.position(3)); 
%             self.patchVal = patch(x, y, z, rand(1, 3), 'FaceAlpha', alpha);
% 
%        end

   end

end
