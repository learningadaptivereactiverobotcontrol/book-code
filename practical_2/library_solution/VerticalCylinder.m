classdef VerticalCylinder < Shapes
   properties
      radius

   end

   methods
    
       function self = VerticalCylinder(radius, position, velocity_function, rho)

           param = [];
           param.radius = radius;

           % Classic formula: gamma between 0 and infinity, works better
           % for display
           gamma = @(x, y, z, param) sqrt(x.^2  + y.^2) / param.radius;

           % abs() is needed to have Gamma value > 1: allow working with
           % obstacles and boundaries
           gammaDistance = @(x, y, z, param) abs(sqrt(x.^2  + y.^2) - param.radius) + 1;

           gradientGamma = @(x, y, z, param) [2*x; 2*y; zeros(size(z))];


           self = self@Shapes(gamma, gammaDistance, gradientGamma, param, position, velocity_function, rho)

           self.radius = radius;

       end

   end

end
