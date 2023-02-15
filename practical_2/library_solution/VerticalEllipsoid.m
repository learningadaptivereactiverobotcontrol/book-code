classdef VerticalEllipsoid < Shapes
   properties
      ellipseAxes

   end

   methods
    
       function self = VerticalEllipsoid(axes, position, velocity_function, rho)

           param = [];
           param.ellipseAxes = axes;

           % Classic formula: gamma between 0 and infinity, works better
           % for display
           gamma = @(x, y, z, param) sqrt((x/param.ellipseAxes(1)).^2 + (y/param.ellipseAxes(2)).^2 + (z/param.ellipseAxes(3)).^2) ;

           % abs() is needed to have Gamma value > 1: allow working with
           % obstacles and boundaries
           gammaDistance = @(x, y, z, param) 1 + abs(sqrt(x.^2 + y.^2 + z.^2) - sqrt((x.^2 + y.^2 + z.^2)./( (x/param.ellipseAxes(1)).^2 + (y/param.ellipseAxes(2)).^2 + (z/param.ellipseAxes(3)).^2 )));

           gradientGamma = @(x, y, z, param) [2*x / param.ellipseAxes(1)^2; ...
                                              2*y / param.ellipseAxes(2)^2; ...
                                              2*z / param.ellipseAxes(3)^2];


           self = self@Shapes(gamma, gammaDistance, gradientGamma, param, position, velocity_function, rho)

           self.ellipseAxes = axes;
       end

   end

end
