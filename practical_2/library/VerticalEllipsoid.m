classdef VerticalEllipsoid < Shapes
   properties
      ellipseAxes

   end

   methods
    
       function self = VerticalEllipsoid(axes, position, velocity_function, rho)

           param = [];
           param.ellipseAxes = axes;
           
           %% ------ Write your code below ------
           %  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %

           % Fill the gamma functions and the gradient
           % The parameter param.ellipseAxes is a 3D vector with the three
           % semi-axes of an ellipse

           gamma = @(x, y, z, param) 0 ;
           gradientGamma = @(x, y, z, param) [0; 0; 0];

           % To complete in TASK 4
           gammaDistance = @(x, y, z, param) 0;

           %  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
           %% ------ Write your code above ------


           self = self@Shapes(gamma, gammaDistance, gradientGamma, param, position, velocity_function, rho)

           self.ellipseAxes = axes;
       end

   end

end
