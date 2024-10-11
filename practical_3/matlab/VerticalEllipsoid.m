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

classdef VerticalEllipsoid < Shapes
   properties
      ellipseAxes

   end

   methods
    
       function self = VerticalEllipsoid(axes, position, velocity_function, rho, id)

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


           self = self@Shapes(gamma, gammaDistance, gradientGamma, param, position, velocity_function, rho, id)

           self.ellipseAxes = axes;
       end

   end

end
