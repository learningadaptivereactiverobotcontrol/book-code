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
