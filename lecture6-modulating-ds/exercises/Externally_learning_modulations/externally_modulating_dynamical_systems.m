%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL,
%    Switzerland
%   Author: Sina Mirrazavi
%   email:   sina.mirrazavi@epfl.ch
%   website: lasa.epfl.ch
%
%   Permission is granted to copy, distribute, and/or modify this program
%   under the terms of the GNU General Public License, version 2 or any
%   later version published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful, but
%   WITHOUT ANY WARRANTY; without even the implied warranty of
%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  Public License for more details
%  Name of the chapter: Modulate dynamical systems
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Example: externally_modulating_dynamical_systems
%           Dx=M(x, phi_c,h_s(s), Influence)*A*(x-Target)     x(0)=Initial
%   input -----------------------------------------------------------------
%
%       o Target      : (2 x 1), The target Position.  (Optional)
%       o Influence   : (1 x 1), The influence of the modulation function.  
%                               (Optional)
%       o Phi_c       : (1 x 1), The rotation of the modulation function.  
%                               (Optional)
%       o A           : (2 x 2), The linear nominal system.   (Optional)
%       o Upper       : (1 x 1), The upper bound of the external modulation.   
%                                Must be positive definite. (Optional)
%       o r         : (1 x 1), The rate of the external ativation funtion. 
%                                Muste be greater than 2   (Optional)
%  How to run -------------------------------------------------------------
%    externally_modulating_dynamical_systems()
%    externally_modulating_dynamical_systems([0;0])
%    externally_modulating_dynamical_systems([0;0], 1)
%    externally_modulating_dynamical_systems([0;0], 1, 1.57)
%    externally_modulating_dynamical_systems([2;2], 1, 1.57, -4*eye(2))
%    externally_modulating_dynamical_systems([2;2], 1, 1.57, -4*eye(2), 10)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function externally_modulating_dynamical_systems(Target, Influence, ...
                                                 Phi_c, A, u, r)
close all; clc;
folder = fileparts(which('externally_modulating_dynamical_systems.m'));
addpath(genpath(fullfile(folder, '..', '..', 'libraries', 'gpml')));
addpath(genpath(fullfile(folder, '..', '..', 'libraries', 'Externally_learning_modulations')));

if ((exist('Target')==0))
    % The target position of the DS
    Target = [0;0];
else
    if all(size(Target)~=[2,1])
        disp('The dimension of the target is not right')
        return;
    end
end


if ((exist('Influence')==0))
    % The influence of the modulation function
    Influence=1;
else
    if all(size(Influence)~=[1,1])
        disp('The dimension of Influence is not right')
        return;
    end
    if Influence<0
        disp('Influence should be positive.')
        return;
    end
end

if ((exist('Phi_c')==0))
    % Compute Random Rotation Angle phi_c
    a = 0; b =3.14;
    Phi_c = a+(b-a)*rand(1,1);
else
    if all(size(Phi_c)~=[1,1])
        disp('The dimension of phi_c is not right')
        return;
    end
    if Phi_c>6.2832
        disp('phi_c should be between 0 and 2\pi')
        return;
    end
    if Phi_c<0
        disp('phi_c should be between 0 and 2\pi')
        return;
    end
end

if ((exist('A')==0))
    % nominal dynamical system
    A=-4*eye(2);
else
    if all(size(A)~=[2,2])
        disp('The dimension of A is not right')
        return;
    end
end

if ((exist('u','var')==0))
    % nominal dynamical system
    u=5;
else
    if u<=0
        disp('upper must be positive definite')
        return;
    end
end

if ((exist('r','var') == 0))
    % nominal dynamical system
    r = 5;
else
    if r<=2
        disp('r must be more than two')
        return;
    end
end

% Base Offset
base = [-1 1]';
% Axis limits
% limits = [-Upper+Target(1) Upper+Target(1) -Upper+Target(2) Upper+Target(2)];
limits = [Target(1)-5 Target(1)+5 Target(2)-5 Target(2)+5];
simulate_lmds(Target, limits, Influence, Phi_c, A, u, r);

