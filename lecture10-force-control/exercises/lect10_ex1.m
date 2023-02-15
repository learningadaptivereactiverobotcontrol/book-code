%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL,
%    Switzerland
%   Author: Nadia Figueroa, Sina Mirrazavi
%   email:   sina.mirrazavi@epfl.ch
%   website: lasa.epfl.ch
%    
%   Permission is granted to copy, distribute, and/or modify this program
%   under the terms of the GNU General Public License, version 2 or any
%   later version published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful, but
%   WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%   Public License for more details
%  Name of the chapter:  Dynamical system based compliant control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Example: passive_ds
%   input -----------------------------------------------------------------
%       o Damping_Gains      : (2 x 2), The damping gains.  (Optional)
%  How to run -------------------------------------------------------------
%    lect10_ex1([0.1 0;0 0.1])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function lect10_ex1(Damping_Gains)
close all; clc;
folder = fileparts(which('lect10_ex1.m'));
cd(folder);
addpath(genpath(fullfile(folder, '..', 'libraries', 'Passive_Dynamical_system')));
addpath(genpath(fullfile(folder, '..', '..', 'libraries', 'book-ds-opt')));
addpath(genpath(fullfile(folder, '..', '..', 'libraries', 'book-phys-gmm')));
addpath(genpath(fullfile(folder, '..', '..', 'libraries', 'book-thirdparty')));

if ((exist('Damping_Gains')==0))
    % nominal dynamical system
    Damping_Gains=[0.1 0;0 0.1];
else
    if all(size(Damping_Gains)~=[2,2])
        disp('The dimension of Damping_Gains is not right.')
        return;
    end
    if (Damping_Gains(1,2)~=0)||(Damping_Gains(2,1)~=0)
        disp('Damping_Gains should be diagonal.')
        return;
    end
    if (Damping_Gains(1,1)<=0)||(Damping_Gains(2,2)<=0)
        disp('Damping_Gains should be positive definite.')
        return;
    end
end

gui_lpvDS(Damping_Gains);