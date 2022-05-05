%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL,
%    Switzerland
%   Author: Walid Amanhoud, Sina Mirrazavi
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
%%%%yes%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    MIT Press book 
%    Learning for Adaptive and Reactive Robot Control
%    Chapter 11: Programming exercise 1 - Parts 1-4
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   TODO: 
%    To run the code, type on the command line 
%    motion_force_generations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function motion_force_generations()
close all; clc;
folder = fileparts(which('motion_force_generations.m'));
addpath(genpath(fullfile(folder,  '..', 'libraries', 'Motion_Force_Generation')));
addpath(genpath(fullfile(folder, '..', 'libraries', 'SVMGrad')));
addpath(genpath(fullfile(folder, '..', '..', 'libraries', 'book-ml-toolbox')));
addpath(genpath(fullfile(folder,  '..', '..', 'libraries', 'book-thirdparty')));
addpath(genpath(fullfile(folder,  '..', '..', 'libraries', 'book-robot-tools')));
modulated_ds_interface;