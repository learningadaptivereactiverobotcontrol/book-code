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

% Clean and import dependencies
clc; clear; close all;
filepath = fileparts(which('practical3_main.m'));
addpath(genpath(fullfile(filepath, 'matlab')));
addpath(genpath(fullfile(filepath, '..','matlab_exercises', 'libraries', 'book-robot-simulation')));
addpath(genpath(fullfile(filepath, '..','matlab_exercises', 'libraries', 'book-ds-opt')));
addpath(genpath(fullfile(filepath, '..','matlab_exercises', 'libraries', 'book-sods-opt')));
addpath(genpath(fullfile(filepath, '..','matlab_exercises', 'libraries', 'book-phys-gmm')));
addpath(genpath(fullfile(filepath, '..','matlab_exercises', 'libraries', 'book-thirdparty')));

% Obstacle Virtual Environment Class
myWorld = Environment();

% Learning DS and modulation class
myDS = DS('SEDS', myWorld); % Specify algo name here

% Central class to transfer information
myHub = mainHub(myDS);
