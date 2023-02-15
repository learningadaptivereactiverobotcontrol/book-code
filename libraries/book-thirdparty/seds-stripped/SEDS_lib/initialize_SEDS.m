function [Priors_0, Mu_0, Sigma_0] = initialize_SEDS(Data,K,varargin)
%
% This function finds a safe initial guess that will be used for SEDS
% optimization. The function can be called using:
%
%           [Priors_0, Mu_0, Sigma_0] = initialize_SEDS(Data,K)
%
% or
%       	[Priors_0, Mu_0, Sigma_0] = initialize_SEDS(Data,K,options)
%
% to also pass a structure of desired options.
%
% Inputs -----------------------------------------------------------------
%
%   o Data:    A 2d x N_Total matrix containing all demonstration data points.
%              Rows 1:d corresponds to trajectories and the rows d+1:2d
%              are their first time derivatives. Each column of Data stands
%              for a datapoint. All demonstrations are put next to each other 
%              along the second dimension. For example, if we have 3 demos
%              D1, D2, and D3, then the matrix Data is:
%                               Data = [[D1] [D2] [D3]]
%
%   o K:       1 x K array representing the prior probabilities of the K GMM 
%
%   o options: A structure to set the optional parameters of the solver.
%              Providing 'options' for initialization forces the function
%              to compute a nice initial guess for the main optimization.
%              It is highly recommended to call the function with this
%              variable. Please type
%
%                               doc SEDS_Solver
%
%              in MATLAB command window to get a list of available
%              parameters that can be passed to the solver via 'options'.
%
% Outputs ----------------------------------------------------------------
%
%   o Priors_0:  1 x K array representing the prior probabilities of the K GMM 
%                components.
%
%   o Mu_0:      2d x K array representing the centers of the K GMM components.
%
%   o Sigma_0:   2d x 2d x K array representing the covariance matrices of the 
%                K GMM components.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    Copyright (c) 2010 S. Mohammad Khansari-Zadeh, LASA Lab, EPFL,   %%%
%%%          CH-1015 Lausanne, Switzerland, http://lasa.epfl.ch         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% The program is free for non-commercial academic use. Please contact the
% author if you are interested in using the software for commercial purposes.
% The software must not be modified or distributed without prior permission
% of the authors. Please acknowledge the authors in any academic publications
% that have made use of this code or part of it. Please use this BibTex
% reference:
% 
% S. M. Khansari-Zadeh and A. Billard, "Learning Stable Non-Linear Dynamical 
% Systems with Gaussian Mixture Models", IEEE Transaction on Robotics, 2011.
%
% To get latest upadate of the software please visit
%                          http://lasa.epfl.ch/khansari
%
% Please send your feedbacks or questions to:
%                           mohammad.khansari_at_epfl.ch

%%
d = size(Data,1)/2; %number of dimensions

fprintf('\nStarting initialization ...\n')
%initializing with EM
[Priors_0, Mu_0, Sigma_0] = EM_init_kmeans_SEDS(Data, K);
[Priors_0, Mu_0, Sigma_0] = EM(Data, Priors_0, Mu_0, Sigma_0);

if isempty(varargin)
    % deforming covariance fucntion such that they always satisfy the stability
    % conditions
    Sigma_tmp = Sigma_0;
    for k=1:K
        Sigma_0(:,:,k) = diag(diag(Sigma_tmp(:,:,k)));
        Sigma_0(1:d,d+1:end,k) = -diag(abs(diag(Sigma_tmp(d+1:2*d,1:d,k))));
        Sigma_0(d+1:end,1:d,k) = -diag(abs(diag(Sigma_tmp(d+1:2*d,1:d,k))));
    end
else
    % Initialize the parameters of Gaussian function with the SEDS solver 
    options = varargin{1};
    for k=1:K
        Pxi(:,k) = Priors_0(k).*gaussPDF(Data, Mu_0(:,k), Sigma_0(:,:,k));
    end
    [~ , ind] = max(Pxi,[],2);
    options.perior_opt = 0;
    options.display = 0;
    options.normalization = 0;
    for k=1:K
        [~, Mu_0(:,k) Sigma_0(:,:,k)]=SEDS_Solver(Priors_0(k),Mu_0(:,k),Sigma_0(:,:,k),Data(:,ind==k),options);
    end
end
fprintf('Initialization finished successfully.\n')