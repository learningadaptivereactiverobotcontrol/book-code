function [ S ] = compute_cov_sim( Sigmas, type )
% This function computes the similiarity value of N covariance matrices
% Input:    Sigmas (1xN or Nx1 cell containing Covariance Matrices)
%           type   (Type of similarity function to compute, default:
%           'KLDM')
%               'RIEM': Affine Invariant Riemannian Metric
%               'LERM': Log-Euclidean Riemannina Metric
%               'KLDM': Kullback-Liebler Divergence Metric
%               'JBLD': Jensen-Bregman LogDet Divergence
% Output:   S (NxN dimensional similarity matrix)

% Example: blah blah

% Author: Nadia Figueroa, PhD Student., Robotics
% Learning Algorithms and Systems Lab, EPFL (Switzerland)
% Email address: nadia.figueroafernandez@epfl.ch  
% Website: http://lasa.epfl.ch
% July 2016; Last revision: 


if isempty(type)
    type = 'KLDM';
end
N = length(Sigmas);
S = zeros (N,N);
D = size(Sigmas{1},1);

fprintf('Computing %s Similarity Function for %dx%d Covariance Matrices of %dx%d dimensions...\n',type,N,N,D,D);
for i=1:N
    for j=1:N
        
        X = Sigmas{i};
        Y = Sigmas{j};
        
        
        switch type
            case 'RIEM'
                % Affine Invariant Riemannian Metric
                S(i,j) = (norm(logm(X^(-1/2)*Y*X^(-1/2)),'fro')^2);
            
            case 'LERM'
                % Log-Euclidean Riemannian Metric
                S(i,j) = (norm(logm(X) - logm(Y),'fro')^2);
            
            case 'KLDM'
                % Kullback-Liebler Divergence Metric (KDLM)
                S(i,j) = 1/2*trace(X^-1*Y + Y^-1*X - 2*eye(size(X)));                
            
            case 'JBLD'
                % Jensen-Bregman LogDet Divergence
                S(i,j) = (logm(det((X+Y)/2)) - 1/2*logm(det(X*Y)));
        end
       
    end
end
fprintf('*************************************************************\n');


end

