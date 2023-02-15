function [ V, L, Mu ] = my_pca( X )
%MY_PCA Step-by-step implementation of Principal Component Analysis
%   In this function, the student should implement the Principal Component 
%   Algorithm following Eq.1, 2 and 3 of Assignment 1.
%
%   input -----------------------------------------------------------------
%   
%       o X      : (N x M), a data set with M samples each being of dimension N.
%                           each column corresponds to a datapoint
%
%   output ----------------------------------------------------------------
%
%       o V      : (M x M), Eigenvectors of Covariance Matrix.
%       o L      : (M x M), Eigenvalues of Covariance Matrix
%       o Mu     : (N x 1), Mean Vector of Dataset

% Auxiliary variables
[N, M] = size(X);

% Output variables
V  = zeros(N,N);
L  = zeros(N,N);
Mu = zeros(N,1);

% ====================== Implement Eq. 1 Here ====================== 
if M > 1    
    Mu = mean(X,2);
else
    Mu = mean(X,1);
end
% Option 1: repmat
% X_mean = X - repmat(Mu, 1, size(X, 2));

% or

% Option 2: bsxfun
X = bsxfun(@minus, X, Mu);


% ====================== Implement Eq.2 Here ======================
if M > 1
    C = (1/(M-1))*X*X';
else
    C = (1/(M))*X*X';
end
% C = cov(X');


% ====================== Implement Eq.3 Here ======================

% Option 1: eig()
[V,L] = eig(C);

% or

% Option 2: svd()
% [V,L,U] = svd(C);


% =================== Sort Eigenvectors wrt. EigenValues ==========
% Sort Eigenvalue and get indices
[L_sort, ind] = sort(diag(L),'descend');

% arrange the columns in this order
V=V(:,ind); 

% Vectorize sorted eigenvalues
L = diag(L_sort); 

end

