function [ X_train, y_train, X_test, y_test ] = split_data(X, y, tt_ratio )
%SPLIT_DATA Randomly partitions a dataset into train/test sets using
%   according to the given tt_ratio
%
%   input -----------------------------------------------------------------
%   
%       o X        : (N x M), a data set with M samples each being of dimension N.
%                           each column corresponds to a datapoint
%       o y        : (1 x M), a vector with labels y \in {0,1} corresponding to X.
%       o tt_ratio : train/test ratio.
%   output ----------------------------------------------------------------
%
%       o X_train  : (N x M_train), a data set with M_test samples each being of dimension N.
%                           each column corresponds to a datapoint
%       o y_train  : (1 x M_train), a vector with labels y \in {0,1} corresponding to X_train.
%       o X_test   : (N x M_test), a data set with M_test samples each being of dimension N.
%                           each column corresponds to a datapoint
%       o y_test   : (1 x M_test), a vector with labels y \in {0,1} corresponding to X_test.
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Auxiliary Variable
[~, M] = size(X);

% Randomize Dataset Indices
rand_idx  = randperm(M); 

% Compute expected number of Training/Testing Points
M_train    = round(M*tt_ratio);
M_test     = M - M_train;

% Split the dataset in train and test subsets
train_idx = rand_idx(1:M_train);
X_train   = X(:,train_idx);
y_train   = y(train_idx);

test_idx  = rand_idx(M_train+1:end);
X_test   = X(:,test_idx);
y_test   = y(test_idx);

end

