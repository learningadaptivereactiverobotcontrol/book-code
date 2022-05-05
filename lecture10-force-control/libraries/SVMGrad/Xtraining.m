%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Implementation of X-training for Collision Avoidance Dataset
% 1.2 million points of training data/1.2 million for testing
% This script needs ML_toolbox
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; clc; close all;
%% Load full dataset (Train and Test)
load('./models/Full-Collision-Avoidance-Dataset.mat')
X_train = X; y_train = y;
clear X y

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Create subsets of K randomly drawn examples and train SVMs on each
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
K       = 10;
K_size  = round(length(y_train)/10);
xtrain_models  = {};

clear options
% Optimal Values from CV on Xk dataset
options.svm_type    = 0;      % 0: C-SVM, 1: nu-SVM
options.C           = 223;    % Misclassification Penalty
options.sigma       = 0.633;  % radial basis function: exp(-gamma*|u-v|^2), gamma = 1/(2*sigma^2)

for i=1:K
    subset_start = (i-1)*K_size + 1;
    subset_end   = i*K_size;
    X_train_ = X_train(subset_start:subset_end,:);
    y_train_ = y_train(subset_start:subset_end,:);   
    
    % Train SVM Classifier (12k+3D pts = 8s,....)
    tic;
    [y_est, model] = svm_classifier(X_train_, y_train_, options, []);
    toc;

    xtrain_models{i,1} = model;   
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  For each training example estimate margin mean and variance
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
full_training_set = K*K_size;
mu_train  = zeros(full_training_set,1);
var_train = zeros(full_training_set,1);
keep_idx = [];

tic;
for i=1:full_training_set    
    query_point = X_train(i,:);
    fprintf('\nEvaluating %d-th point: ',i);
    margin = zeros(1,length(xtrain_models));    
    for k=1:length(xtrain_models)
        fprintf('.');
        % Extract k-th model
        model = xtrain_models{k};                
        [~,~, ~, Gamma] = evalc('svmpredict(ones(size(query_point(:,1))), query_point, model)');
        margin(1,k) = y_train(i,1)*Gamma;
    end
    
    % Mean and Variance of Margin estimations
    mu_i = mean(margin);
    var_i = var(margin);
    
    % Mean and Variance of Margin estimations
    if (mu_i + var_i < 0 ) || (mu_i - var_i > 1)
        ;
    else
        keep_idx = [keep_idx  i];
    end    
end
fprintf('\n');
toc;
fprintf('\n%d training points from %d after X-training with %d-base models.\n', length(keep_idx), full_training_set, K);

%% Find range for rbf kernel
pairwise_distances = zeros(1,length(X_train_keep));
for j = 1:length(X_train_keep)
    pairwise_distances(j) = norm(X_train_keep(1,:) - X_train_keep(j,:));
end
[sorted_distances, sorted_index] = sort(pairwise_distances);

% Visualize pairwise distances
figure('Color',[1 1 1])
histfit(sorted_distances)
title('36D f(q) Pairwise Distances')
xlabel('L_2 Norm')
grid on 
axis tight

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%   Grid-search on CV to find 'optimal' hyper-parameters for C-SVM with RBF %
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Kept Data from dataset editing step in X-training
ids = 1:full_training_set;
X_train_keep = X_train(ids(keep_idx),:);
y_train_keep = y_train(ids(keep_idx),:);

%  Set options for SVM Grid Search and Execute
clear options
options.svm_type   = 0;           % SVM Type (0:C-SVM, 1:nu-SVM)
options.limits_C   = [1, 10000];  % Limits of penalty C
options.limits_w   = [0.2, 2];    % Limits of kernel width \sigma
options.steps      = 10;          % Step of parameter grid 
options.K          = 10;          % K-fold CV parameter

%% Do Grid Search
[ ctest , ctrain , cranges ] = ml_grid_search_class( X_train_keep, y_train_keep, options );

%% Get CV statistics

% Extract parameter ranges
range_C  = cranges(1,:);
range_w  = cranges(2,:);

% Extract parameter ranges
stats = ml_get_cv_grid_states(ctest,ctrain);

%% Visualize Grid-Search Heatmap
cv_plot_options              = [];
cv_plot_options.title        = strcat('36-D, 3.8k --Joint Positions f(q)-- C-SVM :: Grid Search with RBF');
cv_plot_options.param_names  = {'C', '\sigma'};
cv_plot_options.param_ranges = [range_C ; range_w];

if exist('hcv','var') && isvalid(hcv), delete(hcv);end
hcv = ml_plot_cv_grid_states(stats,cv_plot_options);

% Find 'optimal hyper-parameters'
[max_acc,ind] = max(stats.train.acc.mean(:));
[C_max, w_max] = ind2sub(size(stats.train.acc.mean),ind);
C_opt = range_C(C_max);
w_opt = range_w(w_max);


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     Learn Optimal C - SUPPORT VECTOR MACHINE  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Test C-SVM on Data (Assuming you ran CV first)
clear options
% Optimal Values from CV on Xk dataset
options.svm_type    = 0;      % 0: C-SVM, 1: nu-SVM
options.C           = 2223;    % Misclassification Penalty
options.sigma       = 0.8;  % radial basis function: exp(-gamma*|u-v|^2), gamma = 1/(2*sigma^2)

% Train SVM Classifier (12k+3D pts = 8s,....)
tic;
[y_est, model] = svm_classifier(X_train, y_train, options, []);
toc;


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     Evaluate SVM performance on Testing Set   %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Extract a random testing points 
numSamples = 1000;
y_test_   = zeros(1,numSamples);
y_est_    = zeros(1,numSamples);

% Evaluate on Testing Points
for i=1:numSamples
    idx_rand  = randperm(numSamples);
    
    X_test_    = X_test(idx_rand(i),:);
    y_test_(i) = y_test(idx_rand(i),:);
    
    % Test Learnt Model
    [y_est_(i)] = svm_classifier(X_test_, y_test_(i), [], model);
end

% Compute Classifier Test Stats
[test_stats] = class_performance(y_test_,y_est_); 
fprintf('*Classifier Performance on Test Set (%d points)* \n Acc: %1.5f, F-1: %1.5f, FPR: %1.5f, TPR: %1.5f \n', length(y_test_), test_stats.ACC, test_stats.F1, test_stats.FPR, test_stats.TPR)


