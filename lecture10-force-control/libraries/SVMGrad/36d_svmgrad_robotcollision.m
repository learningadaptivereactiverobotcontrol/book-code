%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SVMgrad is a compact library used to evaluate the decision function of a
% Gaussian RBF Kernel Support Vector Machine, as well as the its first and
% Second Derivative.
%
%          y = sign(Gamma(x))
%          Gamma   = \sum_{i=1}^{N_sv}\alpha_iy_ik(x,x_i) + b 
%          DGamma  = \sum_{i=1}^{N_sv}-1/2\sigma^2\alpha_iy_ik(x,x_i)(x-x_i)
%          DDGamma = ...
% 
%  The evaluated SVM model can be learned with any toolbox: 
%  libSVM, SMVlight, EnsembleSVM, etc as long as the user creates a 
%  simplified struct model  with the following fields:
%
%  model.D      : Dat 
%  model.nClass : # of Classes (2 for binary)
%  model.nSV    : Total # of Support Vectors
%  model.b      : Offset for classification function
%  model.sigma  : Gaussian RBF kernel Width
%  model.yalphas: Values for the Lagrangian multipliers*Label per SVs [1xnSV]
%  model.SVs    : Set of Support Vectors                              [DxnSV]
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Load 36D Robot Self-Collision Dataset and model learned through libSVM
%% and training data, or a subset of it to test the code
clc; clear all; close all;
load('./models/NewIIWA-Setup-Feb18/NewIIWASetup-Feb18-SparseModel-2.mat')

%% Create Simplified Struct Model for SVMGrad from libSVM Model
svmgrad = [];
svmgrad.D       = size(X_train,2);
svmgrad.nSV     = model.totalSV;
svmgrad.b       = -model.rho;
svmgrad.sigma   = options.sigma;
svmgrad.yalphas = model.sv_coef'; %\alpha_*y_i
svmgrad.SVs     = full(model.SVs)';

%% Sample classifier and gradient evaluation for on query point
query_point = X_train(randi(length(X_train)),:)';
tic;
class       = calculateClass( svmgrad,  query_point)
value       = calculateGamma( svmgrad,  query_point)
gradient    = calculateGammaDerivative( svmgrad, query_point)
toc;

%% Write SVMGrad Struct to .txt file for C++ Usage
filename = './models/Innovation-Award/36D-12k-Optimal-Model-Innovation.txt';
writeSVMGrad(svmgrad, filename);

%% Write Testing Data for SVMGRad
filename = './models/Innovation-Award/36D-12k-Optimal-Data-Innovation.txt';
ntest    = 500;
randidx  = randperm(length(X_train));
x_test   = X_train(randidx(1:ntest),:)';
y        = zeros(1, ntest);
value    = zeros(1, ntest);
gradient = zeros(svmgrad.D, ntest);
for i=1:ntest
    query_point      = x_test(:,i);
    y(1,i)           = calculateClass( svmgrad,  query_point);
    value(1,i)       = calculateGamma( svmgrad,  query_point);
    gradient(:,i)    = calculateGammaDerivative( svmgrad, query_point);
end

writeSVMGradTestData(x_test, y, value, gradient, filename)