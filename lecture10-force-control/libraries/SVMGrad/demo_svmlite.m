%% SVM-light MEX-interface
tic
model_svmlight = svmlearn(X, labels,'-t 2 -c 100 -g 50');
toc
%% Make SVM-light model to SVMGrad

svmgrad_light         = [];
svmgrad_light.D       = size(X,2);
svmgrad_light.nSV     = model_svmlight.sv_num-1;
svmgrad_light.b       = -model_svmlight.b;
svmgrad_light.sigma   = options.sigma;
svmgrad_light.yalphas = model_svmlight.alpha(2:model_svmlight.sv_num)'; %\alpha_*y_i
svmgrad_light.SVs     = model_svmlight.supvec(2:end,:)';

%% Visualize Decision Function and gradients (Only for 2d dataset)
plot_svmgrad_boundary(X, labels, svmgrad_light,  'draw');

%% Sample classifier and gradient evaluation for on query point
query_point = X_train(randi(length(X_train)),:)';
tic;
class       = calculateClass( svmgrad_light,  query_point)
value       = calculateGamma( svmgrad_light,  query_point)
gradient    = calculateGammaDerivative( svmgrad_light, query_point)
toc;
