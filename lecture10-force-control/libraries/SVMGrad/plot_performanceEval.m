%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate Performance Computation plots for SVM models of increasing
% complexity, models: 
% 0: 1.6kSVs (0.5% Data),1: 2.7kSVs (1% Data), 2: 3.5kSVs (2% Data)
% 3: 8.9kSVs (5%  Data), 4: 15kSVs  (10% Data) from 1 million points.
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; clc; close all;
%% Load full dataset (Train and Test)
load('./models/Fender/Fender-Collision-Avoidance-Dataset.mat')

%% Load models
model_names = dir('./models/Granularity-Tests/Fender-optimal-model-*.mat'); 
nModels = length(model_names);
models  = {};
for i=1:nModels
    models{i,1} = load(strcat('./models/Granularity-Tests/',model_names(i).name));
end
%% Compute Error Rates for each model on test set
% numSamples  = round(length(y_test));
numSamples  = 10000;
model_sizes = zeros(1,nModels);    
ACC = zeros(1,nModels);
F1  = zeros(1,nModels);
FPR = zeros(1,nModels);
TPR = zeros(1,nModels);
clc;
for i =  1:nModels    
    test_model = models{i}.model;
    model_sizes(1,i) = models{i}.model.totalSV;  
    
    % Extract a random testing points
    y_test_   = zeros(1,numSamples);
    y_est_    = zeros(1,numSamples);
    idx_rand  = randperm(numSamples);
    
    % Evaluate on Testing Points   
    X_test_ = X_test(1:numSamples,:);
    y_test_ = y_test(1:numSamples,:);
    
    % Test Learnt Model
    [y_est_] = svm_classifier(X_test_, y_test_, [], test_model);
    
    % Compute Classifier Test Stats
    [stats] = class_performance(y_test_,y_est_);
    fprintf('*Classifier Performance for N_sv=%d on Test Set (%d points)* \n Acc: %1.5f, F-1: %1.5f, FPR: %1.5f, TPR: %1.5f \n', test_model.totalSV, length(y_test_), stats.ACC, stats.F1, stats.FPR, stats.TPR)
    ACC(1,i) = stats.ACC; F1(1,i) = stats.F1;
    FPR(1,i) = stats.FPR; TPR(1,i) = stats.TPR;
    
end

%% Plot of model performance stats on test set
figure('Color',[1 1 1])
train_sizes = [13000 27000 40000 54000 135000];    
[ax,hline1,hline2]=plotyy([train_sizes' train_sizes' train_sizes' train_sizes'],[ACC' F1' (1-FPR)' TPR'], train_sizes', 2*model_sizes');
delete(hline1);
delete(hline2);
hold(ax(1),'on');
plot(train_sizes, ACC, '--oc','LineWidth',2,'MarkerFaceColor','c'); hold on;
plot(train_sizes, F1,  '--vk','LineWidth',2,'MarkerFaceColor','k'); hold on;
plot(train_sizes, 1-FPR, '--dr','LineWidth',2,'MarkerFaceColor','r'); hold on;
plot(train_sizes, TPR, '--sm','LineWidth',2,'MarkerFaceColor','m'); hold on;
ylim([0.94 1])
% xlim([11000 140000])
% set(gca,'xscale','log')

hold(ax(2),'on');
plot(ax(2),train_sizes, model_sizes, '--dg','LineWidth',2, 'MarkerFaceColor','g'); hold on;

xlabel('Training Set Size','Interpreter','Latex','FontSize',20);
ylabel('Performance Measure','Interpreter','Latex','FontSize',20)
% set(gca,'xscale','log')
% xlim([11000 140000])
legend({'ACC','F1','1-FPR','TPR','$N_{sv}$'},'Interpreter','Latex', 'FontSize',14)
grid on


%% Plot of model performance with decreasing granularity at sampling
figure('Color',[1 1 1])
sampling_interval = [20 22 25 30 45];    
dataset_sizes     = [5400000 2300000 870000 240000 8900];
[ax,hline1,hline2]=plotyy([sampling_interval' sampling_interval' sampling_interval' sampling_interval'],[ACC' F1' (1-FPR)' TPR'], sampling_interval', dataset_sizes');
delete(hline1);
delete(hline2);
hold(ax(1),'on');
plot(sampling_interval, ACC, '--oc','LineWidth',2,'MarkerFaceColor','c'); hold on;
plot(sampling_interval, F1,  '--vk','LineWidth',2,'MarkerFaceColor','k'); hold on;
plot(sampling_interval, 1-FPR, '--dr','LineWidth',2,'MarkerFaceColor','r'); hold on;
plot(sampling_interval, TPR, '--sm','LineWidth',2,'MarkerFaceColor','m'); hold on;
% ylim([0.5 1])
% xlim([11000 140000])
% set(gca,'xscale','log')

hold(ax(2),'on');
plot(ax(2),sampling_interval, dataset_sizes, '--dg','LineWidth',2, 'MarkerFaceColor','g'); hold on;

xlabel('Joint Sampling Interval [$\deg$]','Interpreter','Latex','FontSize',20);
ylabel('Performance Measure','Interpreter','Latex','FontSize',20)
set(gca,'xscale','log')
% xlim([11000 140000])
legend({'ACC','F1','1-FPR','TPR','M'},'Interpreter','Latex', 'FontSize',14)
grid on
