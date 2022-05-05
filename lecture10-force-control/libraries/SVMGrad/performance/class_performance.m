function [stats] = class_performance(y_test,y_est)

% Matlab confusion matrix function
[C, order] = confusionmat(y_test, y_est, 'order', [1 -1]);

% Confusion Matrix Values
TP = C(1,1); FN = C(1,2);
FP = C(2,1); TN = C(2,2);

% Accuracy
ACC = (TP + TN ) / (TP + FP + FN + TN);

% F1 Score
F1  = 2*TP / (2*TP + FP + FN);

% Type 1 Error (Fall-out)
FPR = (FP) / (FP + TN);

% True Positive Rate (Sensitivity)
TPR = (TP) / (TP + FN);

% True Negative Rate (Specificity)
TNR = (TN) / (TN + FP);

stats.ACC = ACC;
stats.F1  = F1;
stats.FPR = FPR;
stats.TPR = TPR;
stats.TNR = TNR;
end