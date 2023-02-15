function [Purity NMI F] = cluster_metrics(true_labels, pred_labels)
% This function computes External Clustering metrics namely: 
% - Purity
% - Normalized Mutual Information (NMI) Criteria
% - F-measure
% used to evaluate clustering results wrt. true labels.
% For theoretical info for each metric refer to:
% - http://nlp.stanford.edu/IR-book/html/htmledition/evaluation-of-clustering-1.html
% - http://www.uni-weimar.de/medien/webis/teaching/lecturenotes/machine-learning/unit-en-cluster-analysis-evaluation.pdf
% Input: true_labels (Nx1 vector with thrue labels)
%        pred_labels (Nx1 vector with predicted labels)

% Output: NMI (Normalized Mutual Information Score 1: best, 0: worst)
%         Purity:     1(best), 0 (worst)
%         F-measure : 1(best), 0 (worst)
%
% Author: Nadia Figueroa, PhD Student., Robotics
% Learning Algorithms and Systems Lab, EPFL (Switzerland)
% Email address: nadia.figueroafernandez@epfl.ch  
% Website: http://lasa.epfl.ch
% February 2016; Last revision: 07-July-2016

N = length(true_labels);
true_labels = true_labels(:)';
pred_labels = pred_labels(:)';

%%%%%%% Computing Purity Criteria %%%%%%%
M = crosstab(true_labels,pred_labels); 
nc = sum(M,1);
mc = max(M,[],1);
Purity = sum(mc(nc>0))/sum(nc);


%%%%%%% Computing Normalized Mutual Information (NMI) Criteria %%%%%%%
% The NMI is a measures that allows us to make this tradeoff between 
% the quality of the clustering against the number of clusters

% NMI = I / (H_true + H_pred)/2
% I is the mutual information Criteria
% H_x is the Entropy

% Computing the entropy of the true labels
true_prob = zeros(max(true_labels),1);
H_true = 0;
for i = unique(true_labels)
    true_prob(i) = sum(true_labels == i) / N;
    H_true = H_true - true_prob(i) * log(true_prob(i));
end

% Computing the entropy of the predicted labels
pred_prob = zeros(max(pred_labels),1);
H_pred = 0;
for j = unique(pred_labels)
    pred_prob(j) = sum(pred_labels == j) / N;
    H_pred = H_pred - pred_prob(j) * log(pred_prob(j));
end

I = 0;
for i = unique(true_labels)
    for j = unique(pred_labels)
        joint_p = sum(true_labels == i & pred_labels == j) / N;
        if (joint_p > 0)
            I = I + joint_p * log(joint_p / (true_prob(i)*pred_prob(j)));
        end
    end
end

if (I == 0)
    NMI = 0;
else
    NMI=I/sqrt(H_pred*H_true);
end


%%%%%%% Computing F measure %%%%%%%
% The harmonic mean between Precision and Recall
% http://www.uni-weimar.de/medien/webis/teaching/lecturenotes/machine-learning/unit-en-cluster-analysis-evaluation.pdf
F = ml_Fmeasure(pred_labels, true_labels);

end