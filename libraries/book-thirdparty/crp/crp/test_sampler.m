% Copyright October, 2006, Brown University, Providence, RI. 
% All Rights Reserved 

% Permission to use, copy, modify, and distribute this software and its
% documentation for any purpose other than its incorporation into a commercial
% product is hereby granted without fee, provided that the above copyright
% notice appear in all copies and that both that copyright notice and this
% permission notice appear in supporting documentation, and that the name of
% Brown University not be used in advertising or publicity pertaining to
% distribution of the software without specific, written prior permission. 

% BROWN UNIVERSITY DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
% INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR ANY
% PARTICULAR PURPOSE. IN NO EVENT SHALL BROWN UNIVERSITY BE LIABLE FOR ANY
% SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER
% RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF
% CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
% CONNECTION WITH THE USE.

% Author: Frank Wood fwood@cs.brown.edu.

% create Gaussian mixture model
g1 = gaussian([1 1]', .1*eye(2));
g2 = gaussian([-1 -1]', .1*eye(2));
g3 = gaussian([1 -1]', .1*eye(2));
g4 = gaussian([-1 1]', .1*eye(2));
gmm = gaussian_mixture_model(1/4,g1,1/4,g2,1/4,g3,1/4,g4);

% create training data by sampling from the Gaussian mixture model
N = 500;
[training_data, training_data_class] = sample(gmm,N);
class_label = unique(training_data_class);

% show the training data
figure('Color',[1 1 1])
plot_mixture(training_data,training_data_class)
title('Training Data', 'Interpreter','LaTex', 'FontSize',20)

%% run the CRP sampler to generate the posterior distribution over model 
% parameters
iterations = 200;
[class_id, mean_record, covariance_record, K_record, lP_record, alpha_record] = sampler(training_data, iterations);
[val , Maxiter]  = max(lP_record);
est_labels       = class_id(:,Maxiter);

%% show the clustered data
figure('Color',[1 1 1])
subplot(2,1,1)
semilogx(1:iterations, lP_record'); hold on;
semilogx(Maxiter,lP_record(Maxiter),'ko','MarkerSize',10);
grid on
xlabel('Gibbs Iteration','Interpreter','LaTex','Fontsize',20); ylabel('LogPr','Interpreter','LaTex','Fontsize',20)
xlim([1 iterations])
legend({'$p(Z|Y, \alpha, \lambda)$'},'Interpreter','LaTex','Fontsize',14)
title(sprintf('DP-GMM Sampling results, optimal K=%d at iter=%d', length(unique(est_labels)), Maxiter), 'Interpreter','LaTex','Fontsize',20)

subplot(2,1,2)
stairs(K_record, 'LineWidth',2);
set(gca, 'XScale', 'log')
xlim([1 iterations])
xlabel('Gibbs Iteration','Interpreter','LaTex','Fontsize',20); ylabel('$\Psi$ = Estimated K','Interpreter','LaTex','Fontsize',20);