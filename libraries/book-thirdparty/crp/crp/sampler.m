function [class_id, mean_record, covariance_record, K_record, lP_record, alpha_record] = sampler(training_data, num_sweeps, a_0, b_0, mu_0, k_0, v_0, lambda_0)
% function [class_id, mean_record, covariance_record, K_record, lP_record,
%           alpha_record] = sampler(training_data, num_sweeps, a_0, b_0, 
%                                   mu_0, k_0, v_0, lambda_0)
%
%   Generates samples from the infinite Gaussian mixture model posterior. 
% The model parameter samples returned are the class_id's assigned to each 
% data point, the mean_record which is a record of all of the class means, 
% the covariance_record which is a record of all of the Gaussian density 
% covariances.  Also returned are K_record, lP_record, and alpha_record
% which are the the record of K, the number of mixture densities, the log
% probability of the training data under the model, and the distribution
% over the Dirichlet hyperparameter alpha.
%
%  Input variables are the training_data (Dims x #points), the number of
% sampler sweeps, the gamma prior on alpha parameters a_0 and b_0, and
% the inversh-wishart/normal hyperparameters mu_0, k_0, v_0, lambda_0
% which conform to Gelman's notation.

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

% throughout the code there are chunks that are commented out -- this is
% to enhance efficiency in most cases.

if(nargin < 2)
    num_sweeps = 1000;
end

% show trace plots and current clustering (2D)?
GRAPHICS = 0;

% set normal inverse wishart hyper parameters
if(nargin < 5)
    mu_0 = zeros(size(training_data(:,1)));
end
if(nargin < 6)
    k_0 = 1;
end
if(nargin < 7)
    v_0 = size(training_data(:,1),1);
end
if(nargin < 8)
    lambda_0 = eye(size(training_data(:,1),1))*.3;
end

% set alpha gamma prior parameters
if(nargin < 3)
    a_0 = 0.1;
%     a_0 = 1;
end
if(nargin <4)
    b_0 = 2;
end

% initialize
alpha = 10;
N = size(training_data,2);
y = training_data;
% phi = cell(num_sweeps,1);
D = size(training_data,1);
mean_record = cell(num_sweeps,1);
covariance_record = cell(num_sweeps,1);
inv_covariance_record = cell(num_sweeps,1);
K_plus = 1;
class_id = zeros(N,num_sweeps);
K_record = zeros(num_sweeps,1);
alpha_record = zeros(num_sweeps,1);
lP_record = zeros(num_sweeps,1);

% seat the first customer at the first table
class_id(:,1) = 1;
% phi{1} = {gaussian(y(:,1),eye(size(y,1)))};
mean_record{1} = y(:,1);
covariance_record{1} = zeros(D,D,1);
covariance_record{1}(:,:,1) = eye(size(y,1));
inv_covariance_record{1} = zeros(D,D,1);
inv_covariance_record{1}(:,:,1) = eye(size(y,1));

max_lP = 0;
break_counter = 0;

% compute the log likelihood
lp = lp_crp(class_id(:,1),alpha);
for(k=1:K_plus)
    class_k_datapoint_indexes = find(class_id(:,1)==k);
    %     lp= lp + sum(fvnlp(y(:,class_k_datapoint_indexes),...
    %         phi{1}{k}.mean,phi{1}{k}.covariance));
    lp= lp + sum(fvnlp(y(:,class_k_datapoint_indexes),...
        mean_record{1}(:,k),covariance_record{1}(:,:,k)));
    %     mu = phi{1}{k}.mean;
    %     sigma = phi{1}{k}.covariance;
    mu = mean_record{1}(:,k);
    sigma = covariance_record{1}(:,:,k);
    lp = lp + lpnormalinvwish(mu,sigma,mu_0,k_0,v_0,lambda_0);
    %         lp = lp + palpha();
end
lP_record(1) = lp;


% run the Gibbs sampler
for(sweep = 2:num_sweeps)
    %disp(['Sweep ' num2str(sweep) '/' num2str(num_sweeps)])
    %     phi{sweep} = phi{sweep-1};
    mean_record{sweep} = mean_record{sweep-1};
    covariance_record{sweep} = covariance_record{sweep-1};
    inv_covariance_record{sweep} = inv_covariance_record{sweep-1};
    class_id(:,sweep) = class_id(:,sweep-1);

    % for each datapoint, unseat it and reseat it, potentially generating a
    % new table
    for(i=1:N)
        m_k = zeros(K_plus,1);

        % compute the CRP prior
        for(k=1:K_plus)
            if(k>K_plus)
                break;
            end
            m_k(k) = length(find(class_id(:,sweep)==k));
            if(class_id(i,sweep)==k)
                m_k(k) = m_k(k)-1;
                if(m_k(k)==0)
                    % delete
                    %                     temp = phi{sweep};
                    %                     phi{sweep} = temp([1:k-1 k+1:end]);

                    mean_record{sweep} = mean_record{sweep}(:,[1:k-1 k+1:end]);
                    covariance_record{sweep} = covariance_record{sweep}(:,:,[1:k-1 k+1:end]);
                    inv_covariance_record{sweep} = inv_covariance_record{sweep}(:,:,[1:k-1 k+1:end]);
                    for(j=k:K_plus)
                        change_inds = find(class_id(:,sweep)==j);
                        class_id(change_inds,sweep) = j-1;
                    end
                    K_plus = K_plus-1;
                    m_k = m_k(1:end-1);
                end
            end
        end

        % sneakily add on the new table generation prior prob to the vec.
        prior = [m_k; alpha]/(N-1+alpha);
        likelihood = zeros(length(prior)-1,1);
        %         temp = phi{sweep};

        % compute the per class data likelihood p(x_i|g_i,\Theta)
        for(l = 1:length(likelihood))

            %             likelihood(l) = lp(temp{l},y(:,i));
            %                 [p lp] = p(temp{l},y(:,i));
            %                 lp = fvnlp(y(:,1),temp{l}.mean, temp{l}.covariance);
            %                 lp = fvnlp(y(:,i),mean_record{sweep}(:,l),covariance_record{sweep}(:,:,l));
            l1 = y(:,i)-mean_record{sweep}(:,l);
            lp = -l1'*inv_covariance_record{sweep}(:,:,l)*l1/2-.5*log(det(covariance_record{sweep}(:,:,l)))-(D/2)*log(2*pi);
            likelihood(l) = lp;
        end

        % do Monte Carlo integration to get prob. of new table
        % current code uses one sample njk=1:1 to estimate integrand
        new_table_likelihood = 0;
        %         for(njk=1:1)
        new_table_covariance = iwishrnd(lambda_0, v_0);
        new_table_mean = mvnrnd(mu_0',new_table_covariance'/k_0)';
        %             new_table_density = gaussian(new_table_mean,new_table_covariance);

        %             new_table_likelihood =
        %             new_table_likelihood+lp(new_table_density,y(:,i));
        %             [p lp] = p(new_table_density,y(:,i));
        l1 = y(:,i)-new_table_mean;
        inverse_new_table_covariance = inv(new_table_covariance);
        new_table_likelihood = new_table_likelihood-l1'*inverse_new_table_covariance*l1/2-.5*log(det(new_table_covariance))-(D/2)*log(2*pi);
        %             new_table_likelihood = new_table_likelihood+fvnlp(y(:,i),new_table_mean, new_table_covariance);
        %         end

        %         new_table_likelihood = new_table_likelihood/1;
        likelihood = [likelihood; new_table_likelihood];
        likelihood = exp(likelihood-max(likelihood));%/sum(exp(likelihood-max(likelihood)));
        likelihood = likelihood/sum(likelihood);
        if(sum(likelihood==0))
            likelihood(end) = eps;
        end
        % compute the posterior over seating assignment for datum i
        temp = prior.*likelihood;
        temp = temp/sum(temp);

        cdf = cumsum(temp);
        rn = rand;
        new_class_id = min(find((cdf>rn)==1));
        %         picker = multinomial(temp);
        %         new_class_id = sample(picker,1);

        % if the new table was chosen, add it to the list of tables a
        % datum can sit at
        %         temp = phi{sweep};
        if(new_class_id == K_plus+1)
            %             phi{sweep} = {temp{:}, new_table_density};

            mean_record{sweep} = [mean_record{sweep} new_table_mean];
            covariance_record{sweep}(:,:,new_class_id) =  new_table_covariance;
            inv_covariance_record{sweep}(:,:,new_class_id) =  inv(new_table_covariance);
            K_plus = K_plus+1;
        end

        % record the new table
        class_id(i,sweep) = new_class_id;

    end

    % flag to set whether to use a normal inverse wishart prior
    % or a Jeffries non-informative prior...  the Jeffries prior is not
    % tested
    NIW = 1;

    %     temp = phi{sweep};
    for(k=1:K_plus)
        class_k_datapoint_indexes = find(class_id(:,sweep)==k);
        N_this_class = length(class_k_datapoint_indexes);
        S = cov(y(:,class_k_datapoint_indexes)')'*(N_this_class-1);
        y_bar = mean(y(:,class_k_datapoint_indexes)')';
        mu_n = (N_this_class*y_bar+...
            k_0*mu_0)/(k_0+N_this_class);
        if(NIW)
            lambda_n = lambda_0 + ...
                (k_0*N_this_class)/(k_0+N_this_class)*...
                ((y_bar-mu_0)*(y_bar-mu_0)')+S;

            k_n = k_0 + N_this_class;
            v_n = v_0 + N_this_class;
            % sample the new covariance and mean from the joint posterior
            new_covariance = iwishrnd(lambda_n, v_n);
            new_mean = mvnrnd(mu_n',new_covariance'/k_n)';
        else
            new_covariance = iwishrnd(S,N_this_class-1);
            new_mean = mvnrnd(mean(y(:,class_k_datapoint_indexes)')',...
                new_covariance'/N_this_class)';
        end
        mean_record{sweep}(:,k) = new_mean;
        covariance_record{sweep}(:,:,k) = new_covariance;
        inv_covariance_record{sweep}(:,:,k) = inv(new_covariance);
        %         temp(k) =  {gaussian(new_mean,new_covariance)};
    end
    %     phi{sweep} = temp;



    METROPOLISALPHA = 1;
    % I could not get the Gibbs sampler for alpha to work.
    if(~METROPOLISALPHA)
    %sample alpha
        nu = betarnd(alpha+1,N);
        a = 1;
        b = 1;
        % this is the same as eqn. 14 of Escobar and West 1994 Bayesian
        % Density Estimation and Inference Using Mixtures
        pia = (a+K_plus-1)/((a+K_plus-1)+(b-log(nu))*N)
        if(rand < pia)
            alpha = gamrnd(a+K_plus,b-log(nu))
        else
            alpha = gamrnd(a+K_plus-1,b-log(nu))
        end
    end

    if(METROPOLISALPHA)
        % use a metropolis step to sample alpha
        lp = lp_crp(class_id(:,sweep),alpha);
        for(k=1:K_plus)
            class_k_datapoint_indexes = find(class_id(:,sweep)==k);
            %         lp= lp + sum(fvnlp(y(:,class_k_datapoint_indexes),phi{sweep}{k}.mean,phi{sweep}{k}.covariance));
            lp= lp + sum(fvnlp(y(:,class_k_datapoint_indexes),mean_record{sweep}(:,k),covariance_record{sweep}(:,:,k)));
            %         mu = phi{sweep}{k}.mean;
            %         sigma = phi{sweep}{k}.covariance;
            mu = mean_record{sweep}(:,k);
            sigma = covariance_record{sweep}(:,:,k);
            lp = lp + lpnormalinvwish(mu,sigma,mu_0,k_0,v_0,lambda_0);
            lp = lp + -  gamlike([a_0 b_0],alpha);
        end

        alpha_prop = alpha + randn*.1;

        if(alpha_prop < 0)
            lp_alpha_prop = -Inf;
        else
            lp_alpha_prop = lp_crp(class_id(:,sweep),alpha_prop);
            lp_alpha_prop = lp_alpha_prop -  gamlike([a_0 b_0],alpha_prop);
        end
        log_acceptance_ratio = lp_alpha_prop - lp;
        if(log(rand)<min(log_acceptance_ratio,0))
            alpha = alpha_prop;
        end
    end

    % record the current parameters values
    K_record(sweep) = length(temp);
    alpha_record(sweep) = alpha;
    lP_record(sweep) = lp;
    if (lP_record(sweep) > max_lP)
        max_lP = lP_record(sweep);
        break_counter = 0;
    else
        break_counter = break_counter + 1;
        if break_counter >= 500
            alpha_record(sweep+1:end) = [];
            class_id(:,sweep+1:end) = [];
            covariance_record(sweep+1:end) = [];
            K_record(sweep+1:end) = [];
            lP_record(sweep+1:end) = [];
            mean_record(sweep+1:end) = [];
            break
        end
    end
    
    if(GRAPHICS)
        figure(3)
        subplot(3,1,1)
        plot(1:sweep,lP_record(1:sweep));
        title('Log P')
        subplot(3,1,2)
        plot(1:sweep,K_record(1:sweep));
        title('K');
        subplot(3,1,3)
        plot(1:sweep,alpha_record(1:sweep));
        title('alpha');


        figure(2)
        %         plot_mixture(training_data(1:2,1:32:end),class_id(1:32:end,sweep))
        plot_mixture(training_data(1:2,:),class_id(:,sweep))
        drawnow
        %     pause(1)
    end


end

