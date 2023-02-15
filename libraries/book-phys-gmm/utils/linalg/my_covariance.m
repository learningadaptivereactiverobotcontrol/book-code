function [ Sigma ] = my_covariance( X, X_bar, type )
%MY_COVARIANCE computes the covariance matrix of X given a covariance type.
%
% Inputs -----------------------------------------------------------------
%       o X     : (N x M), a data set with M samples each being of dimension N.
%                          each column corresponds to a datapoint
%       o X_bar : (N x 1), an Nx1 matrix corresponding to mean of data X
%       o type  : string , type={'full', 'diag', 'iso'} of Covariance matrix
%
% Outputs ----------------------------------------------------------------
%       o Sigma : (N x N), an NxN matrix representing the covariance matrix of the 
%                          Gaussian function
%%

% Auxiliary Variable
[N, M] = size(X);

% Output Variable
Sigma = zeros(N, N);

switch type
        case 'full'  % Equation 6   

            % Zero-mean Data if M > 1
            if M > 1
                X = bsxfun(@minus, X, X_bar);
                Sigma = (1/(M-1))*X*X';
            else
                Sigma = (1/(M))*X*X';
            end
            % OR
            % Sigma = cov(X');
            
        case 'diag' % Equation 6  + diag()
            
            % Zero-mean Data if M > 1
            if M > 1
                X = bsxfun(@minus, X, X_bar);
                Sigma = (1/(M-1))*X*X';
            else
                Sigma = (1/(M))*X*X';
            end                                    
            Sigma = diag(diag(Sigma));

        case 'iso'    % Equation 7         
            sqr_dist = sum((X - repmat(X_bar,1,M)).^2,1);                                
            c_var    = sum(sqr_dist) ./ M ./ N;
            Sigma    = eye(N) .* c_var;

        otherwise
            warning('Unexpected Covariance type. No Covariance computed.')
end

% Add a tiny variance to avoid numerical instability
% Sigma = Sigma + 1E-5.*diag(ones(N,1));


end