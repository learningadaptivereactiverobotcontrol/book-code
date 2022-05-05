function y = rbf_2d(x,weights,centers,sigma)
    % Compute relative distance squared between x and each kernel
    d = sum((centers(:,1:2)-x(1:2)').^2,2);
 
    % Evaluate kernels
    values = exp(-d./(2*sigma*sigma));
    if(sum(values)<eps)
        y = 0;
    else
        % Compute weighted average
        y = weights'*real(values./sum(values));
    end
end

