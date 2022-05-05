function grad = grad_rbf_2d(x,weights,centers,sigma)

    % Compute relative distance squared between x and each kernel
    d = sum((centers(:,1:2)-x(1:2)').^2,2);
 
    % Evaluate kernels
    values = exp(-d./(2*sigma*sigma));
    
    % Compute gradient
    if(sum(values)<eps)
        grad = zeros(size(weights));
    else
        grad = values./sum(values);
    end
end