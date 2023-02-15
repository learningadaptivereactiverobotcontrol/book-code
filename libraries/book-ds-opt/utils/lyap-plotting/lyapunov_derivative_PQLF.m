function [lyap_der] = lyapunov_derivative_PQLF(x, att, P, ds_fun)

[~,M]    = size(x);
% Variables
xd       = feval(ds_fun,x);
lyap_der = zeros(1,M);
for i = 1:M
    % Gradient of global component
    grad_lyap = P*(x(:,i) - att) + P'*(x(:,i) - att);

    % Computing full derivative
    lyap_der(1,i) = xd(:,i)' * (grad_lyap);
end
end