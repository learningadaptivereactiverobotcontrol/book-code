function [lyap_fun] = lyapunov_function_PQLF(x, att, P)

% Variables           
[~,M]    = size(x);
lyap_fun = zeros(1,M);

for i = 1:M
    lyap_fun(1,i) = (x(:,i) - att)'*P*(x(:,i) - att);
end

end