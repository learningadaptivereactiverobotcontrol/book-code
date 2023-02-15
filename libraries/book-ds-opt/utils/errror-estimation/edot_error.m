function trajectory_edot = edot_error(ds_fun, xi_ref, xi_dot_ref)
xi_dot = feval(ds_fun, xi_ref);
[D, N] = size(xi_dot);
trajectory_edot = zeros(1,N);
for i=1:N
    trajectory_edot(1,i) = abs(1 - xi_dot(:,i)'*xi_dot_ref(:,i) / (norm(xi_dot(:,i))*norm(xi_dot_ref(:,i))));
    if isnan(trajectory_edot(1,i))
        trajectory_edot(1,i) = 0;
    end
end
end

