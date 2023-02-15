function trajectory_RMSE = rmse_error(ds_fun, xi_ref, xi_dot_ref)
xi_dot = feval(ds_fun, xi_ref);
trajectory_RMSE = sqrt(mean((xi_dot' - xi_dot_ref').^2, 2))';
end

