function [vel,varargout] = gp_mds_2d(original_dynamics, gp_handle, demonstrated_data, query_pos)

[angle_est, var] = feval(gp_handle, demonstrated_data(1:2,:)', demonstrated_data(3,:)', query_pos');
speed_est = feval(gp_handle, demonstrated_data(1:2,:)', demonstrated_data(4,:)', query_pos');
%angle_est = gp(hyp, @infExact, {@meanZero},{@covSEiso}, @likGauss, demonstrated_data(1:2,:)', demonstrated_data(3,:)', query_pos');
%speed_est = gp(hyp, @infExact, {@meanZero},{@covSEiso}, @likGauss, demonstrated_data(1:2,:)', demonstrated_data(4,:)', query_pos');
% make sure gp does not cause spurious attractors
speed_est = max(speed_est, -0.9);
% compute original dynamics at query points
vel = feval(original_dynamics, query_pos);
% and modulate
% Force scaling
scale = 1;
vel = scale*locally_rotate_and_scale_2d(vel, angle_est, speed_est);

if nargout >1
    varargout{1} = eye(2)*var;
end

end