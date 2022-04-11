function [vel] = lmds_2d(original_dynamics, exp_funct, query_pos, phi_c)

% Compute original dynamics at query points
vel     = feval(original_dynamics, query_pos);
dim     = size(vel,1);
samples = size(vel,2);

% Compute Activation Values
h_x     = feval(exp_funct, query_pos');

display('Modulating Dynamics through Local Rotation');
% and modulate
vel = locally_rotate_2d(vel, h_x', phi_c);

end