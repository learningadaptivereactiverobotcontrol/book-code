function [x_dot] = lin_ds(x, target, A)

x_dot = A*(x-target);

end