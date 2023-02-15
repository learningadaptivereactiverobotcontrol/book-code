function [df, userdata]=ncm2y_objgrad(x,Y,userdata)
% return gradient of the objective w.r.t. all variables (even matrix)
% return NYnnz x 1

  % obj = sum(x_ij - h_ij)^2
  % d/dx_ij obj = 2*(x_ij-h_ij)
  % and we go only over lower triangle --> get indices which belong only to it
  idx = find(tril(ones(size(Y{1}))));
  YH = full(Y{1}-userdata);
  df = 2*YH(idx);

