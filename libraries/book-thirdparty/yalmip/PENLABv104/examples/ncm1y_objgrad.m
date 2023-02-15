function [df, userdata]=ncm1y_objgrad(x,Y,userdata)
% return gradient of the objective w.r.t. all variables (even matrix)
% return NYnnz x 1

  % hard-code me for the moment
  YH=Y{1}-userdata;
  df = [2*YH(1,1); 4*YH(2,1); 2*YH(2,2); 4*YH(3,2); 2*YH(3,3)];

