function [g,userdata] = ncm2y_confun(x,Y,userdata)
% only one constraints: tr(Y)=1, return value of the body of the cunstraint
% thus tr(Y)
% vector Ng x 1

  g=diag(Y{1});

