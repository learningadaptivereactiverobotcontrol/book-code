function [g,userdata] = ncm1y_confun(x,Y,userdata)
% only one constraints: tr(Y)=1, return value of the body of the cunstraint
% thus tr(Y)
% vector Ng x 1

  g=trace(Y{1});

