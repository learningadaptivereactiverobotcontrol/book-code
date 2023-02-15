function [df, userdata]=ex72_objgrad(x,Y,userdata)
% Example 7.2, problem (17) from PENNON user's guide
% return gradient of the objective w.r.t. all variables

  % 
  YH=svec2(x(1).*Y{1}-userdata.H);
  
  df(1) = sum(2*svec2(Y{1}).*YH);
  df(2:length(YH)+1) = 2*x(1).*YH;
  
  df = df';