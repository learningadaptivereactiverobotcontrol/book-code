function [f,userdata] = ex72_objfun(x,Y,userdata)
% Example 7.2, problem (17) from PENNON user's guide
  % matrix H is stored in userdata

  YH = svec2(x(1).*Y{1}-userdata.H);
  f = YH(:)'*YH(:);

