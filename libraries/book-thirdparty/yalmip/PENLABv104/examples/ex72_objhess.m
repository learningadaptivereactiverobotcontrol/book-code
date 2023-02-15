function [ddf, userdata] = ex72_objhess(x,Y,userdata)
% Example 7.2, problem (17) from PENNON user's guide
% Hessians of the objective function and constraints
  
  YH=packmat(x(1).*Y{1}-userdata.H);
  yy = packmat(Y{1});
  n = length(yy);
  ddf = zeros(n+1,n+1);
  
  ddf(1,1) = 2*sum(yy.^2);
  ddf(1,2:n+1) = 2.*(x(1).*yy+YH);
  ddf(2:n+1,1) = 2.*(x(1).*yy'+YH');
  for i= 1:n
      ddf(i+1,i+1) = 2*x(1)^2;
  end



