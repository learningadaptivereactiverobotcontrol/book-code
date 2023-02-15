function [g,userdata] = ex72_confun(x,Y,userdata)
% Example 7.2, problem (17) from PENNON user's guide

  % 
  for i=1:length(Y{1})
      g(i,1)=x(1)*Y{1}(i,i);
  end

