function [ddgk, userdata]=ex72_conhess(x,Y,k,userdata)
% Example 7.2, problem (17) from PENNON user's guide

  %
  n =length(Y{1});
  nn = n*(n+1)/2;
  idiag = userdata.idiag;
  
  ddgk = zeros((nn+1),(nn+1));
  ddgk(1,1) = 1;
  ddgk(idiag(k)+1,idiag(k)+1) = 1;



  
  

