function [dg, userdata]=ex72_congrad(x,Y,userdata)
% Example 7.2, problem (17) from PENNON user's guide
% returns all constraints at once
% rectangule matrix (Nx+NYnnz) x Ng

  %
  n =length(Y{1});
  nn = n*(n+1)/2;
  idiag = userdata.idiag;
  
  dg = sparse((nn+1),n); % #variables x #constraints
  dg(1,:) = diag(Y{1});
  
  for i=1:n
      dg(idiag(i)+1,i) = x(1);
  end
  
  

