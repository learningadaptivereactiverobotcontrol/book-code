function [x_dir,ueq_dir,nFlag,pert]=solvekkt_ldl(obj,H,A,rhs1,rhs2)
% A direct solver based on LDL() function in Matlab (version 7.6, R2008a++)
% for indefinite KKT system [H,A; A',0]
% with inertia control and diagonal perturbations if needed
% INPUT
%   H, A  ... matrices nxn, nxm respectively forming the system K=[H,A;A',0]
%   rhs1,2... vectors nx1, mx1, right hand side
% OUTPUT
%   x_dir, ueq_dir ... primal & dual reached solution
%   nFlag ... flag (0..OK, 1..inertia control/ldl() failed)
%   pert ... optional parameter - used perturbation of (1,1) block to make
%      the kkt matrix inertia right
%
% 'obj' is used for option settings

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  start_time = cputime;
  [n m] = size(A);

  LMUPDATE = obj.allopts.pert_update;
  LMLOW = obj.allopts.pert_min;
  PERT_TRY_MAX = obj.allopts.pert_try_max;
  pivot = obj.allopts.ldl_pivot;

  % LDL() with inertia control
  block22=sparse(m,m);

  [L,D,perm] = ldl([H,A;A',block22],pivot,'vector');
  [e_pos, e_neg, e_zero] = inertia(D);

  pert_try=0;
  lambda=max(obj.chol_lmlast/LMUPDATE, LMLOW);  % lambda to try if it fails now
  lambda_last=0.;

  while ((e_zero>0 || e_neg>m) && pert_try<PERT_TRY_MAX)
    obj.print(4,Inf,'Wrong inertia: %d,%d,%d, new pert: %e',e_pos,e_neg,e_zero,lambda);
    H = H + (lambda-lambda_last)*speye(n,n);
    [L,D,perm] = ldl([H,A;A',block22],pivot,'vector');
    [e_pos, e_neg, e_zero] = inertia(D);

    pert_try = pert_try+1;
    lambda_last=lambda;
    lambda=lambda*LMUPDATE;
  end

  if (pert_try>0)
    obj.chol_lmlast=lambda_last;
  end

  % update statistics
  time_factor = cputime-start_time;
  obj.stats_time_fact_last=obj.stats_time_fact_last+time_factor;
  obj.initer_last = obj.initer_last+pert_try+1;
  if (nargout>=4)
    pert=lambda_last;
  end
  
  if (e_zero~=0 || e_neg~=m)
    obj.print(3,Inf,'Inertia control failure, wrong inertia: %d,%d,%d, last pert: %e',e_pos,e_neg,e_zero,lambda_last);
    nFlag = 1;
    x_dir=zeros(n,1);
    ueq_dir=zeros(m,1);
    return;
  end

  rhs=[rhs1;rhs2];
%  dir(perm)=L' \ (D \ (L \ rhs(perm))); % this somehow gives 1x(n+m)
%  vector instead of (n+m)x1, WHY ???
  dir=L' \ (D \ (L \ rhs(perm)));
  dir(perm)=dir;
  x_dir=dir(1:n);
  ueq_dir=dir(n+1:n+m);
  time_total=cputime - start_time;
  nFlag=0;

  obj.print(4,Inf,'LDL KKT OK, factor in %fs, total %fs, no pert=%i, pert=%.4e',time_factor, time_total, pert_try, lambda_last);
  if (lambda_last>0)
    obj.print(3,4,'LDL KKT solve: OK, pert=%.4e',lambda_last);
  end

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [e_pos, e_neg, e_zero] = inertia(D)
% returns the inertia triplet of matrix D, expects D to be a diagonal/block
% diagonal symmetric matrix with at most 2-times-2 blocks on the diagonal

  % simple but perhaps ?slow? way how to do it
  [dim dim2] = size(D);
  e=eig(D);
  e_pos=sum(e>0);
  e_neg=sum(e<0);
  e_zero=dim-e_pos-e_neg;

  return;

  % faster?? but more complicated way - split it to 1-by-1 / 2-by-2 blocks
  e_pos=0;
  e_neg=0; 

  i=1;
  while (i<=dim)
    if (i<dim && D(i,i+1))  % 2-by-2 block
      e=eig(D(i:i+1,i:i+1));
      e_pos = e_pos + sum(e>0);
      e_neg = e_neg + sum(e<0);
      i=i+2;
    else           % 1-by-1 block
      e_pos = e_pos + D(i,i)>0;
      e_neg = e_neg + D(i,i)<0;
      i=i+1;
    end
  end
  e_zero=dim-e_pos-e_neg;

end

