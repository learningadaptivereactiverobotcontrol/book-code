function [dir,nFlag]=solve_chol(obj,matrix,rhs)
% (Modified) Newton method via Cholesky factorization (with inertia control)
% Symmetric approximate minimum degree permutation is possible to employ
% Solves the system "matrix*dir=rhs"
% OUTPUT
%   nFlag ... 0..OK, 1..Cholesky/inertia failed
% 'obj' is used only for option settings (and to store last perturbation)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


  start_time=cputime;
  [n m] = size(matrix);
  mat_nnz=nnz(matrix);

  LMUPDATE = obj.allopts.pert_update;
  LMLOW = obj.allopts.pert_min;
  PERT_TRY_MAX = obj.allopts.pert_try_max;
  ORDERING = obj.allopts.chol_ordering;
  %LMFASTER = obj.allopts.pert_faster; % && issparse(matrix); ??
  LMFASTER = 0; % && issparse(matrix); ??

  if (ORDERING && issparse(matrix))
    if (~isempty(obj.chol_factperm))
      obj.print(4,Inf,'Reusing ordering');
      p=obj.chol_factperm;
    else
      obj.print(4,Inf,'Computing ordering');
      p=symamd(matrix);
      obj.chol_factperm=p;
    end
  else
    p=1:n;
  end
  rev(p)=1:n;  % inverse permutation

  matrix=matrix(p,p);
  [factor, status] = chol(matrix);
  chol_try=0;
  lambda=max(obj.chol_lmlast/LMUPDATE, LMLOW);  % lambda to try if it fails now
  lambda_last=0.;
  while (status~=0 && chol_try<PERT_TRY_MAX)
    obj.print(4,Inf,'Chol fact failed (%d), new pert: %e',status,lambda);
    if (LMFASTER)
      % find negative curvature vector from the last factorization
      %k = status-1;
      %R11 = factor(1:status-1,1:status-1);
      %R12 = factor(1:status-1,status:n);
      %v_nc = [R11 \ R12(:,1); -1; zeros(n-k-1,1)];
      R12e1 = factor(1:status-1,status);
      v_nc = [factor(1:status-1,1:status-1) \ R12e1; -1; zeros(n-status,1)];
      v_nc = v_nc/norm(v_nc);      % negative curvature vector
      % perturbated matrix must be positive definite, e.i., v_nc'*(matrix+lambda*I)*v_nc>0
      % it gives: lambda > -v_nc'*matrix*v_nc since v_nc is normed
      negcrv = -v_nc'*matrix*v_nc;
      cnt=0;
      while (lambda-lambda_last <= negcrv)
        lambda=lambda*LMUPDATE;
	    cnt=cnt+1;
      end
      obj.print(5,Inf,'Chol: pert_faster: skipped %d steps: pert=%e -> %e',cnt,lambda_last,lambda);
    end
    matrix=matrix + (lambda-lambda_last)*speye(n,n);
    [factor, status] = chol(matrix);
    chol_try = chol_try+1;
    lambda_last=lambda;
    lambda=lambda*LMUPDATE;
  end

  if (chol_try>0)
    obj.chol_lmlast=lambda_last;
  end

  % update statistics
  time_fact = cputime - start_time;
  obj.stats_time_fact_last=obj.stats_time_fact_last+time_fact;
  obj.initer_last = obj.initer_last+chol_try+1;

  if (status~=0)
    obj.print(3,Inf,'Chol fact failure (%d), last pert: %e, giving up',status,lambda_last);
    nFlag = 1;
    dir=zeros(n,1);
    return;
  end

  dir=factor \ (factor' \ rhs(p));
  dir=dir(rev);
  time_total=cputime - start_time;
  nFlag=0;

  obj.print(4,Inf,'Chol fact OK in %fs, total %fs, no pert=%i, pert=%.4e, nnz=%d (dim %dx%d)',time_fact, time_total, chol_try, lambda_last, nnz(factor),n,n);
  if (lambda_last>0)
    obj.print(3,4,'Chol fact: OK, pert=%.4e',lambda_last);
  end


