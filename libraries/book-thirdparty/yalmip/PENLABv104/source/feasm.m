% Compute a perturbation needed for a given symmetric matrix to be pos. def.
% I.e., compute a lower estimate of the smallest (negative) eigenvalue (if any).
% Input:
%   M ... symmetric matrix to find 'pert>=0' so that M+pert*I>0 (pos.def.)
%   lb ... (>=0) lower bound on pert, i.e., find pert>=lb>=0, 
%          e.g., current known feasibility of other matrix constraints
%   ub ... (>0, Inf if not known), upper bound on pert which is known that
%          M+ub*I>0, typically the penalty parameter
%   pstart ... if lb==0, pstart is the first perturbation to try if M not posdef
%   pstop ... tolerance of the requested perturbation, the exact perturbation
%          will be in interval [pert,pert+ptol]
% Output:
%   pert ... estimated perturbation within the given tolerance unless
%          M+lb*I>=0 is already pos def
%
% TODO
%   how to make pstart&ptol dynamic to save some time...
function [pert] = feasm(M, lb, ub, pstart, pstop)

  nfactor=0;

  % Get ordering if M really sparse & shuffle M to avoid recomputing
  [n m] = size(M);
  Missparse = n>10 && issparse(M) && nnz(M)<0.15*n*n;  
     % nnz() used directly and behind issparse() because otherwise it 
     % counts nonzero elements even in the dense matrix

  if (Missparse)
    perm=amd(M);
    M=M(perm,perm);
    I=speye(n,n);
  else
    % it is usually faster to compute with dense matrices in dense format
    M=full(M);
    I=eye(n,n);
  end

  % first quick try
  pert=max(0.0, lb);
  [R,p] = chol(M+pert*I);
  %pert
  nfactor=nfactor+1;
  if (p==0)
    % first match -> go
    return;
  end

  % guess a reasonable step how to increase the perturbation
  if (ub<Inf)
    % perturbation with ub should be pos. def; with 'pert' wasnt --> pert<ub
    %pstep=min(20,max((ub-pert)/4, 2));
    pstep=min(50,max(1.01*realpow(ub/pert,1/4),2));
      % in 4 steps jump above the known perturbation
  else
    pstep=10;
  end
  %pstep
  pertlowstart = pert;
  pert=max(pert,pstart/pstep);

  while (p~=0)
    pertlow = pert;
    pert=pert*pstep;
    [R,p] = chol(M+pert*I);
    %disp(sprintf('up   pert=%e (%i)',pert,p));
    nfactor=nfactor+1;
  end
  pertup=pert;

  if (nfactor==2)
    % this is probably not necessary but it the upper loop finishes after one
    % step, perlow might be actually higher than the one which failed in the
    % "first quick try"
    pertlow=pertlowstart;
  end
  %nfactor

  pstopn = max(pstop,norm(pertlow)*pstop);
  
  while (pertup-pertlow>pstopn)
    %pert=pertlow + (pertup-pertlow)/3;
    pert=(pertlow+pertup)/2;
    %pert=realsqrt(pertlow*pertup);
    [R,p] = chol(M+pert*I);
    %disp(sprintf('down pert=%e (%i)',pert,p));
    nfactor=nfactor+1;
    if (p==0)
      pertup=pert;
    else
      pertlow=pert;
    end
  end
  pert = pertup;   % return upper bound on the perturbation (=overestimate)
  % (=lower bound on the (negative) eigenvalue)

  %nfactor
end

