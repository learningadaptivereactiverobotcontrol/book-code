% Penalty for matrix constraints update
function  [] = mpen_update(obj)

  % add barrier penalty update as well...
  idx=obj.Yboxindbar;
  pnew=obj.PYbox;   % to get the right shape
  pnew(idx)=p_update(obj.PYbox(idx), obj.allopts.mpenalty_update, obj.allopts.mpenalty_border, obj.allopts.mpenalty_min);

  if ~isempty(obj.Yboxindbar)
  for k=obj.Yboxindbar
    Ykx = obj.Y{obj.Yboxmap(k)};
    Akx=obj.Yboxshift(k)*speye(size(Ykx)) + obj.Yboxmlt(k)*Ykx;

    %pkx=pnew(k); %obj.PA(k);
    [pnew(k), nfactor] = p_check(-Akx, pnew(k), obj.PYbox(k));
  end
  end
  obj.PYbox(idx) = max(pnew(idx));

  idx=obj.Yboxindphi;
  pnew=obj.PYbox;   % to get the right shape
  pnew(idx)=p_update(obj.PYbox(idx), obj.allopts.mpenalty_update, obj.allopts.mpenalty_border, obj.allopts.mpenalty_min);

  if ~isempty(obj.Yboxindphi)
  for k=obj.Yboxindphi
    Ykx = obj.Y{obj.Yboxmap(k)};
    Akx=obj.Yboxshift(k)*speye(size(Ykx)) + obj.Yboxmlt(k)*Ykx;

    %pkx=pnew(k); %obj.PA(k);
    [pnew(k), nfactor] = p_check(-Akx, pnew(k), obj.PYbox(k));
  end
  end
  obj.PYbox(idx) = max(pnew(idx));


  idx=obj.Aindphi;
  %pold=obj.PA;
  pnew=obj.PA;   % to get the right shape
  pnew(idx)=p_update(obj.PA(idx), obj.allopts.mpenalty_update, obj.allopts.mpenalty_border, obj.allopts.mpenalty_min);
     % p, pfactor, pborder, pmin

  % check that each matrix constraints is still feasible with the new penalty parameter
  if ~isempty(obj.Aindphi)
  for k=obj.Aindphi
    kuser=obj.Amap(k);
    [Akuserx, obj.userdata] = obj.mconfun(obj.x, obj.Y, kuser, obj.userdata);
    Akx = obj.Ashift(k)*speye(size(Akuserx)) + obj.Amlt(k) .* Akuserx;

    %pkx=pnew(k); %obj.PA(k);
    [pnew(k), nfactor] = p_check(-Akx, pnew(k), obj.PA(k));
  end
  end

  % use the minima for all
  obj.PA(idx) = max(pnew(idx));

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% penalty vector update according to the rules
%   p - vector of original penalty parameters
%   pfactor - update factor for "normal" operation
%   pborder - where to switch to slower update
%   pmin - minimal penalty which will be used
% OR do it up there??
function [p] = p_update(p, pfactor, pborder, pmin)
  idxnormal = p>pborder;

  p(idxnormal) = p(idxnormal)*pfactor;
  p(~idxnormal) = p(~idxnormal)*0.9;

  % don't go below pmin
  p = max(p,pmin);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% check if the new penalty parameter is usable for the matrix 
% constraints, i.e., check that M+p*I >=0 (positive semidefinite)
%   M - matrix of the constraint (M>=0) at the current point
%   p - new penalty parameter
%   pold - previous penalty parameter (to backtrack)
% returns
%   p - valid penalty argument
%   nfactor - how many times it was necessary to factorize
%     (nfactor>1 ==> p had to be shortened)
function [p, nfactor] = p_check(M, p, pold)

  rFactor=0.75;   % if need to refactorize, prefer new penalty parameter
  nfactor=0;

  [n m] = size(M);
  Missparse = n>10 && issparse(M) && nnz(M)<0.15*n*n;  

  if (Missparse)
    perm=amd(M);
    M=M(perm,perm);
    I=speye(n,n);
  else
    % it is usually faster to compute with dense matrices in dense format
    M=full(M);
    I=eye(n,n);
  end

  [R,k] = chol(M+p*I);
  nfactor=nfactor+1;
  if (k==0)
    % first match -> go
    return;
  end

  while (k~=0)
    p=rFactor*p + (1-rFactor)*pold;
    [R,k] = chol(M+p*I);
    %disp(sprintf('up   pert=%e (%i)',p,k));
    nfactor=nfactor+1;
    % add save bounds??
  end

end


