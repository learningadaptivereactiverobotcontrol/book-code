% Initialize lagrangian multipliers, penalty parameters etc.
% If called without argument (or with forceupdate=true), restores default
% settings for all (not only for mis-shaped or empty items).
%
% TODO: if not forced and wrong --> warning?
% TODO: force transposing if column vector?
% problem with vectors 0x1 --> changed it, if empty then 0x0 size
function [] = init(obj, forceupdate)
  
  if (nargin<2)
    forceupdate = true;
  end

  % function constraints: Lagr. multipliers & penalty
  if (forceupdate || ~isvector(obj.uxbox) || length(obj.uxbox)~=obj.Nxbox)
    if (obj.Nxbox>0)
      obj.uxbox = obj.allopts.uinit_box*ones(obj.Nxbox,1);
    else
      obj.uxbox = [];
    end
  end
  obj.uxboxnew = ones(obj.Nxbox,1);

  if (forceupdate || ~isvector(obj.uineq) || length(obj.uineq)~=obj.Nineq)
    if (obj.Nineq>0)
      obj.uineq = obj.allopts.uinit*ones(obj.Nineq,1);
    else
      obj.uineq = [];
    end
  end
  obj.uineqnew = ones(obj.Nineq,1);

  if (forceupdate || ~isvector(obj.ueq) || length(obj.ueq)~=obj.Neq)
    if (obj.Neq>0)
      obj.ueq = obj.allopts.uinit_eq*ones(obj.Neq,1);
    else
      obj.ueq = [];
    end
  end

  if (forceupdate || ~isvector(obj.pxbox) || length(obj.pxbox)~=obj.Nxbox)
    if (obj.Nxbox>0)
      obj.pxbox = obj.allopts.pinit*ones(obj.Nxbox,1);
    else
      obj.pxbox = [];
    end
  end

  if (forceupdate || ~isvector(obj.pineq) || length(obj.pineq)~=obj.Nineq)
    if (obj.Nineq>0)
      obj.pineq = obj.allopts.pinit*ones(obj.Nineq,1);
    else
      obj.pineq = [];
    end
  end

  % matrix constraints: NYbox, NA 
  % MATRIX variables (Y) TODO   don't forget obj.PY...!!
  if (forceupdate || ~isvector(obj.PYbox) || length(obj.PYbox)~=obj.NYbox)
    if (obj.NYbox>0)
      % same as below, should be length(Ybox used in Phi, not barrier)
      pnew=1;  % TODO get me from the option settings!!
      for k=1:obj.NYbox   %obj.Yboxindphi
        kuser=obj.Yboxmap(k);
        Ykuserx=obj.Y{kuser};
        Ykx = obj.Yboxshift(k)*speye(size(Ykuserx)) + obj.Yboxmlt(k) .* Ykuserx;

        %pkx=pnew(k); %obj.PA(k);
        [pnew, nfactor] = p_check2(-Ykx, pnew);
      end

      % use the minima for all  TODO shoudln't it be maxima????
      obj.PYbox=pnew*ones(obj.NYbox,1);

    else
      obj.PYbox = [];
    end
  end

  % perhpas in addition to these, it should be tested if PA is sufficiently large and increase it as necessary...
  if (forceupdate || ~isvector(obj.PA) || length(obj.PA)~=obj.NA)
    if (obj.NA>0)
      % TODO this should be infact length(obj.Aindphi)>0 rather than obj.NA>0
      % needs to be set up big enough that the point is feasible
      % start with one from option settins and assign the biggest one to all
      pnew=1;  % TODO get me from the option settings!!
      for k=obj.Aindphi
        kuser=obj.Amap(k);
        [Akuserx, obj.userdata] = obj.mconfun(obj.x, obj.Y, kuser, obj.userdata);
        Akx = obj.Ashift(k)*speye(size(Akuserx)) + obj.Amlt(k) .* Akuserx;

        %pkx=pnew(k); %obj.PA(k);
        [pnew, nfactor] = p_check2(-Akx, pnew);
      end

      % use the minima for all  TODO shoudln't it be maxima????
      obj.PA=pnew*ones(obj.NA,1);
    else
      obj.PA=[];
    end
  end

  % force update always... (just for now)
  if (forceupdate || isempty(obj.UA))
    obj.UA = [];
    obj.UAnew = [];
    for k=1:obj.NA
      kuser=obj.Amap(k);
      [Akuserx, obj.userdata] = obj.mconfun(obj.x, obj.Y, kuser, obj.userdata);
      obj.UA{k} = eye(size(Akuserx));
      % does this get used???
      obj.UAnew{k} = eye(size(Akuserx));
    end
  end

  if (forceupdate || isempty(obj.UYbox))
    obj.UYbox = [];
    obj.UYboxnew = [];
    for k=1:obj.NYbox
      kuser=obj.Yboxmap(k);
      obj.UYbox{k} = eye(size(obj.Y{kuser}));
      obj.UYboxnew{k} = eye(size(obj.Y{kuser}));
    end
  end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO this is the same as from mpen_update ... put extra if needed!
% the difference is that there is no parameter to backtrack ...
%  ... probably just double it every time??
%  ... original Pennon adjusts rUpdate to control how fast it goes down
%
% check if the new penalty parameter is usable for the matrix 
% constraints, i.e., check that M+p*I >=0 (positive semidefinite)
%   M - matrix of the constraint (M>=0) at the current point
%   p - new penalty parameter
%   xxxxpold - previous penalty parameter (to backtrack)
% returns
%   p - valid penalty argument
%   nfactor - how many times it was necessary to factorize
%     (nfactor>1 ==> p had to be shortened)
function [p, nfactor] = p_check2(M, p)

  rSafetyFactor = 1.2;  % add 20% to the initial penalty to be on the safe side
  %rSafetyFactor = 1;
  rFactor=2;   % if need to refactorize, try rFactor*p
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

  p = p/rSafetyFactor;

  [R,k] = chol(M+p*I);
  nfactor=nfactor+1;
  if (k==0)
    % first match -> go
    return;
  end

  while (k~=0)
    p=rFactor*p;
    [R,k] = chol(M+p*I);
    %disp(sprintf('up   pert=%e (%i)',p,k));
    nfactor=nfactor+1;
    % add save bounds??
  end

  p = p*rSafetyFactor;

end




